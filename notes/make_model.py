"""
Turn the Go2 robot description into two static files the browser can load fast.

Run this ONCE on the laptop. It writes into notes/dashboard_static/:

    go2_model.json   the link tree: bodies, joint axes, mesh offsets, colours
    go2_meshes.bin   all mesh geometry, packed binary

Why not just serve the .dae or .obj files and parse them in the browser?
Because they are 25 MB of XML/text between them, and base.dae alone is 11 MB.
Parsing that in JavaScript on every page load, served by a 4-core Jetson with
no GPU, is the wrong trade. MuJoCo has already parsed these meshes for the
simulator, so we read the vertex and face arrays straight out of the compiled
model and write them as binary. No OBJ or COLLADA parser is needed anywhere --
not here, and not in the browser.

Geometry comes from go2.xml (MuJoCo) and the kinematics are cross-checked
against go2_description.urdf, which is authoritative for the real robot. The
script fails loudly if the two disagree about a joint axis, because a leg
bending the wrong way is the failure mode we most want to catch early.

    source ~/unitree_ws/venv-rl/bin/activate
    python3 make_model.py
"""
import json
import os
import struct
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

GO2_XML = os.path.expanduser("~/unitree_ws/sim/unitree_mujoco/unitree_robots/go2/go2.xml")
URDF = os.path.expanduser(
    "~/unitree_ws/src/unitree_ros/robots/go2_description/urdf/go2_description.urdf")
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dashboard_static")

# ---------------------------------------------------------------------------
# THE JOINT ORDER AND AXIS CONVENTION -- the one place it is written down.
#
# The Go2 SDK indexes its twelve motors 0..11 as:
#
#     0  FR_hip   1  FR_thigh   2  FR_calf
#     3  FL_hip   4  FL_thigh   5  FL_calf
#     6  RR_hip   7  RR_thigh   8  RR_calf
#     9  RL_hip  10  RL_thigh  11  RL_calf
#
# F/R is front/rear, R/L is right/left. So low_state.motor_state[4].q is the
# front-LEFT thigh. Both go2.xml (actuators, jointpos sensors) and this file
# use that order; dashboard.py and dashboard_static/app.js repeat it.
#
# Axes, from the URDF and confirmed against go2.xml below:
#     hip    rotates about X  (1 0 0) -- swings the leg out sideways
#     thigh  rotates about Y  (0 1 0) -- swings the leg fore/aft
#     calf   rotates about Y  (0 1 0) -- bends the knee
#
# All angles are radians, right-handed about the stated axis, in the parent
# link's frame. That is what motor_state[i].q holds, so the browser can apply
# it directly as a rotation about the axis with no sign flips. There are no
# sign flips anywhere in this pipeline, by design -- if the model ever looks
# mirrored, the bug is in the link tree, not in a missing minus sign.
# ---------------------------------------------------------------------------
MOTOR_ORDER = [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
]
FEET = ["FR", "FL", "RR", "RL"]     # order of low_state.foot_force


def urdf_joint_axes(path):
    """Read the twelve revolute joint axes out of the URDF, for cross-checking."""
    root = ET.parse(path).getroot()
    axes = {}
    for j in root.findall("joint"):
        if j.get("type") != "revolute":
            continue
        a = j.find("axis")
        if a is None:
            continue
        axes[j.get("name")] = [float(v) for v in a.get("xyz").split()]
    return axes


def check_axes(model, urdf_axes):
    """Fail loudly if MuJoCo and the URDF disagree about any joint axis."""
    problems = []
    for name in MOTOR_ORDER:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            problems.append("%s: not present in go2.xml" % name)
            continue
        mj = np.array(model.jnt_axis[jid], dtype=float)
        ur = np.array(urdf_axes.get(name, []), dtype=float)
        if ur.size != 3:
            problems.append("%s: not present in the URDF" % name)
        elif not np.allclose(mj, ur, atol=1e-6):
            problems.append("%s: go2.xml axis %s vs URDF axis %s" % (name, mj, ur))
        else:
            print("  ok  %-16s axis %s" % (name, mj))
    if problems:
        raise SystemExit("joint axis mismatch:\n  " + "\n  ".join(problems))


def main():
    if not os.path.exists(GO2_XML):
        raise SystemExit("not found: %s" % GO2_XML)
    model = mujoco.MjModel.from_xml_path(GO2_XML)

    print("cross-checking joint axes, go2.xml against the URDF")
    check_axes(model, urdf_joint_axes(URDF))

    os.makedirs(OUT_DIR, exist_ok=True)

    # --- pack every unique mesh once -------------------------------------
    # Positions float32, normals int8 (they are unit vectors, so a byte per
    # component is plenty and saves 1.7 MB), indices uint16 -- the largest
    # mesh has 46,564 vertices, comfortably under 65,536.
    verts, norms, idx = bytearray(), bytearray(), bytearray()
    meshes = {}
    for i in range(model.nmesh):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, i)
        vadr, vnum = model.mesh_vertadr[i], model.mesh_vertnum[i]
        fadr, fnum = model.mesh_faceadr[i], model.mesh_facenum[i]
        if vnum > 65535:
            raise SystemExit("%s has %d vertices, too many for uint16 indices"
                             % (name, vnum))
        # OBJ files can index positions and normals separately, but MuJoCo's
        # compiler unifies them: mesh_normalnum == mesh_vertnum and
        # mesh_facenormal == mesh_face. That is what lets us write one
        # interleaved-by-index vertex stream below. Assert it rather than
        # assume it -- getting this wrong corrupts shading in a way that is
        # easy to mistake for a lighting problem.
        if model.mesh_normalnum[i] != vnum:
            raise SystemExit("%s: %d normals for %d vertices -- the model no "
                             "longer uses unified indexing"
                             % (name, model.mesh_normalnum[i], vnum))
        if not np.array_equal(model.mesh_face[fadr:fadr + fnum],
                              model.mesh_facenormal[fadr:fadr + fnum]):
            raise SystemExit("%s: face and normal indices differ" % name)

        v = np.array(model.mesh_vert[vadr:vadr + vnum], dtype=np.float32)
        n = np.array(model.mesh_normal[vadr:vadr + vnum], dtype=np.float32)
        f = np.array(model.mesh_face[fadr:fadr + fnum], dtype=np.uint16)

        meshes[name] = {
            "voff": len(verts) // 12, "vcount": int(vnum),
            "ioff": len(idx) // 2, "icount": int(fnum) * 3,
        }
        verts += v.tobytes()
        norms += np.clip(np.round(n * 127.0), -127, 127).astype(np.int8).tobytes()
        idx += f.tobytes()

    # Concatenate into one file, four-byte aligned, offsets recorded in the JSON.
    blob = bytearray()
    sections = {}
    for key, buf in (("positions", verts), ("normals", norms), ("indices", idx)):
        while len(blob) % 4:
            blob += b"\0"
        sections[key] = {"byteOffset": len(blob), "byteLength": len(buf)}
        blob += buf

    # --- the link tree ----------------------------------------------------
    # One entry per MuJoCo body, in index order, so a child always follows its
    # parent and the browser can build the hierarchy in a single pass.
    jnt_by_body = {}
    for j in range(model.njnt):
        jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if jname in MOTOR_ORDER:
            jnt_by_body[int(model.jnt_bodyid[j])] = (jname, j)

    bodies = []
    for b in range(model.nbody):
        bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or "body%d" % b
        entry = {
            "name": bname,
            "parent": int(model.body_parentid[b]),
            "pos": [float(v) for v in model.body_pos[b]],
            "quat": [float(v) for v in model.body_quat[b]],   # w,x,y,z
            "joint": None,
            "geoms": [],
        }
        if b in jnt_by_body:
            jname, j = jnt_by_body[b]
            entry["joint"] = {
                "name": jname,
                "motor": MOTOR_ORDER.index(jname),
                "axis": [float(v) for v in model.jnt_axis[j]],
                "pos": [float(v) for v in model.jnt_pos[j]],
                "range": [float(v) for v in model.jnt_range[j]],
            }
        bodies.append(entry)

    # Visual geoms only. Collision shapes in go2.xml are capsules and spheres
    # (including the four foot contact spheres), so filtering to mesh geoms
    # drops them all.
    for g in range(model.ngeom):
        if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mesh_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH,
                                      int(model.geom_dataid[g]))
        matid = int(model.geom_matid[g])
        rgba = model.mat_rgba[matid] if matid >= 0 else model.geom_rgba[g]
        bodies[int(model.geom_bodyid[g])]["geoms"].append({
            "mesh": mesh_name,
            "pos": [float(v) for v in model.geom_pos[g]],
            "quat": [float(v) for v in model.geom_quat[g]],
            "color": [float(v) for v in rgba],
        })

    # Where to draw the four foot-load pads. go2.xml puts a body named
    # <FOOT>_foot at the contact point of each leg.
    feet = []
    for i, f in enumerate(FEET):
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "%s_foot" % f)
        if bid < 0:
            raise SystemExit("body %s_foot not found in go2.xml" % f)
        feet.append({"name": f, "index": i, "body": bid,
                     "pos": [float(v) for v in model.body_pos[bid]]})

    doc = {
        "note": "generated by notes/make_model.py -- do not edit by hand",
        "sections": sections,
        "meshes": meshes,
        "bodies": bodies,
        "feet": feet,
        "motor_order": MOTOR_ORDER,
    }

    with open(os.path.join(OUT_DIR, "go2_meshes.bin"), "wb") as fh:
        fh.write(blob)
    with open(os.path.join(OUT_DIR, "go2_model.json"), "w") as fh:
        json.dump(doc, fh, separators=(",", ":"))

    print("\nwrote %s" % OUT_DIR)
    print("  go2_meshes.bin   %.1f MB  (%d meshes)" % (len(blob) / 1e6, len(meshes)))
    print("  go2_model.json   %d bodies, %d visual geoms, 12 joints"
          % (len(bodies), sum(len(b["geoms"]) for b in bodies)))


if __name__ == "__main__":
    main()
