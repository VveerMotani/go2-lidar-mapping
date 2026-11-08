"""Long-running work: download a walk, convert it, save a copy.

Each job runs in its own thread and outlives the page: closing the window
does not stop a conversion, and reopening it shows where it has got to.
Conversions run one at a time (walk_to_bag.sh refuses a second one anyway,
because two at once corrupt each other); downloads and copies run beside
them.

Progress for a conversion is read from the scripts' own output. They already
say what they are doing -- "4. decoding the LiDAR", "frame 1333 of about
2332", "1600 scans, 1096 matched" -- so nothing had to change in them for
Studio to show a progress bar. If a script's wording changes, the bar just
moves in bigger steps; the conversion itself is unaffected.
"""
import collections
import json
import os
import re
import shutil
import subprocess
import threading
import time
import urllib.request
import uuid
import zipfile

from . import paths, robot, settings, walks, winos
from .applog import log

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
_lock = threading.Lock()
JOBS = collections.OrderedDict()          # id -> Job, newest last
_convert_queue = []
_convert_wake = threading.Event()
HISTORY = os.path.join(paths.DATA, "jobs.json")


class Cancelled(Exception):
    pass


class Job:
    def __init__(self, kind, walk, stages, **opts):
        self.id = uuid.uuid4().hex[:8]
        self.kind = kind                  # download | convert | map | export
        self.walk = walk
        self.stages = stages              # e.g. ["bag", "map", "dense"]
        self.opts = opts
        self.state = "queued"             # queued running done failed cancelled
        self.stage = stages[0]
        self.stage_pct = 0.0
        self.stage_t0 = None
        self.eta = None
        self.note = ""
        self.error = None
        self.created = time.time()
        self.started = None
        self.finished = None
        self.engine = None
        self.cmd = None
        self.lines = collections.deque(maxlen=6000)
        self.seq = 0
        self.proc = None
        self.cancel_requested = False

    # -- progress ---------------------------------------------------------
    @property
    def pct(self):
        if self.state == "done":
            return 100.0
        i = self.stages.index(self.stage) if self.stage in self.stages else 0
        return round((i + self.stage_pct / 100.0) / len(self.stages) * 100.0, 1)

    def set_stage(self, stage, pct=0.0):
        if stage != self.stage:
            log("inf", "job", "%s %s: %s done, now %s" % (self.kind, self.walk, self.stage, stage))
            self.stage, self.stage_t0 = stage, time.time()
        self.progress(pct)

    def progress(self, pct):
        self.stage_pct = max(0.0, min(100.0, pct))
        if self.stage_t0 is None:
            self.stage_t0 = time.time()
        spent = time.time() - self.stage_t0
        self.eta = (spent * (100 - self.stage_pct) / self.stage_pct
                    if self.stage_pct >= 4 and spent > 3 else None)

    def line(self, text):
        self.seq += 1
        self.lines.append((self.seq, time.time(), text))

    def public(self):
        return {"id": self.id, "kind": self.kind, "walk": self.walk,
                "stages": self.stages, "stage": self.stage,
                "stage_pct": round(self.stage_pct, 1), "pct": self.pct,
                "eta": round(self.eta) if self.eta else None, "state": self.state,
                "note": self.note, "error": self.error, "engine": self.engine,
                "cmd": self.cmd, "created": self.created, "started": self.started,
                "finished": self.finished, "lines": self.seq}


# ------------------------------------------------------------------ queue
def _add(job):
    with _lock:
        JOBS[job.id] = job
        while len(JOBS) > 60:
            JOBS.popitem(last=False)
    log("inf", "job", "queued %s %s (%s)" % (job.kind, job.walk, job.id))
    return job


def active_for(walk):
    with _lock:
        return [j for j in JOBS.values() if j.walk == walk and j.state in ("queued", "running")]


def get(job_id):
    with _lock:
        return JOBS.get(job_id)


def listing():
    with _lock:
        return [j.public() for j in reversed(JOBS.values())]


def cancel(job_id):
    j = get(job_id)
    if j is None or j.state not in ("queued", "running"):
        return False
    j.cancel_requested = True
    log("wrn", "job", "cancel requested: %s %s" % (j.kind, j.walk))
    if j.state == "queued":
        j.state, j.finished = "cancelled", time.time()
        return True
    threading.Thread(target=_stop_process, args=(j,), daemon=True).start()
    return True


def _finish(job, state, error=None):
    job.state, job.error, job.finished = state, error, time.time()
    lvl = {"done": "ok", "cancelled": "wrn"}.get(state, "err")
    took = job.finished - (job.started or job.finished)
    log(lvl, "job", "%s %s %s in %d s%s" % (job.kind, job.walk, state, took,
                                           (": " + error) if error else ""))
    _save_history()


def _save_history():
    try:
        os.makedirs(paths.DATA, exist_ok=True)
        with open(HISTORY, "w", encoding="utf-8") as fh:
            json.dump([j.public() for j in JOBS.values() if j.state not in ("queued", "running")],
                      fh, indent=1)
    except OSError:
        pass


# ---------------------------------------------------------------- engines
def engine_status():
    """Everything the Settings page needs to say what is missing, and fix it.

    step is the one thing to do next: ready | install-docker | start-docker |
    get-image | wsl-missing.
    """
    choice = settings.get("engine")
    distro = settings.get("wsl_distro")
    image = settings.get("docker_image")
    wsl = choice in ("auto", "wsl") and winos.wsl_distro_ok(distro) and os.path.isdir(paths.NOTES)
    out = {"choice": choice, "image": image, "distro": distro, "wsl": wsl,
           "docker_installed": False, "docker_running": False, "image_present": False,
           "image_created": None, "docker_exe": winos.docker_desktop_exe()}
    if wsl:
        out.update(kind="wsl", detail=distro, step="ready")
        return out
    if choice == "wsl":
        out.update(kind=None, detail="WSL distro '%s' not found" % distro, step="wsl-missing")
        return out
    installed, running, present = winos.docker_state(image)
    out.update(docker_installed=installed, docker_running=running, image_present=present)
    if present:
        out["image_created"] = winos.image_created(image)
    if not installed:
        out.update(kind=None, step="install-docker", detail="Docker Desktop is not installed")
    elif not running:
        out.update(kind=None, step="start-docker",
                   detail="Docker Desktop is not running: start it and wait for 'Engine running'")
    elif not present:
        out.update(kind=None, step="get-image",
                   detail="the map-making engine is not downloaded yet: Settings, Map-making engine")
    else:
        out.update(kind="docker", detail=image, step="ready")
    return out


def engine():
    """('wsl', distro) or ('docker', image) or (None, why)."""
    s = engine_status()
    return s["kind"], s["detail"]


def _command(job, walk_dir):
    kind, what = engine()
    if kind is None:
        raise RuntimeError(what)
    job.engine = kind
    map_only = job.kind == "map"
    if kind == "wsl":
        script = paths.wsl_path(os.path.join(paths.NOTES, "process_walk.sh"))
        target = paths.wsl_path(walk_dir)
        sh = "exec bash '%s' '%s'%s%s" % (script, target, " --map-only" if map_only else "",
                                            " --colour" if job.opts.get("colour") else "")
        job.cancel_cmd = ["wsl.exe", "-d", what, "--", "pkill", "-TERM", "-f", target]
        return ["wsl.exe", "-d", what, "--", "bash", "-lc", sh]
    name = "go2studio-" + job.id
    job.cancel_cmd = ["docker", "stop", "-t", "20", name]
    return ["docker", "run", "--rm", "--init", "--name", name,
            "-v", "%s:/walks" % walks.root(), what,
            "map" if map_only else "process", job.walk] + (["--colour"] if job.opts.get("colour") else [])


def _stop_process(job):
    cmd = getattr(job, "cancel_cmd", None)
    if cmd:
        winos.run(cmd, timeout=40)
    p = job.proc
    if p and p.poll() is None:
        try:
            p.wait(timeout=25)
        except subprocess.TimeoutExpired:
            p.kill()


# ------------------------------------------------------------ conversion
STEP = re.compile(r"^\s*(\d{1,2})\.\s")
FRAME = re.compile(r"frame (\d+) of about (\d+)")
DEPTH_TOTAL = re.compile(r"depth\.stream:\s*(\d+) depth")
DEPTH_DONE = re.compile(r"(\d+) depth frames written")
SCANS_TOTAL = re.compile(r"(\d+) scans to align")
SCANS = re.compile(r"^\s*(\d+) scans,")
DENSE_STEPS = ("left out", "seen by fewer", "seen through", "statistical", "outside", "kept ")
# The scripts' own progress lines (notes/progress.py): "~ what: 600/1500 unit",
# or "~ what" for a phase with no count.
PROG = re.compile(r"^\s*~ (.+?)(?::\s*(\d+)/(\d+)(?:\s+(.*))?)?\s*$")
# Where each named part sits in the "clean cloud" stage, which also holds the
# floor plans (step 9) and the mesh (step 10). A part not listed moves nothing
# but still shows in the line under the walk.
DENSE_PARTS = [
    ("placing scans", 2, 34), ("adding the depth camera", 34, 42), ("cleaning the cloud", 42, 42),
    ("colouring the cloud", 50, 84), ("writing the clean cloud", 84, 87),
    ("drawing floor plans", 87, 91), ("building the mesh surface", 91, 96),
    ("colouring the mesh", 96, 97), ("writing the mesh", 97, 98), ("checking the mesh", 98, 99),
]


class _Parser:
    """Turns process_walk.sh's output into stage and percent."""

    def __init__(self, job):
        self.j = job
        self.step = 0
        self.depth_total = 0
        self.scans_total = 0
        self.dense_seen = 0
        self.last_bad = None

    def feed(self, text):
        j = self.j
        m = STEP.match(text)
        if m:
            self.step = int(m.group(1))
            if self.step <= 6 and "bag" in j.stages:
                j.set_stage("bag", {1: 2, 2: 5, 3: 8, 4: 10, 5: 75, 6: 96}.get(self.step, 0))
            elif self.step == 7:
                j.set_stage("map", 1)
            elif self.step >= 8:
                j.set_stage("dense", j.stage_pct if j.stage == "dense" else 1)
            j.note = text.strip().split(". ", 1)[-1]
            return
        m = PROG.match(text)
        if m:
            self._counted(*m.groups())
            return
        if text.lstrip().startswith("X "):
            self.last_bad = text.strip()[2:]
        if "decoding again" in text:
            j.note = "the laptop was too busy; decoding again at the walk's own pace"
            j.set_stage("bag", 10)
        if self.step == 4:
            m = FRAME.search(text)
            if m and int(m.group(2)):
                j.progress(10 + 62 * min(1.0, int(m.group(1)) / int(m.group(2))))
        elif self.step == 5:
            m = DEPTH_TOTAL.search(text)
            if m:
                self.depth_total = int(m.group(1))
            m = DEPTH_DONE.search(text)
            if m and self.depth_total:
                j.progress(75 + 20 * int(m.group(1)) / self.depth_total)
        elif self.step == 7:
            m = SCANS_TOTAL.search(text)
            if m:
                self.scans_total = int(m.group(1))
            m = SCANS.match(text)
            if m and self.scans_total:
                j.progress(2 + 93 * int(m.group(1)) / self.scans_total)
            if "wrote" in text and "_map.ply" in text:
                j.progress(100)
        elif self.step == 8:
            # the cleaning steps print one line each, between the depth camera
            # and the colour
            if any(k in text for k in DENSE_STEPS):
                self.dense_seen += 1
                self._up(min(50, 42 + 2 * self.dense_seen))

    def _up(self, pct):
        """Forward only: a later, rougher estimate never moves the bar back."""
        if pct > self.j.stage_pct:
            self.j.progress(pct)

    def _counted(self, what, done, total, unit):
        j = self.j
        if done is not None and int(total):
            j.note = "%s: %s of %s%s" % (what, done, total, " " + unit if unit else "")
        else:
            j.note = what
        for name, lo, hi in DENSE_PARTS:
            if what == name and j.stage == "dense":
                frac = int(done) / int(total) if done is not None and int(total) else 0.0
                self._up(lo + (hi - lo) * min(1.0, frac))
                return


def _stream(job, proc, parser, logfile):
    buf = b""
    while True:
        chunk = proc.stdout.read1(4096) if hasattr(proc.stdout, "read1") else proc.stdout.read(4096)
        if not chunk:
            break
        buf += chunk
        # Progress lines end in \r, finished ones in \n: treat both as ends.
        parts = re.split(rb"[\r\n]", buf)
        buf = parts.pop()
        for raw in parts:
            text = ANSI.sub("", raw.decode("utf-8", "replace")).rstrip()
            if not text:
                continue
            job.line(text)
            parser.feed(text)
            # The decode counter rewrites one line ten times a second; the
            # file keeps the other lines, the page still sees every one.
            if not FRAME.search(text):
                logfile.write(text + "\n")
    if buf.strip():
        text = ANSI.sub("", buf.decode("utf-8", "replace")).rstrip()
        job.line(text)
        parser.feed(text)
        logfile.write(text + "\n")


def _run_convert(job):
    d = walks.folder(job.walk)
    if d is None or not os.path.isdir(d):
        raise RuntimeError("no walk folder for %s" % job.walk)
    cmd = _command(job, d)
    job.cmd = " ".join(('"%s"' % c) if " " in c else c for c in cmd)
    log("inf", "job", "%s %s with %s: %s" % (job.kind, job.walk, job.engine, job.cmd))
    job.line("$ " + job.cmd)
    parser = _Parser(job)
    with open(os.path.join(d, "studio-convert.log"), "w", encoding="utf-8") as logfile:
        logfile.write("$ %s\n" % job.cmd)
        job.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    stdin=subprocess.DEVNULL, creationflags=winos.NO_WINDOW)
        _stream(job, job.proc, parser, logfile)
        code = job.proc.wait()
    if job.cancel_requested:
        raise Cancelled()
    if code != 0:
        raise RuntimeError(parser.last_bad or "the conversion stopped with exit code %d" % code)
    if job.kind == "convert" and settings.get("keep") == "maps":
        shutil.rmtree(os.path.join(d, "bag"), ignore_errors=True)
        job.line("bag removed after converting (Settings: keep maps only)")


def _convert_worker():
    while True:
        _convert_wake.wait()
        while True:
            with _lock:
                job = _convert_queue.pop(0) if _convert_queue else None
                if job is None:
                    _convert_wake.clear()
                    break
            if job.state != "queued":
                continue
            job.state, job.started, job.stage_t0 = "running", time.time(), time.time()
            try:
                _run_convert(job)
                _finish(job, "done")
            except Cancelled:
                _finish(job, "cancelled")
            except Exception as e:                          # noqa: BLE001
                _finish(job, "cancelled" if job.cancel_requested else "failed", str(e))


threading.Thread(target=_convert_worker, daemon=True, name="convert-worker").start()


def convert(walk, map_only=False, colour=None):
    for j in active_for(walk):
        if j.kind in ("convert", "map", "download"):
            return None, "already %s" % ("queued" if j.state == "queued" else "running")
    stages = ["map", "dense"] if map_only else ["bag", "map", "dense"]
    job = _add(Job("map" if map_only else "convert", walk, stages,
                   colour=settings.get("colour") if colour is None else bool(colour)))
    with _lock:
        _convert_queue.append(job)
    _convert_wake.set()
    return job, None


# --------------------------------------------------------------- download
def download(walk, expected_bytes=0, then_convert=False, colour=None):
    if walks.folder(walk) is None:
        return None, "bad walk name"
    if os.path.isdir(walks.folder(walk)):
        return None, "%s is already on this laptop" % walk
    for j in active_for(walk):
        return None, "already busy with %s" % j.kind
    job = _add(Job("download", walk, ["download", "unpack"],
                   expected=expected_bytes, then_convert=then_convert, colour=colour))
    threading.Thread(target=_run_download, args=(job,), daemon=True).start()
    return job, None


def _run_download(job):
    job.state, job.started, job.stage_t0 = "running", time.time(), time.time()
    root = walks.root()
    tmpdir = os.path.join(root, ".studio-downloads")
    zpath = os.path.join(tmpdir, job.walk + ".zip.part")
    unpack = os.path.join(tmpdir, job.walk + ".unpack")
    try:
        os.makedirs(tmpdir, exist_ok=True)
        url = robot.download_url(job.walk)
        job.cmd = "GET " + url
        job.line(job.cmd)
        got, t0 = 0, time.time()
        with urllib.request.urlopen(url, timeout=30) as r, open(zpath, "wb") as out:
            while True:
                if job.cancel_requested:
                    raise Cancelled()
                chunk = r.read(1 << 20)
                if not chunk:
                    break
                out.write(chunk)
                got += len(chunk)
                exp = job.opts.get("expected") or 0
                job.progress(99 * got / exp if exp else min(95, got / 2e7))
                rate = got / max(0.1, time.time() - t0) / 1e6
                job.note = "%.0f MB, %.0f MB/s" % (got / 1e6, rate)
        job.line("downloaded %.1f MB in %.0f s" % (got / 1e6, time.time() - t0))

        job.set_stage("unpack", 5)
        shutil.rmtree(unpack, ignore_errors=True)
        with zipfile.ZipFile(zpath) as z:
            members = z.infolist()
            total = sum(m.file_size for m in members) or 1
            done = 0
            for m in members:
                top = m.filename.split("/", 1)[0]
                if top != job.walk or ".." in m.filename.split("/"):
                    raise RuntimeError("unexpected entry in the zip: %s" % m.filename)
                z.extract(m, unpack)
                done += m.file_size
                job.progress(5 + 90 * done / total)
                if job.cancel_requested:
                    raise Cancelled()
        final = walks.folder(job.walk)
        if os.path.exists(final):
            raise RuntimeError("%s appeared on this laptop meanwhile; left the download in %s"
                               % (job.walk, unpack))
        os.replace(os.path.join(unpack, job.walk), final)
        os.remove(zpath)
        shutil.rmtree(unpack, ignore_errors=True)
        job.line("unpacked into " + final)
        try:
            os.rmdir(tmpdir)                # only if nothing else is downloading
        except OSError:
            pass
        _finish(job, "done")
        if job.opts.get("then_convert"):
            convert(job.walk, colour=job.opts.get("colour"))
    except Cancelled:
        _finish(job, "cancelled")
        for p in (zpath,):
            try:
                os.remove(p)
            except OSError:
                pass
        shutil.rmtree(unpack, ignore_errors=True)
    except Exception as e:                                  # noqa: BLE001
        _finish(job, "failed", str(e))


# ------------------------------------------------------------------ copy
def export(walk, rels, dest, as_zip=False):
    d = walks.folder(walk)
    if d is None or not os.path.isdir(d):
        return None, "no such walk"
    srcs = []
    for rel in rels:
        p = os.path.normpath(os.path.join(d, rel))
        if not p.startswith(os.path.normpath(d) + os.sep) or not os.path.exists(p):
            return None, "not in the walk: %s" % rel
        srcs.append((rel, p))
    job = _add(Job("export", walk, ["copy"], dest=dest, as_zip=as_zip))
    threading.Thread(target=_run_export, args=(job, srcs), daemon=True).start()
    return job, None


def _files(srcs):
    for rel, p in srcs:
        if os.path.isdir(p):
            for r, _, fs in os.walk(p):
                for f in fs:
                    full = os.path.join(r, f)
                    yield os.path.join(rel, os.path.relpath(full, p)), full
        else:
            yield rel, p


def _run_export(job, srcs):
    job.state, job.started, job.stage_t0 = "running", time.time(), time.time()
    dest = job.opts["dest"]
    try:
        items = list(_files(srcs))
        total = sum(os.path.getsize(p) for _, p in items) or 1
        free, _ = winos.free_bytes(dest)
        if free is not None and free < total * 1.02:
            raise RuntimeError("not enough space: %.1f GB needed, %.1f GB free"
                               % (total / 1e9, free / 1e9))
        done = 0
        if job.opts["as_zip"]:
            os.makedirs(dest, exist_ok=True)
            target = os.path.join(dest, job.walk + ".zip")
            with zipfile.ZipFile(target, "w", zipfile.ZIP_STORED, allowZip64=True) as z:
                for rel, p in items:
                    if job.cancel_requested:
                        raise Cancelled()
                    z.write(p, job.walk + "/" + rel.replace(os.sep, "/"))
                    done += os.path.getsize(p)
                    job.progress(100 * done / total)
        else:
            target = os.path.join(dest, job.walk)
            for rel, p in items:
                if job.cancel_requested:
                    raise Cancelled()
                out = os.path.join(target, rel)
                os.makedirs(os.path.dirname(out), exist_ok=True)
                shutil.copyfile(p, out)
                done += os.path.getsize(p)
                job.progress(100 * done / total)
        job.opts["result"] = target
        job.line("copied %.1f MB to %s" % (total / 1e6, target))
        _finish(job, "done")
    except Cancelled:
        _finish(job, "cancelled")
    except Exception as e:                                  # noqa: BLE001
        _finish(job, "failed", str(e))


# ------------------------------------------------------ map-making engine
# Getting the Docker image, as a job like any other: a progress bar on the
# Settings page, a Cancel button, and every line in the developer log.
#
#   pull   download the published image (about 1 GB compressed)
#   build  make it here from docker/Dockerfile, for when the download is
#          refused (a private package) or there is no internet to GitHub
PULL_LAYER = re.compile(r"^([0-9a-f]{12}): (Pulling fs layer|Waiting|Download complete|Pull complete|Already exists)")
BUILD_STEP = re.compile(r"^#\d+ \[\s*(\d+)/(\d+)\]")


def get_engine(how):
    for j in active_for("(engine)"):
        return None, "already getting the engine"
    if how not in ("pull", "build"):
        return None, "pull or build"
    job = _add(Job("engine", "(engine)", ["engine"], how=how))
    threading.Thread(target=_run_engine, args=(job,), daemon=True).start()
    return job, None


def _run_engine(job):
    job.state, job.started, job.stage_t0 = "running", time.time(), time.time()
    image = settings.get("docker_image")
    if job.opts["how"] == "pull":
        cmd = ["docker", "pull", image]
        job.note = "downloading the engine"
    else:
        cmd = ["docker", "build", "--progress", "plain", "-t", image,
               "-f", os.path.join(paths.REPO, "docker", "Dockerfile"), paths.REPO]
        job.note = "building the engine here (about 15 minutes)"
    job.cmd = " ".join(cmd)
    job.line("$ " + job.cmd)
    log("inf", "engine", job.cmd)
    layers, done = {}, 0
    try:
        job.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    stdin=subprocess.DEVNULL, creationflags=winos.NO_WINDOW)
        job.cancel_cmd = None
        for raw in iter(job.proc.stdout.readline, b""):
            text = ANSI.sub("", raw.decode("utf-8", "replace")).rstrip()
            if not text:
                continue
            job.line(text)
            m = PULL_LAYER.match(text)
            if m:
                state = m.group(2)
                layers[m.group(1)] = {"Pulling fs layer": 0, "Waiting": 0, "Download complete": .7,
                                      "Pull complete": 1, "Already exists": 1}[state]
                job.progress(100 * sum(layers.values()) / max(1, len(layers)))
            m = BUILD_STEP.match(text)
            if m:
                done = max(done, int(m.group(1)))
                job.progress(100 * (done - 1) / int(m.group(2)))
                job.note = "building: step %s of %s" % (m.group(1), m.group(2))
            if job.cancel_requested:
                job.proc.kill()
        code = job.proc.wait()
        if job.cancel_requested:
            raise Cancelled()
        if code != 0:
            last = [t for _, _, t in list(job.lines)[-8:]]
            why = " ".join(last)[-300:]
            if "denied" in why or "unauthorized" in why:
                why = ("the download was refused: the engine on GitHub is private. Build it here "
                       "instead, or make the package public on GitHub")
            raise RuntimeError(why)
        _finish(job, "done")
    except Cancelled:
        _finish(job, "cancelled")
    except FileNotFoundError:
        _finish(job, "failed", "Docker is not installed")
    except Exception as e:                                  # noqa: BLE001
        _finish(job, "failed", str(e))
