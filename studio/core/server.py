"""Studio's HTTP API, for its own page and nothing else.

Two locks on the door, because this server can start programs:
  * it listens on 127.0.0.1 and refuses any Host header that is not local,
    so nothing on the network can reach it;
  * every POST must carry the X-Go2-Studio header. A page on some other web
    site can make the browser send a plain POST to localhost, but not one with
    a custom header without a CORS preflight, which this server never grants.
"""
import json
import mimetypes
import os
import platform
import shutil
import subprocess
import sys
import threading
import time
import zipfile
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from . import applog, jobs, paths, robot, settings, walks, winos
from .applog import log

VERSION = "0.1.0"
LOCAL_HOSTS = ("127.0.0.1", "localhost")
last_page_poll = time.time()
PENDING_NOTES = os.path.join(paths.DATA, "pending-notes.json")

try:
    import checks as lp_checks              # launchpad/checks.py
except ImportError:                          # pragma: no cover
    lp_checks = None


def git_commit():
    code, out = winos.run(["git", "-C", paths.REPO, "rev-parse", "--short", "HEAD"], timeout=5)
    return out if code == 0 else "unknown"


COMMIT = git_commit()


# ------------------------------------------------------- background checks
class Every(threading.Thread):
    """Run fn every `period` seconds and keep the latest result."""

    def __init__(self, name, fn, period):
        super().__init__(daemon=True, name=name)
        self.fn, self.period, self.value, self.t = fn, period, None, 0

    def run(self):
        while True:
            try:
                self.value, self.t = self.fn(), time.time()
            except Exception as e:                          # noqa: BLE001
                log("err", "app", "%s check failed: %s" % (self.name, e))
            time.sleep(self.period)


CONNECT = Every("connect", lambda: lp_checks.run_all() if lp_checks else None, 4.0)
ENGINE = Every("engine", jobs.engine_status, 10.0)


def _programs():
    s = settings.load()
    return {"foxglove": winos.find_program("foxglove", s["foxglove"]),
            "cloudcompare": winos.find_program("cloudcompare", s["cloudcompare"])}


PROGRAMS = Every("programs", _programs, 60.0)


def start_background():
    robot.POLLER.start()
    for t in (CONNECT, ENGINE, PROGRAMS):
        t.start()


# ------------------------------------------------------------------ views
def state():
    root = walks.root()
    free, total = winos.free_bytes(root)
    return {
        "time": time.time(),
        "app": {"version": VERSION, "commit": COMMIT},
        "home": os.path.expanduser("~"),
        "robot": robot.POLLER.snapshot(),
        "connect": CONNECT.value,
        "engine": ENGINE.value,
        "programs": PROGRAMS.value,
        "jobs": jobs.listing()[:20],
        "settings": settings.load(),
        "walks_dir": {"path": root, "exists": os.path.isdir(root), "free": free,
                      "total": total, "onedrive": winos.in_onedrive(root)},
    }


def walk_list():
    snap = robot.POLLER.snapshot()
    rec = (snap["data"].get("recordings") or {}).get("body") or {}
    robot_walks = (rec.get("walk") or {}).get("walks") or []
    return {"walks": walks.merged(robot_walks, snap["reachable"]),
            "robot_reachable": snap["reachable"]}


def system():
    s = settings.load()
    return {
        "studio": "%s (%s)" % (VERSION, COMMIT),
        "python": "%s %s" % (platform.python_implementation(), sys.version.split()[0]),
        "windows": platform.platform(),
        "pid": os.getpid(),
        "repo": paths.REPO,
        "walks": s["walks_dir"],
        "logs": paths.LOGS,
        "settings": paths.SETTINGS,
        "engine": ENGINE.value,
        "programs": PROGRAMS.value,
        "robot": robot.base(),
        "robot_health": ((robot.POLLER.snapshot()["data"].get("health") or {}).get("body")),
    }


def debug_bundle():
    """Logs, settings, job history and system info in one zip in Downloads."""
    out_dir = os.path.join(os.path.expanduser("~"), "Downloads")
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, time.strftime("go2studio-debug-%Y%m%d-%H%M%S.zip"))
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        for f in os.listdir(paths.LOGS):
            z.write(os.path.join(paths.LOGS, f), "logs/" + f)
        for p in (paths.SETTINGS, jobs.HISTORY):
            if os.path.isfile(p):
                z.write(p, os.path.basename(p))
        z.writestr("system.json", json.dumps(system(), indent=1, default=str))
        z.writestr("requests.json", json.dumps(list(applog.REQUESTS), indent=1))
        z.writestr("robot.json", json.dumps(robot.POLLER.snapshot(), indent=1, default=str))
        for j in list(jobs.JOBS.values())[-10:]:
            z.writestr("jobs/%s-%s-%s.log" % (j.walk, j.kind, j.id),
                       "\n".join(t for _, _, t in j.lines))
    log("ok", "app", "debug bundle written: %s" % path)
    return path


def _pending(name=None, notes=None):
    try:
        with open(PENDING_NOTES, encoding="utf-8") as fh:
            data = json.load(fh)
    except (OSError, ValueError):
        data = {}
    if name is not None and notes is not None:
        data[name] = notes
        os.makedirs(paths.DATA, exist_ok=True)
        with open(PENDING_NOTES, "w", encoding="utf-8") as fh:
            json.dump(data, fh, indent=1)
    return data


# ---------------------------------------------------------------- actions
def act_robot(p):
    a = p.get("action")
    if a == "set-clock":
        return robot.control("set-clock", epoch_ms=int(time.time() * 1000))
    if a == "walk-start":
        name = p.get("name") or ""
        if p.get("notes"):
            _pending(name, p["notes"])
        return robot.control("walk-start", name=name, force=bool(p.get("force")))
    if a == "walk-stop":
        return robot.control("walk-stop")
    if a == "walk-delete":
        return robot.control("walk-delete", name=p.get("name"))
    if a == "sensor":
        return robot.call("POST", "/api/control", {"action": "sensor", **p.get("params", {})})
    return 400, {"error": "unknown robot action"}, None


def act_walks(p):
    a, name = p.get("action"), p.get("name")
    if a == "download":
        job, err = jobs.download(name, int(p.get("bytes") or 0), bool(p.get("convert")), p.get("colour"))
        if job and name in _pending():
            job.opts["notes"] = _pending()[name]
        return (200, job.public()) if job else (409, {"error": err})
    if a in ("convert", "map"):
        job, err = jobs.convert(name, map_only=(a == "map"), colour=p.get("colour"))
        return (200, job.public()) if job else (409, {"error": err})
    if a == "cancel":
        return (200, {"ok": True}) if jobs.cancel(p.get("job")) else (404, {"error": "no such job"})
    if a == "export":
        job, err = jobs.export(name, p.get("files") or [], p.get("dest") or "", bool(p.get("zip")))
        return (200, job.public()) if job else (400, {"error": err})
    if a == "detail":
        w = walks.describe(name)
        return (200, w) if w else (404, {"error": "not on this laptop"})
    if a == "notes":
        return (200, {"ok": walks.save_notes(name, p.get("notes"))})
    if a == "notes-pending":
        # Notes and markers from the Record page, for a walk still on the robot:
        # written into the walk's folder once it has been downloaded.
        if walks.folder(name) is None:
            return 400, {"error": "bad name"}
        _pending(name, p.get("notes") or {})
        return 200, {"ok": True}
    if a in ("show", "open"):
        d = walks.folder(name)
        if d is None:
            return 400, {"error": "bad name"}
        target = os.path.normpath(os.path.join(d, p.get("file") or ""))
        if target != os.path.normpath(d) and not target.startswith(os.path.normpath(d) + os.sep):
            return 400, {"error": "outside the walk"}
        if a == "show":
            ok, err = winos.show_in_folder(target)
        else:
            prog = None
            if p.get("with") in ("foxglove", "cloudcompare"):
                prog = (PROGRAMS.value or {}).get(p["with"])
            ok, err = winos.open_file(target, prog)
        return (200, {"ok": True}) if ok else (404, {"error": err})
    if a == "delete-local":
        d = walks.folder(name)
        if not d or not os.path.isdir(d):
            return 404, {"error": "no such walk"}
        if jobs.active_for(name):
            return 409, {"error": "a job is running on this walk"}
        shutil.rmtree(d)
        log("wrn", "disk", "deleted walk %s from this laptop" % d)
        return 200, {"ok": True}
    return 400, {"error": "unknown walks action"}


def act_os(p):
    a = p.get("action")
    if a == "pick-folder":
        return 200, {"path": winos.pick_folder(p.get("initial") or "")}
    if a == "robot-dashboard":
        winos.open_url(robot.base())
        return 200, {"ok": True}
    if a == "open-url" and str(p.get("url", "")).startswith(("https://", "http://192.168.123.")):
        winos.open_url(p["url"])
        return 200, {"ok": True}
    if a == "open-logs":
        winos.show_in_folder(paths.LOGS)
        return 200, {"ok": True}
    if a == "show-walks":
        os.makedirs(walks.root(), exist_ok=True)
        winos.show_in_folder(walks.root())
        return 200, {"ok": True}
    if a == "debug-bundle":
        path = debug_bundle()
        winos.show_in_folder(path)
        return 200, {"path": path}
    if a == "engine-get":
        job, err = jobs.get_engine(p.get("how", "pull"))
        return (200, job.public()) if job else (409, {"error": err})
    if a == "engine-start-docker":
        return (200, {"ok": True}) if winos.start_docker() else (404, {"error": "Docker Desktop not found"})
    if a == "engine-refresh":
        ENGINE.value = jobs.engine_status()
        return 200, ENGINE.value
    if a == "set-address":
        alias = ((CONNECT.value or {}).get("link") or {}).get("adapter") or "Ethernet"
        ok, cmd = winos.set_robot_address(bool(p.get("on")), alias)
        return 200, {"started": ok, "command": cmd}
    return 400, {"error": "unknown os action"}


# ------------------------------------------------------------------- HTTP
class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *_):
        pass

    def _local(self):
        host = (self.headers.get("Host") or "").rsplit(":", 1)[0].strip("[]")
        return host in LOCAL_HOSTS

    def _send(self, code, body, ctype="application/json", cache="no-store"):
        if not isinstance(body, (bytes, bytearray)):
            body = json.dumps(body, default=str).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", cache)
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        global last_page_poll
        if not self._local():
            return self._send(403, {"error": "local only"})
        u = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(u.query).items()}
        path = u.path
        try:
            if path == "/":
                return self._static("index.html")
            if path.startswith("/static/"):
                return self._static(path[len("/static/"):])
            if path == "/api/ping":
                return self._send(200, {"app": "go2-studio", "version": VERSION})
            if path == "/api/state":
                last_page_poll = time.time()
                return self._send(200, state())
            if path == "/api/walks":
                return self._send(200, walk_list())
            if path == "/api/system":
                return self._send(200, system())
            if path == "/api/logs":
                since = int(q.get("since") or 0)
                kind = q.get("kind", "log")
                if kind == "req":
                    return self._send(200, applog.since(applog.REQUESTS, since))
                if kind == "job":
                    j = jobs.get(q.get("job", ""))
                    if not j:
                        return self._send(404, {"error": "no such job"})
                    return self._send(200, {"job": j.public(),
                                            "lines": [l for l in j.lines if l[0] > since]})
                return self._send(200, applog.since(applog.LOG, since))
            if path == "/api/walk-image":
                # A walk's floor plan, for the page to show. Only .png files
                # inside the walk's own map3d folder.
                d = walks.folder(q.get("name", ""))
                rel = q.get("file", "")
                if d is None or not rel.endswith(".png"):
                    return self._send(400, {"error": "bad request"})
                base = os.path.normpath(os.path.join(d, "map3d"))
                full = os.path.normpath(os.path.join(d, rel))
                if not full.startswith(base + os.sep) or not os.path.isfile(full):
                    return self._send(404, {"error": "not found"})
                with open(full, "rb") as fh:
                    return self._send(200, fh.read(), "image/png", cache="max-age=60")
            if path == "/api/robot-logs":
                name = q.get("name", "dashboard")
                status, body, err = robot.call("GET", "/api/logs?name=%s" % name, timeout=4)
                return self._send(200, {"status": status, "body": body, "error": err})
            return self._send(404, {"error": "not found"})
        except Exception as e:                              # noqa: BLE001
            log("err", "app", "GET %s failed: %r" % (path, e))
            return self._send(500, {"error": str(e)})

    def do_POST(self):
        if not self._local() or self.headers.get("X-Go2-Studio") != "1":
            return self._send(403, {"error": "local page only"})
        length = int(self.headers.get("Content-Length") or 0)
        try:
            p = json.loads(self.rfile.read(length) or b"{}")
        except ValueError:
            return self._send(400, {"error": "bad JSON"})
        path = urlparse(self.path).path
        try:
            if path == "/api/robot":
                status, body, err = act_robot(p)
                if status is None:
                    return self._send(502, {"error": "robot did not answer: %s" % err})
                return self._send(status, body or {"error": err})
            if path == "/api/walks":
                return self._send(*act_walks(p))
            if path == "/api/os":
                return self._send(*act_os(p))
            if path == "/api/settings":
                before = settings.load()
                after = settings.save(p.get("changes") or {})
                changed = {k: after[k] for k in after if after[k] != before.get(k)}
                if changed:
                    log("inf", "app", "settings changed: %s" % json.dumps(changed))
                return self._send(200, after)
            if path == "/api/log":                 # the page's own errors
                log(p.get("level", "err"), "ui", str(p.get("msg"))[:1000])
                return self._send(200, {"ok": True})
            return self._send(404, {"error": "not found"})
        except Exception as e:                              # noqa: BLE001
            log("err", "app", "POST %s %s failed: %r" % (path, p.get("action"), e))
            return self._send(500, {"error": str(e)})

    def _static(self, rel):
        full = os.path.normpath(os.path.join(paths.STATIC, rel))
        if not full.startswith(paths.STATIC) or not os.path.isfile(full):
            return self._send(404, {"error": "not found"})
        with open(full, "rb") as fh:
            body = fh.read()
        ctype = mimetypes.guess_type(full)[0] or "application/octet-stream"
        if full.endswith(".js"):
            ctype = "text/javascript"
        return self._send(200, body, ctype + ("; charset=utf-8" if ctype.startswith("text") else ""))


def serve(port):
    httpd = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    httpd.daemon_threads = True
    log("inf", "app", "Go2 Studio %s (%s) serving http://127.0.0.1:%d" % (VERSION, COMMIT, port))
    return httpd


def downloaded_notes_hook():
    """Write notes typed at Start walk into the walk once it is downloaded."""
    pend = _pending()
    for name, notes in list(pend.items()):
        if walks.folder(name) and os.path.isdir(walks.folder(name)) \
                and not os.path.isfile(os.path.join(walks.folder(name), "notes.json")):
            walks.save_notes(name, notes)
