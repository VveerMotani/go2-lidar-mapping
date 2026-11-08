"""Talking to the robot's own dashboard (notes/dashboard.py on the Jetson).

Studio never talks to ROS or DDS. Everything goes through the dashboard's
small HTTP API, the same one its own page uses:

    GET  /health               is it up, its version, the stream rates
    GET  /data                 battery, motors, IMU, motion, stream report
    GET  /api/preflight        the checks before a walk
    GET  /api/recordings       walk status, last verdict, walks on the robot
    POST /api/control          walk-start, walk-stop, walk-delete, set-clock
    GET  /api/walk-download    a walk as a stored .zip

A poller reads the first four once a second in the background and keeps the
latest answer, so the page never waits on the robot. When the cable is out
(every walk, on purpose) the last answers are kept and marked stale rather
than thrown away: "last seen 37 s ago, still recording" is useful.
"""
import json
import threading
import time
import urllib.error
import urllib.parse
import urllib.request

from . import settings
from .applog import log, request

POLL = [("health", "/health", 2.0), ("data", "/data", 2.0),
        ("preflight", "/api/preflight", 3.0), ("recordings", "/api/recordings", 3.0),
        ("sensors", "/api/sensors", 3.0)]


def base():
    return "http://%s:%d" % (settings.get("robot_host"), settings.get("robot_port"))


def call(method, path, params=None, timeout=5.0, quiet=False):
    """One request. Returns (status, json-or-None, error-or-None)."""
    url = base() + path
    data = None
    headers = {}
    if method == "POST":
        data = json.dumps(params or {}).encode("utf-8")
        headers["Content-Type"] = "application/json"
    t0 = time.time()
    status, body, err = None, b"", None
    try:
        req = urllib.request.Request(url, data=data, method=method, headers=headers)
        with urllib.request.urlopen(req, timeout=timeout) as r:
            status, body = r.status, r.read()
    except urllib.error.HTTPError as e:
        status, body = e.code, e.read()
    except (urllib.error.URLError, OSError) as e:
        err = getattr(e, "reason", None) or e
        err = "timeout" if "timed out" in str(err) else str(err)
    ms = (time.time() - t0) * 1000
    request(method, path, status, ms, len(body), err)
    if not quiet and method == "POST":
        log("ok" if status == 200 else "wrn", "robot",
            "%s %s %s -> %s (%d ms)" % (method, path, json.dumps(params or {})[:120],
                                        status or err, ms))
    try:
        return status, (json.loads(body) if body else None), err
    except ValueError:
        return status, None, err or "not JSON"


class Poller(threading.Thread):
    """Keeps the latest answer for each POLL endpoint."""

    def __init__(self):
        super().__init__(daemon=True, name="robot-poller")
        self.lock = threading.Lock()
        self.latest = {}                  # key -> (time, json)
        self.last_ok = None               # time of the last answer at all
        self.reachable = False
        self._was = None

    def run(self):
        while True:
            t0 = time.time()
            for key, path, timeout in POLL:
                # Once the robot stops answering, stop asking for everything:
                # four timeouts a second would make the whole app sluggish.
                if not self.reachable and key != "health":
                    continue
                status, body, err = call("GET", path, timeout=timeout if self.reachable else 1.0,
                                         quiet=True)
                if status == 200 and body is not None:
                    with self.lock:
                        self.latest[key] = (time.time(), body)
                        self.last_ok = time.time()
                if key == "health":
                    self._set_reachable(status == 200, err)
            time.sleep(max(0.2, 1.0 - (time.time() - t0)))

    def _set_reachable(self, ok, err):
        self.reachable = ok
        if ok != self._was:
            if ok:
                log("ok", "robot", "robot dashboard answering at %s" % base())
            elif self._was is not None:
                log("wrn", "robot", "robot out of reach (%s)" % (err or "no answer"))
            self._was = ok

    def snapshot(self):
        now = time.time()
        with self.lock:
            out = {k: {"age": round(now - t, 1), "body": b} for k, (t, b) in self.latest.items()}
            last = self.last_ok
        return {"reachable": self.reachable, "base": base(),
                "last_seen_age": round(now - last, 1) if last else None,
                "data": out}


POLLER = Poller()


def control(action, **params):
    params["action"] = action
    timeout = 20.0 if action in ("walk-start", "walk-stop") else 8.0
    return call("POST", "/api/control", params, timeout=timeout)


def download_url(name):
    return base() + "/api/walk-download?name=" + urllib.parse.quote(name)
