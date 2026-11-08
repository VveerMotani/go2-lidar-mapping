"""The app's own log: what the Developer logs console shows.

Two things are kept in memory, each as a numbered ring so the page can ask
"everything after line N" instead of re-reading the lot every second:

  * LOG      every event: app, robot, job, net, engine, os, disk
  * REQUESTS every HTTP call to the robot, with its status and timing

The main log is also written to %LOCALAPPDATA%\\Go2Studio\\logs\\studio.log, so a
problem that happened while nobody was looking can still be read afterwards,
and the debug bundle has something to include.
"""
import collections
import logging
import logging.handlers
import os
import threading
import time

from . import paths

LEVELS = ("inf", "ok", "wrn", "err")

_lock = threading.Lock()
_seq = 0
LOG = collections.deque(maxlen=5000)
REQUESTS = collections.deque(maxlen=1000)

_file = logging.getLogger("go2studio")
_file.setLevel(logging.INFO)
_file.propagate = False


def _open_file():
    os.makedirs(paths.LOGS, exist_ok=True)
    h = logging.handlers.RotatingFileHandler(
        os.path.join(paths.LOGS, "studio.log"), maxBytes=5_000_000,
        backupCount=3, encoding="utf-8")
    h.setFormatter(logging.Formatter("%(message)s"))
    _file.addHandler(h)


_open_file()


def _next():
    global _seq
    _seq += 1
    return _seq


def log(level, source, message):
    """Record one event. level is one of LEVELS."""
    if level not in LEVELS:
        level = "inf"
    t = time.time()
    with _lock:
        LOG.append({"seq": _next(), "t": t, "level": level, "src": source,
                    "msg": message})
    stamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t))
    _file.info("%s.%03d %-3s %-7s %s", stamp, int(t * 1000) % 1000,
               level.upper(), source, message)


def request(method, url, status, ms, nbytes, error=None):
    """Record one call to the robot."""
    with _lock:
        REQUESTS.append({"seq": _next(), "t": time.time(), "method": method,
                         "url": url, "status": status, "ms": round(ms),
                         "bytes": nbytes, "error": error})


def since(ring, seq, limit=2000):
    """Entries after `seq`, oldest first."""
    with _lock:
        out = [e for e in ring if e["seq"] > seq]
    return out[-limit:]
