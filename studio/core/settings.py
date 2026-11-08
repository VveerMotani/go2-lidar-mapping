"""Settings, saved per user in %APPDATA%\\Go2Studio\\settings.json.

Only keys listed in DEFAULTS are ever stored, so the page cannot write
arbitrary values into the file, and a newer Studio reading an older file
simply gets the defaults for anything new.
"""
import json
import os
import threading

from . import paths

DEFAULTS = {
    "walks_dir": "C:\\go2-walks",
    "robot_host": "192.168.123.18",
    "robot_port": 8090,
    "engine": "auto",                  # auto | docker | wsl
    "wsl_distro": "unitree",
    "docker_image": "ghcr.io/vveermotani/go2-lidar-mapping/go2-walks:latest",
    "keep": "all",                     # all | maps (drop the full bag)
    "foxglove": "",                    # blank: find it
    "cloudcompare": "",
    "notify": True,
    "auto_convert": False,             # download and make the map of new walks by itself
    "colour": False,                   # camera colour when making maps (an experiment, 5-10 min more)
    "guide": "all",                    # the Next step bar: all | quiet (only when it needs you) | off
    "dev": False,
}

_lock = threading.Lock()
_cache = None


def load():
    global _cache
    with _lock:
        if _cache is None:
            data = {}
            try:
                with open(paths.SETTINGS, encoding="utf-8") as fh:
                    data = json.load(fh)
            except (OSError, ValueError):
                pass
            _cache = dict(DEFAULTS)
            _cache.update({k: v for k, v in data.items() if k in DEFAULTS})
        return dict(_cache)


def get(key):
    return load()[key]


def save(changes):
    """Merge `changes` (known keys only, type-checked against the default)."""
    global _cache
    current = load()
    for k, v in (changes or {}).items():
        if k not in DEFAULTS:
            continue
        want = type(DEFAULTS[k])
        if want is int:
            v = int(v)
        elif want is bool:
            v = bool(v)
        elif not isinstance(v, str):
            continue
        current[k] = v
    os.makedirs(os.path.dirname(paths.SETTINGS), exist_ok=True)
    tmp = paths.SETTINGS + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(current, fh, indent=2)
    os.replace(tmp, paths.SETTINGS)
    with _lock:
        _cache = current
    return dict(current)
