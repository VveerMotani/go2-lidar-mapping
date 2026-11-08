"""Where Studio keeps things, and where the repo's own scripts are.

Studio lives inside the repo (studio/), next to notes/ and launchpad/, and
uses their code rather than copying it: notes/walk_library.py judges a walk,
launchpad/checks.py looks at the network, notes/process_walk.sh converts.
"""
import os
import sys

STUDIO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(STUDIO)
NOTES = os.path.join(REPO, "notes")
LAUNCHPAD = os.path.join(REPO, "launchpad")
STATIC = os.path.join(STUDIO, "static")

_local = os.environ.get("LOCALAPPDATA") or os.path.expanduser("~")
_roaming = os.environ.get("APPDATA") or os.path.expanduser("~")
DATA = os.path.join(_local, "Go2Studio")          # logs, job history, Edge profile
LOGS = os.path.join(DATA, "logs")
SETTINGS = os.path.join(_roaming, "Go2Studio", "settings.json")

for p in (NOTES, LAUNCHPAD):
    if p not in sys.path:
        sys.path.insert(0, p)


def wsl_path(win_path):
    """C:\\a\\b -> /mnt/c/a/b, the one translation that is exact.

    A path already inside WSL (\\\\wsl.localhost\\<distro>\\home\\...) maps to
    /home/...; anything else is returned unchanged and will fail loudly.
    """
    p = os.path.abspath(win_path)
    if len(p) > 1 and p[1] == ":":
        return "/mnt/" + p[0].lower() + p[2:].replace("\\", "/")
    for prefix in ("\\\\wsl.localhost\\", "\\\\wsl$\\"):
        if p.lower().startswith(prefix):
            rest = p[len(prefix):].split("\\", 1)
            return "/" + (rest[1].replace("\\", "/") if len(rest) > 1 else "")
    return p.replace("\\", "/")
