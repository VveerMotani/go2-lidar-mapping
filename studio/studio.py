"""Go2 Studio: record walks with the Go2, turn them into maps, keep them tidy.

    python studio.py            start, and open the Studio window
    python studio.py --no-window   start the server only (for development)

The window is Microsoft Edge in app mode, pointed at a small server that runs
here, on this laptop only (127.0.0.1:8421). Starting Studio again while it is
running just opens another window onto the same server.

It quits by itself a minute after the last window closes, unless a download
or conversion is still running; then it finishes that first.
"""
import argparse
import os
import subprocess
import sys
import threading
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core import jobs, paths, server, winos   # noqa: E402
from core.applog import log                   # noqa: E402

PORT = 8421


def running_already(port):
    try:
        with urllib.request.urlopen("http://127.0.0.1:%d/api/ping" % port, timeout=1) as r:
            return b"go2-studio" in r.read()
    except OSError:
        return False


def open_window(port):
    url = "http://127.0.0.1:%d/" % port
    edge = winos.edge_path()
    if edge:
        profile = os.path.join(paths.DATA, "edge")
        subprocess.Popen([edge, "--app=" + url, "--user-data-dir=" + profile,
                          "--window-size=1440,920", "--no-first-run",
                          "--disable-features=Translate"], creationflags=winos.NO_WINDOW)
    else:
        winos.open_url(url)


def idle_watch():
    """Quit when no window has polled for a minute and nothing is running."""
    while True:
        time.sleep(15)
        busy = any(j["state"] in ("queued", "running") for j in jobs.listing())
        if not busy and time.time() - server.last_page_poll > 60:
            log("inf", "app", "no window open and nothing running: quitting")
            os._exit(0)


def main():
    ap = argparse.ArgumentParser(description="Go2 Studio")
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--no-window", action="store_true", help="do not open the window")
    ap.add_argument("--stay", action="store_true", help="keep running with no window open")
    args = ap.parse_args()

    if running_already(args.port):
        if not args.no_window:
            open_window(args.port)
        return

    httpd = server.serve(args.port)
    server.start_background()
    threading.Thread(target=_notes_loop, daemon=True).start()
    if not args.stay:
        threading.Thread(target=idle_watch, daemon=True).start()
    if not args.no_window:
        open_window(args.port)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass


def _notes_loop():
    while True:
        try:
            server.downloaded_notes_hook()
        except Exception:                                  # noqa: BLE001
            pass
        time.sleep(10)


if __name__ == "__main__":
    main()
