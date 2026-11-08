"""
Forward the Hesai XT16 stream from the robot to the laptop.

Runs on the Jetson. The LiDAR is configured to send its point data to the
Jetson (192.168.123.18:2368); nothing on the robot consumes it, so we take the
packets and repeat them to the laptop unchanged.

Doing it this way means the LiDAR's own configuration is never touched -- the
one time we repointed it, it stopped transmitting until a power cycle.

    python3 lidar_relay.py                       # -> 192.168.123.99:2368
    python3 lidar_relay.py --dest 192.168.1.5 --dest-port 2368
"""
import argparse
import json
import os
import socket
import sys
import time

DEFAULT_DEST = "192.168.123.99"


STATUS_FILE = "/tmp/lidar_relay_status.json"


def write_status(counts, elapsed, dropped):
    """Packets a second per port, once a second, for the dashboard's Sensors panel."""
    status = {"time": time.time(),
              "pps": {str(p): round(n / max(elapsed, 1e-3), 1) for p, n in counts.items()},
              "dropped": dropped}
    try:
        tmp = STATUS_FILE + ".tmp"
        with open(tmp, "w") as fh:
            json.dump(status, fh)
        os.replace(tmp, STATUS_FILE)
    except OSError:
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--listen-port", type=int, default=2368,
                    help="port the LiDAR sends point data to")
    ap.add_argument("--gps-port", type=int, default=10110,
                    help="port the LiDAR sends GPS/position packets to (0 disables)")
    ap.add_argument("--dest", default=DEFAULT_DEST)
    ap.add_argument("--dest-port", type=int, default=None,
                    help="defaults to the same port it was received on")
    ap.add_argument("--report-every", type=float, default=30.0)
    args = ap.parse_args()

    ports = [args.listen_port]
    if args.gps_port:
        ports.append(args.gps_port)

    socks = {}
    for port in ports:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # a big receive buffer: the XT16 pushes ~5000 packets/second
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
        try:
            s.bind(("", port))
        except OSError as exc:
            print("cannot bind port %d: %s" % (port, exc), file=sys.stderr)
            raise SystemExit(1)
        s.setblocking(False)
        socks[s] = port

    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    out.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8 * 1024 * 1024)

    print("relaying %s -> %s (%d bytes/pkt typical)"
          % (", ".join(str(p) for p in ports), args.dest, 1080))
    sys.stdout.flush()

    import select
    counts = {p: 0 for p in ports}
    dropped = 0
    last_report = time.time()
    status_counts = {p: 0 for p in ports}
    last_status = time.time()

    while True:
        ready, _, _ = select.select(list(socks), [], [], 1.0)
        for s in ready:
            port = socks[s]
            dest_port = args.dest_port or port
            # drain everything queued before going back to select
            while True:
                try:
                    data = s.recv(65535)
                except BlockingIOError:
                    break
                except OSError:
                    break
                counts[port] += 1
                status_counts[port] += 1
                try:
                    out.sendto(data, (args.dest, dest_port))
                except OSError:
                    dropped += 1

        now = time.time()
        if now - last_status >= 1.0:
            write_status(status_counts, now - last_status, dropped)
            status_counts = {p: 0 for p in ports}
            last_status = now
        if now - last_report >= args.report_every:
            elapsed = now - last_report
            rates = "  ".join("%d: %.0f pkt/s" % (p, counts[p] / elapsed)
                              for p in ports)
            print("%s%s" % (rates, "  dropped %d" % dropped if dropped else ""))
            sys.stdout.flush()
            counts = {p: 0 for p in ports}
            dropped = 0
            last_report = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nstopped")
