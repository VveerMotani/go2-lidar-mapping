"""
Fetch the XT16's per-laser angle calibration and save it, so recordings can be
replayed offline.

    python3 get_lidar_calibration.py --out ~/go2-lidar-mapping/walks/xt16_correction.csv

WHY THIS IS NEEDED
------------------
A spinning LiDAR's sixteen lasers each sit at a different vertical angle, and
those angles are measured per unit at the factory. The driver normally asks the
sensor for them over TCP at startup -- its log says "Read correction file from
lidar success" -- and without them every laser is treated as horizontal.

Replaying a recorded pcap has no sensor to ask. The result is a point cloud that
looks plausible from above and is completely flat from the side: one layer, no
height. Observed exactly that on the first recorded walk.

So: fetch the calibration once while the sensor is connected, keep the file, and
point replay configs at it via pcap_type.correction_file_path.

WHAT IT SENDS
-------------
One command, 0x05, "get lidar calibration". It reads and writes nothing else.

That restraint is deliberate. The same protocol carries command 0x20, which sets
the sensor's destination IP -- and changing that once already stopped this XT16
transmitting until it was power-cycled (docs/hard-won-facts.md). This program
will not send anything except the read commands in SAFE_COMMANDS below.
"""
import argparse
import os
import socket
import struct
import sys

# PTC 1.0 framing, from the SDK's libhesai/PtcParser/include/ptc_1_0_parser.h:
#   identifier0 (0x47), identifier1 (0x74), cmd, return_code, payload_len
HEADER = struct.Struct("!BBBBI")
IDENT0, IDENT1 = 0x47, 0x74

# Read-only commands, from libhesai/PtcClient/include/ptc_client.h. Anything
# that writes to the sensor is deliberately absent and must stay absent.
SAFE_COMMANDS = {
    "calibration": 0x05,     # kPTCGetLidarCalibration
    "inventory": 0x07,       # kPTCGetInventoryInfo
    "status": 0x09,          # kPTCGetLidarStatus
}


def query(host, port, cmd, timeout=10.0):
    """Send one PTC command with an empty payload and return the response body."""
    if cmd not in SAFE_COMMANDS.values():
        raise SystemExit("refusing to send command 0x%02x -- not a read command" % cmd)

    sock = socket.create_connection((host, port), timeout=timeout)
    try:
        sock.sendall(HEADER.pack(IDENT0, IDENT1, cmd, 0, 0))

        head = b""
        while len(head) < HEADER.size:
            chunk = sock.recv(HEADER.size - len(head))
            if not chunk:
                raise SystemExit("sensor closed the connection while sending its header")
            head += chunk

        i0, i1, rcmd, rcode, length = HEADER.unpack(head)
        if (i0, i1) != (IDENT0, IDENT1):
            raise SystemExit("not a PTC response: %02x %02x" % (i0, i1))
        if rcode != 0:
            raise SystemExit("sensor refused the request, return code %d" % rcode)
        if length > 10 * 1024 * 1024:
            raise SystemExit("implausible payload length %d -- wrong byte order?" % length)

        body = b""
        while len(body) < length:
            chunk = sock.recv(min(65536, length - len(body)))
            if not chunk:
                break
            body += chunk
        if len(body) < length:
            raise SystemExit("short read: %d of %d bytes" % (len(body), length))
        return body
    finally:
        sock.close()


def main():
    ap = argparse.ArgumentParser(
        description="save the XT16's angle calibration for offline replay")
    ap.add_argument("--host", default="192.168.123.20")
    ap.add_argument("--port", type=int, default=9347, help="PTC port")
    # The repo's walks/ folder, beside notes/, wherever the repo lives.
    walks = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "walks")
    ap.add_argument("--out", default=os.path.join(walks, "xt16_correction.csv"))
    args = ap.parse_args()

    print("asking %s:%d for its angle calibration" % (args.host, args.port))
    data = query(args.host, args.port, SAFE_COMMANDS["calibration"])
    print("  received %d bytes" % len(data))

    text = data.decode("utf-8", "replace")
    lines = [ln for ln in text.splitlines() if ln.strip()]
    print("  %d lines, first two:" % len(lines))
    for ln in lines[:2]:
        print("    %s" % ln)

    # An XT16 file has a header plus one row per laser. Anything wildly
    # different means we have not got what we think we have.
    if len(lines) < 10:
        print("  WARNING: expected about 17 lines for a 16-laser sensor")

    out = os.path.abspath(args.out)
    d = os.path.dirname(out)
    if d and not os.path.isdir(d):
        os.makedirs(d)
    with open(out, "wb") as fh:
        fh.write(data)
    print("\nsaved %s" % out)
    print("\nUse it when replaying a recording, under pcap_type:")
    print("  correction_file_path: \"%s\"" % out)


if __name__ == "__main__":
    main()
