"""
Repair a bag whose mcap was cut off mid-write.

    python3 recover_bag.py ~/go2-lidar-mapping/walks/First-Walk-CB-1s-Floor/bag

Writes a repaired copy next to the original as <bagdir>-recovered/ and, if the
ROS tooling is available, regenerates its metadata.yaml so `ros2 bag play` and
`ros2 bag info` work again. **The original is never modified.**

READ THIS BEFORE REACHING FOR IT
--------------------------------
If the recording that produced the bag is still on disk -- lidar.pcap, imu.csv
and the rest -- **re-running walk_to_bag.sh is better than recovering**, and it
is not close:

  * You get the WHOLE walk. Recovery gets back what was written before the cut,
    which for a conversion killed a third of the way through is a third of a
    walk.
  * You get a properly INDEXED file (see below).
  * You get whatever the pipeline has learned since, which for anything
    recorded before 2026-09-05 includes the moving transform.

Recovery is for when the source files are gone, or when you want to see what a
damaged bag contains before deciding.

THE OUTPUT IS NOT INDEXED, AND FOXGLOVE CARES
---------------------------------------------
A finished mcap ends with a summary section: an index of every chunk and the
time range it covers, so a reader can seek. That summary is written at
shutdown, along with everything else this script is replacing -- so a truncated
file never had one, and nothing can reconstruct it from outside the chunks.

What we write instead is a valid mcap with an EMPTY footer, meaning "no
summary". Readers are required to cope by scanning the file linearly, and
rosbag2 does exactly that ("no message indices found, falling back to reading
in file order").

**Foxglove refuses to scan a file over 600 MB** and reports:

    File has exceeded the maximum unindexed read size of 600.0 MB bytes.

So for a recovered file under 600 MB, Foxglove opens it. Over that, it will
not, and your options are to re-run walk_to_bag.sh (best), or to index the file
with Foxglove's own `mcap` CLI, which is the tool their error message is
pointing at:

    mcap recover <file>.mcap -o <fixed>.mcap

That is a single Go binary from github.com/foxglove/mcap releases. It is not
installed here, deliberately -- this script has no dependencies so that it
works when nothing else does.

WHAT GOES WRONG, AND WHY IT IS INVISIBLE
----------------------------------------
rosbag2 writes an mcap as it goes and finishes the job at shutdown: it appends
a DataEnd record, a Footer, and the file's closing magic, and only then writes
metadata.yaml beside it. Kill the recorder outright -- SIGKILL, a crash, two
conversions racing each other, a machine going down -- and you get a large file
full of perfectly good data with none of that ending.

Nothing about it looks wrong. It has the right name, a plausible size, and its
first bytes are a valid mcap header. It simply plays back as nothing.

Observed 2026-09-04 on First-Walk-CB-1s-Floor: 625 MB, no closing magic, no
metadata.yaml, 137 seconds of walking unreadable. walk_to_bag.sh now shuts the
recorder down with SIGTERM and waits, and checks the ending afterwards, so this
should not recur -- but the file already exists and is worth an hour.

HOW THE REPAIR WORKS
--------------------
An mcap is a very simple envelope:

    <8-byte magic> <record> <record> ... <DataEnd> <Footer> <8-byte magic>

and every record is one opcode byte, an 8-byte little-endian length, and that
many bytes of content. The summary section that normally sits before the Footer
is OPTIONAL -- a reader with no summary falls back to scanning the file, which
is exactly what we want here.

So the repair is: walk the records from the start, stop at the first one that
runs off the end of the file, and write everything before it followed by a
DataEnd, an empty Footer and the closing magic. Whatever was written before the
cut is kept; only the partial record at the very end is lost, which is at most
one chunk -- a fraction of a second of walking.

Deliberately no dependencies. This has to work when the ROS tooling cannot open
the file, which is the entire situation it exists for.
"""
import argparse
import os
import struct
import subprocess
import sys

MAGIC = b"\x89MCAP0\r\n"

# From the mcap specification. Only the ones we need to name are listed.
OP_HEADER = 0x01
OP_FOOTER = 0x02
OP_SCHEMA = 0x03
OP_CHANNEL = 0x04
OP_MESSAGE = 0x05
OP_CHUNK = 0x06
OP_MESSAGE_INDEX = 0x07
OP_DATA_END = 0x0F

OP_NAMES = {
    0x01: "Header", 0x02: "Footer", 0x03: "Schema", 0x04: "Channel",
    0x05: "Message", 0x06: "Chunk", 0x07: "MessageIndex", 0x08: "ChunkIndex",
    0x09: "Attachment", 0x0A: "AttachmentIndex", 0x0B: "Statistics",
    0x0C: "Metadata", 0x0D: "MetadataIndex", 0x0E: "SummaryOffset",
    0x0F: "DataEnd",
}

# Records that belong to the summary section. If the writer got far enough to
# emit these the file was very nearly finished, but they must not be left
# dangling in the data section of the repaired file.
SUMMARY_OPS = {0x08, 0x0A, 0x0B, 0x0D, 0x0E}


def scan(path):
    """Walk the records. Returns (end_offset, counts, complete).

    end_offset is where the last whole record finishes -- the point to cut.
    complete says whether the file already ends properly, in which case there
    is nothing to do.
    """
    size = os.path.getsize(path)
    counts = {}
    with open(path, "rb") as fh:
        if fh.read(len(MAGIC)) != MAGIC:
            raise SystemExit("%s does not start with the mcap magic -- this is "
                             "not an mcap file, or its beginning is damaged"
                             % path)

        # Already finished?
        if size > len(MAGIC) * 2:
            fh.seek(-len(MAGIC), os.SEEK_END)
            if fh.read(len(MAGIC)) == MAGIC:
                return size, counts, True

        fh.seek(len(MAGIC))
        offset = len(MAGIC)
        while True:
            head = fh.read(9)
            if len(head) < 9:
                break                      # ran out mid-header
            op = head[0]
            (length,) = struct.unpack("<Q", head[1:9])
            end = offset + 9 + length
            if end > size:
                sys.stderr.write(
                    "  cut inside a %s record at offset %d "
                    "(wanted %d bytes, %d remain)\n"
                    % (OP_NAMES.get(op, "0x%02x" % op), offset, length,
                       size - offset - 9))
                break
            if op in SUMMARY_OPS or op == OP_FOOTER:
                # The summary had begun. Everything from here is rebuildable,
                # and keeping it would produce a file that claims an index it
                # does not have.
                break
            counts[op] = counts.get(op, 0) + 1
            fh.seek(end)
            offset = end
        return offset, counts, False


def write_repaired(src, dst, cut):
    """Copy the good prefix, then close the file off properly."""
    with open(src, "rb") as fh_in, open(dst, "wb") as fh_out:
        remaining = cut
        while remaining > 0:
            block = fh_in.read(min(8 * 1024 * 1024, remaining))
            if not block:
                break
            fh_out.write(block)
            remaining -= len(block)

        # DataEnd: one uint32 data_section_crc. Zero means "not computed",
        # which is legal and saves re-reading the whole file.
        fh_out.write(struct.pack("<BQ", OP_DATA_END, 4))
        fh_out.write(struct.pack("<I", 0))

        # Footer: summary_start, summary_offset_start, summary_crc. All zero
        # means there is no summary section, and readers fall back to scanning.
        fh_out.write(struct.pack("<BQ", OP_FOOTER, 20))
        fh_out.write(struct.pack("<QQI", 0, 0, 0))

        fh_out.write(MAGIC)


def find_mcap(target):
    if os.path.isfile(target):
        return target, os.path.dirname(os.path.abspath(target))
    if os.path.isdir(target):
        for name in sorted(os.listdir(target)):
            if name.endswith(".mcap"):
                return os.path.join(target, name), os.path.abspath(target)
    raise SystemExit("no .mcap found at %s" % target)


def main():
    ap = argparse.ArgumentParser(
        description="repair a truncated rosbag2 mcap and rebuild its metadata")
    ap.add_argument("bag", help="the bag directory, or the .mcap itself")
    ap.add_argument("--out", default=None,
                    help="output directory (default: <bagdir>-recovered)")
    ap.add_argument("--no-reindex", action="store_true",
                    help="skip regenerating metadata.yaml")
    args = ap.parse_args()

    mcap, bagdir = find_mcap(args.bag)
    size = os.path.getsize(mcap)
    print("reading %s  (%.1f MB)" % (mcap, size / 1e6))

    # Say this before doing any work, not after. Rebuilding from the recording
    # gets the whole walk and an indexed file; this gets neither.
    session = os.path.dirname(bagdir.rstrip("/\\"))
    if os.path.isfile(os.path.join(session, "lidar.pcap")):
        print()
        print("  NOTE: the recording this came from is still on disk.")
        print("        Rebuilding is better than recovering -- you get the")
        print("        whole walk and a properly indexed file:")
        print("          walk_to_bag.sh %s" % session)
        print("        Carry on only if you want to see what survived.")
        print()

    cut, counts, complete = scan(mcap)

    if complete:
        print("\nthis file already ends properly -- nothing to repair.")
        if not os.path.exists(os.path.join(bagdir, "metadata.yaml")):
            print("but metadata.yaml is missing, so ros2 cannot open it:")
            print("  ros2 bag reindex %s -s mcap" % bagdir)
        return

    print("  records read:")
    for op in sorted(counts):
        print("    %-14s %d" % (OP_NAMES.get(op, "0x%02x" % op), counts[op]))
    lost = size - cut
    print("  usable: %.1f MB of %.1f MB  (discarding %d bytes, %.2f%%)"
          % (cut / 1e6, size / 1e6, lost, 100.0 * lost / size))

    if counts.get(OP_CHUNK, 0) == 0 and counts.get(OP_MESSAGE, 0) == 0:
        raise SystemExit(
            "\nno message or chunk records survived -- there is nothing to "
            "recover.\nRebuild the bag from the recording instead:\n"
            "  walk_to_bag.sh <session-directory>")

    out_dir = args.out or (bagdir.rstrip("/\\") + "-recovered")
    if os.path.exists(out_dir):
        raise SystemExit("%s already exists -- move it aside first" % out_dir)
    os.makedirs(out_dir)
    out_mcap = os.path.join(out_dir, os.path.basename(mcap))

    print("\nwriting %s" % out_mcap)
    write_repaired(mcap, out_mcap, cut)
    print("  %.1f MB" % (os.path.getsize(out_mcap) / 1e6))

    # Confirm our own work rather than assuming it.
    with open(out_mcap, "rb") as fh:
        fh.seek(-len(MAGIC), os.SEEK_END)
        if fh.read(len(MAGIC)) != MAGIC:
            raise SystemExit("the repaired file does not end correctly -- "
                             "this is a bug in recover_bag.py")
    print("  ends correctly")

    # The one caveat that will actually bite, said where it cannot be missed.
    out_size = os.path.getsize(out_mcap)
    if out_size > 600e6:
        print()
        print("  WARNING: %.0f MB, and this file has no index." % (out_size / 1e6))
        print("  Foxglove will refuse it: 'exceeded the maximum unindexed read")
        print("  size of 600.0 MB'. ros2 bag play and inspect_bag.py are fine")
        print("  with it -- they scan linearly. To open it in Foxglove either")
        print("  rebuild with walk_to_bag.sh, or index it with Foxglove's own")
        print("  tool:  mcap recover %s" % out_mcap)

    if args.no_reindex:
        print("\nnow rebuild the index:\n  ros2 bag reindex %s -s mcap" % out_dir)
        return

    # rosbag2 can rebuild metadata.yaml from the storage file itself. If the
    # tooling is not on the path -- this script deliberately does not need it --
    # say what to run rather than failing.
    print("\nrebuilding metadata.yaml")
    try:
        proc = subprocess.run(["ros2", "bag", "reindex", out_dir, "-s", "mcap"],
                              capture_output=True, text=True, timeout=600)
    except (OSError, subprocess.TimeoutExpired) as exc:
        print("  could not run ros2 (%s). Do it by hand:" % exc)
        print("    ros2 bag reindex %s -s mcap" % out_dir)
        return

    if os.path.exists(os.path.join(out_dir, "metadata.yaml")):
        print("  done")
        print("\ncheck it:")
        print("  ros2 bag info %s" % out_dir)
        print("  python3 %s %s"
              % (os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "inspect_bag.py"), out_dir))
    else:
        print("  reindex did not produce metadata.yaml")
        for line in (proc.stderr or proc.stdout or "").splitlines()[:6]:
            print("    %s" % line)
        print("  The .mcap itself is repaired and Foxglove will open it")
        print("  directly (File -> Open local file) with no metadata at all.")


if __name__ == "__main__":
    main()
