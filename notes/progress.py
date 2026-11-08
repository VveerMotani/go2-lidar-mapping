"""How far a long step has got, in one line format for people and for Studio.

    ~ colouring the cloud: 600/1500 frames      a count
    ~ building the mesh surface                 a phase with no count

In a terminal these are a steady count; Go2 Studio (studio/core/jobs.py) reads
them into its progress bar and the line under a walk. A count prints at most
every two seconds, and always at the end, so a log stays short.

    from progress import Progress, phase
    p = Progress("colouring the cloud", len(frames), "frames")
    for f in frames:
        ...
        p.step()
    p.end()
"""
import time


def phase(what):
    print("  ~ %s" % what, flush=True)


class Progress(object):
    def __init__(self, what, total, unit="", every=2.0):
        self.what, self.total, self.unit, self.every = what, max(0, int(total)), unit, every
        self.done = 0
        self.last = 0.0
        self._print()

    def step(self, n=1):
        self.done += n
        now = time.time()
        if now - self.last >= self.every:
            self._print(now)

    def end(self):
        if self.total and self.done < self.total:
            self.done = self.total
        if self.printed != self.done:
            self._print()

    def _print(self, now=None):
        self.last = now or time.time()
        self.printed = self.done
        print("  ~ %s: %d/%d%s" % (self.what, min(self.done, self.total or self.done),
                                   self.total or self.done, " " + self.unit if self.unit else ""),
              flush=True)
