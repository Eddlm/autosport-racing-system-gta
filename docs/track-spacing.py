"""Node spacing report for ARS route files.

Reads the <Route> block of each track XML and reports the distribution of the horizontal
distance between consecutive nodes. The curvature law advances by node index, so it treats
one node as one metre; this measures how true that is.

Usage:
    python docs/track-spacing.py                       # every track in the game folder
    python docs/track-spacing.py "path/to/Track.xml"   # one file
    python docs/track-spacing.py --long "path/to/Track.xml"  # list the long steps
"""

import glob
import math
import os
import re
import sys

TRACKS = (r"D:/SteamLibrary/steamapps/common/Grand Theft Auto V/Scripts/AutosportRacingSystem"
          r"/Tracks/*.xml")


def route_points(path):
    text = open(path, encoding="utf-8", errors="replace").read()
    block = re.search(r"<Route>(.*?)</Route>", text, re.S)
    if not block:
        return []
    pairs = re.findall(r"<X>(-?[\d.]+)</X>\s*<Y>(-?[\d.]+)</Y>", block.group(1))
    return [(float(x), float(y)) for x, y in pairs]


def report(path, list_long):
    points = route_points(path)
    if len(points) < 3:
        print(f"{os.path.basename(path)}: no route block")
        return
    steps = sorted(math.dist(points[i], points[i + 1]) for i in range(len(points) - 1))
    n = len(steps)
    over = sum(1 for s in steps if s > 1.5)
    under = sum(1 for s in steps if s < 0.5)
    print(f"{os.path.basename(path)[:26]:26} "
          f"nodes={len(points):5} "
          f"min={steps[0]:.4f} p50={steps[n // 2]:.4f} p99={steps[int(n * 0.99)]:.4f} "
          f"max={steps[-1]:.4f}  >1.5m={over:4} ({100 * over / n:.1f}%)  <0.5m={under:3}")
    if list_long:
        pairs = [(i, math.dist(points[i], points[i + 1]))
                 for i in range(len(points) - 1)
                 if math.dist(points[i], points[i + 1]) > 1.5]
        print("    long steps (node -> node+1, metres):",
              ", ".join(f"{i}->{i + 1} ({d:.3f})" for i, d in pairs))


if __name__ == "__main__":
    args = sys.argv[1:]
    list_long = "--long" in args
    targets = [a for a in args if a != "--long"] or sorted(glob.glob(TRACKS))
    for target in targets:
        report(target, list_long)
