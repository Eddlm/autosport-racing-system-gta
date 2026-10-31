"""Print one line of commit cadence, so the build itself says whether a commit is owed.

The rule for this repository: one commit per ten builds (a build number ending in 0) or when
the topic changes. The build is the moment the decision comes up, so the number that decides
it belongs in the build output rather than in somebody's memory.

Reads the build number, the last commit's message (each commit ends with "Build <n>"), whether
the tree is dirty and how many commits are unpushed. Always exits 0 and prints exactly one line:
the csproj captures it with ConsoleToMsBuild and re-emits it at high importance, which is what
makes it visible at /v:minimal.

Run by hand: python docs/build-reminder.py
"""
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def git(*args):
    try:
        done = subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True, timeout=15)
        return done.stdout.strip()
    except Exception:
        return ""


def main():
    try:
        now = int((ROOT / "build" / "build-number.txt").read_text().strip())
    except Exception:
        print("commit: build number unreadable")
        return

    body = git("log", "-1", "--format=%B")
    cited = re.findall(r"Build (\d+)", body)
    last = int(cited[-1]) if cited else None
    head = git("log", "-1", "--format=%h")
    dirty = bool(git("status", "--porcelain"))
    unpushed = git("rev-list", "--count", "origin/master..HEAD")

    parts = ["build %d" % now]
    if last is None:
        parts.append("no commit cites a build")
    else:
        since = now - last
        parts.append("last commit %d%s%s" % (last, " (" + head + ")" if head else "",
                                             "" if since == 0 else " - %d build%s ago" % (since, "" if since == 1 else "s")))
    parts.append("tree " + ("dirty" if dirty else "clean"))

    if now % 10 == 0 and (last is None or last < now):
        parts.append("BATCH POINT - commit owed")
    else:
        nxt = (now // 10 + 1) * 10
        parts.append("next batch point %d, in %d build%s (or on a topic change)"
                     % (nxt, nxt - now, "" if nxt - now == 1 else "s"))
    if unpushed and unpushed != "0":
        parts.append("%s unpushed" % unpushed)

    print("commit: " + " | ".join(parts))


if __name__ == "__main__":
    try:
        main()
    except Exception as error:  # never fail a build over a reminder
        print("commit: reminder failed (%s)" % error)
