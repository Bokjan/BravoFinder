#!/usr/bin/env python3
"""Reject commit messages that hard-wrap markdown paragraphs.

BravoFinder's no-hard-wrap discipline (see CLAUDE.md) requires every markdown
paragraph -- including commit-message bodies -- to be one logical line; the
renderer soft-wraps. A hard-wrapped paragraph folds sentences at a column
boundary across multiple lines, which renders as broken line breaks and makes
diffs noisy.

This script is shared by the local `commit-msg` hook and the CI `commitlint`
workflow. It only *detects* violations; it never rewrites anything.

Usage:
  tools/check_commit_messages.py --range <git-rev-list-range>   # CI: check a range
  tools/check_commit_messages.py <message-file>                # hook: check one file
  cat msg | tools/check_commit_messages.py --stdin             # check stdin
"""
import re
import subprocess
import sys

# A line ending a sentence. A line that does NOT end here and is followed by a
# continuation (not a fresh list item / sentence) is a hard-wrap break.
SENT_END = (".", "?", "!")
LIST_RE = re.compile(r"^\s*([-*+]|\d+\.|[a-zA-Z]\.|>) ")
FENCE_RE = re.compile(r"^\s*```")
BOX_RE = re.compile(r"[─│┌┐└┘├┤┬┴┼═║╔╗╚╝╠╣╦╩╬]")
HEADER_RE = re.compile(r"^\s{0,3}#{1,6}\s")
COMMENT_RE = re.compile(r"^\s*#")  # git's "# Please enter..." trailer lines


def is_list(line):
    return bool(LIST_RE.match(line))


def is_code_marker(line):
    return bool(FENCE_RE.match(line))


def is_box(line):
    return bool(BOX_RE.search(line))


def is_header(line):
    return bool(HEADER_RE.match(line))


def is_comment(line):
    return bool(COMMENT_RE.match(line))


def block_kind(lines):
    """Classify a block of consecutive non-blank lines."""
    nb = [l for l in lines if l.strip() != ""]
    if not nb:
        return "empty"
    # git trailer comments are not part of the message; treat as ignorable.
    if all(is_comment(l) for l in nb):
        return "comment"
    if any(is_header(l) for l in nb):
        return "header"
    if any(is_code_marker(l) for l in nb):
        return "code"
    if any(is_box(l) for l in nb):
        return "box"
    if sum(1 for l in nb if " -> " in l) >= max(1, len(nb) // 2):
        return "renamelist"
    if any(is_list(l) for l in nb):
        return "list"
    if any("|" in l and l.strip().startswith("|") is False for l in nb):
        return "table"
    return "text"


def text_is_hardwrapped(lines):
    nb = [l for l in lines if l.strip() != ""]
    if len(nb) < 2:
        return False
    mid = 0
    for i in range(len(nb) - 1):
        cur = nb[i].rstrip()
        nxt = nb[i + 1].lstrip()
        if cur.endswith(SENT_END):
            continue
        if is_list(nxt) or is_comment(nxt):
            continue
        mid += 1
    # Require a majority of internal breaks to be mid-sentence so that
    # intentional separate sentences (each on its own line) are not flagged.
    return mid >= max(1, (len(nb) - 1) // 2)


def find_violations(text):
    """Return a list of (line_index, preview) for hard-wrapped text blocks."""
    text = text.replace("\r\n", "\n").replace("\r", "\n")
    raw = text.split("\n")
    blocks, cur, start = [], [], 0
    violations = []
    line_no = 0
    for ln in raw:
        if ln.strip() == "":
            if cur:
                blocks.append((start, cur))
            cur = []
            start = None
        else:
            if start is None:
                start = line_no
            cur.append(ln)
        line_no += 1
    if cur:
        blocks.append((start, cur))
    for start, blk in blocks:
        if block_kind(blk) == "text" and text_is_hardwrapped(blk):
            preview = blk[0].strip()[:72]
            violations.append((start + 1, preview))
    return violations


def check_one(sha, message):
    v = find_violations(message)
    if not v:
        return True
    print(f"  hard-wrapped paragraph(s) in {sha}:", file=sys.stderr)
    for ln, preview in v:
        print(f"    line {ln}: {preview}", file=sys.stderr)
    return False


def main():
    args = sys.argv[1:]
    if args and args[0] == "--range":
        rev_range = args[1] if len(args) > 1 else "HEAD"
        try:
            out = subprocess.check_output(
                ["git", "log", "--format=%H%x00%B%x1e", rev_range],
                text=True,
            )
        except subprocess.CalledProcessError as e:
            print(f"git log failed: {e}", file=sys.stderr)
            return 2
        commits = [c for c in out.split("\x1e") if c.strip() != ""]
        ok = True
        for c in commits:
            c = c.strip("\x00")
            sha, _, body = c.partition("\x00")
            if not check_one(sha, body):
                ok = False
        if not ok:
            print(
                "Commit message(s) hard-wrap a paragraph. Keep each paragraph on "
                "one logical line (the renderer soft-wraps).",
                file=sys.stderr,
            )
            return 1
        return 0

    # hook / stdin mode: validate a single message
    if args and args[0] == "--stdin":
        data = sys.stdin.read()
    elif args and args[0] != "--stdin":
        with open(args[0], encoding="utf-8") as f:
            data = f.read()
    else:
        data = sys.stdin.read()
    v = find_violations(data)
    if v:
        print("Commit message hard-wraps a paragraph:", file=sys.stderr)
        for ln, preview in v:
            print(f"  line {ln}: {preview}", file=sys.stderr)
        print(
            "Keep each paragraph on one logical line (the renderer soft-wraps).",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
