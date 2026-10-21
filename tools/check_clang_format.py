#!/usr/bin/env python3
"""Reject C++ files that are not clang-format clean.

BravoFinder's style is enforced by the repo-root `.clang-format`
(Google base, C++20, ColumnLimit 100). This hook checks only the *staged*
content of `.h` / `.cc` (and a few common C++ extensions) -- i.e. exactly what
would be committed -- so an unstaged edit never triggers a false rejection.

It only *detects* violations; it never rewrites anything. On failure it prints
the offending files and the exact command to fix them:

  clang-format -i <file>...   # then `git add` the files and commit again

Shared by the local `pre-commit` hook (staged files) and the CI `lint` job
(`--all`: every tracked file as checked out). Mirrors the sibling
`check_commit_messages.py` (check-only, never rewrites).

Usage:
  tools/check_clang_format.py            # hook mode: check staged files
  tools/check_clang_format.py --all      # CI: check every tracked file
"""
import difflib
import os
import shutil
import subprocess
import sys
import tempfile

# C++ source extensions the project commits. The project standard is `.cc` /
# `.h`; the extras cover occasional headers/sources without changing behavior.
CPP_EXTS = {".cc", ".h", ".cpp", ".hpp", ".cxx", ".hh", ".hxx", ".c"}
CONFIG_NAMES = (".clang-format", "_clang-format")

# Cap the printed per-file diff so a wholesale reformat does not flood the
# terminal; the fix command still shows the exact files to reformat.
MAX_DIFF_LINES = 300


def find_clang_format():
    """Return the clang-format executable path, or None if absent."""
    for name in ("clang-format", "clang-format-18", "clang-format-17",
                 "clang-format-16"):
        path = shutil.which(name)
        if path:
            return path
    return None


def repo_root():
    return subprocess.check_output(
        ["git", "rev-parse", "--show-toplevel"], text=True
    ).strip()


def is_cpp(path):
    return os.path.splitext(path)[1].lower() in CPP_EXTS


def tracked_cpp_files():
    """Repo-relative paths of every tracked C++ file."""
    out = subprocess.check_output(["git", "ls-files"], text=True)
    return [line for line in out.splitlines() if is_cpp(line)]


def staged_cpp_files():
    """Repo-relative paths of staged C++ files (added/copied/modified/renamed)."""
    out = subprocess.check_output(
        ["git", "diff", "--cached", "--name-only", "--diff-filter=ACMR"],
        text=True,
    )
    files = []
    for line in out.splitlines():
        line = line.strip()
        if not line:
            continue
        if is_cpp(line):
            files.append(line)
    return files


def locate_config(root):
    for name in CONFIG_NAMES:
        p = os.path.join(root, name)
        if os.path.isfile(p):
            return p
    return None


def stage_blob(relpath, dest_dir, root, staged):
    """Write `relpath` into dest_dir, mirroring its tree path: the staged blob
    in hook mode, the checked-out file otherwise. Returns the absolute path of
    the written file."""
    if staged:
        out = subprocess.check_output(["git", "show", ":" + relpath])
    else:
        with open(os.path.join(root, relpath), "rb") as f:
            out = f.read()
    target = os.path.join(dest_dir, relpath)
    os.makedirs(os.path.dirname(target) or dest_dir, exist_ok=True)
    with open(target, "wb") as f:
        f.write(out)
    return target


def format_bytes(clang_format, abs_path):
    """Return (formatted_bytes, rc, stderr)."""
    proc = subprocess.run(
        [clang_format, "--style=file", abs_path],
        capture_output=True,
    )
    return proc.stdout, proc.returncode, proc.stderr


def main():
    args = sys.argv[1:]
    if args not in ([], ["--all"]):
        print(f"usage: {sys.argv[0]} [--all]", file=sys.stderr)
        return 2
    staged = not args
    clang_format = find_clang_format()
    if clang_format is None:
        print(
            "clang-format not found on PATH. Install it to enforce the repo "
            "style (e.g. `pip install clang-format`, or your package manager), "
            "then retry.",
            file=sys.stderr,
        )
        return 2

    try:
        root = repo_root()
    except subprocess.CalledProcessError as e:
        print(f"git rev-parse failed: {e}", file=sys.stderr)
        return 2

    files = staged_cpp_files() if staged else tracked_cpp_files()
    if not files:
        return 0

    config = locate_config(root)
    if config is None:
        print(
            "No .clang-format found at repo root; cannot determine the style.",
            file=sys.stderr,
        )
        return 2

    violations = []  # list of (relpath, original_bytes, formatted_bytes)
    tmp = tempfile.mkdtemp(prefix="bf-clang-")
    try:
        shutil.copy(config, os.path.join(tmp, os.path.basename(config)))
        for relpath in files:
            abs_path = stage_blob(relpath, tmp, root, staged)
            formatted, rc, err = format_bytes(clang_format, abs_path)
            if rc != 0:
                # clang-format itself failed (e.g. unparseable input); treat as
                # a violation so we never silently pass an unchecked file.
                print(
                    f"clang-format failed on {relpath}:\n"
                    f"{err.decode('utf-8', 'replace')}",
                    file=sys.stderr,
                )
                violations.append((relpath, b"", b""))
                continue
            with open(abs_path, "rb") as f:
                original = f.read()
            if formatted != original:
                violations.append((relpath, original, formatted))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if not violations:
        return 0

    bad = [v[0] for v in violations]
    if staged:
        print("Commit rejected: staged C++ file(s) are not clang-format clean.",
              file=sys.stderr)
        print("Reformat and re-stage, then commit again:", file=sys.stderr)
    else:
        print("C++ file(s) are not clang-format clean. Reformat with:",
              file=sys.stderr)
    print(f"  clang-format -i {' '.join(bad)}", file=sys.stderr)
    print("Offending file(s):", file=sys.stderr)
    for relpath in bad:
        print(f"  {relpath}", file=sys.stderr)

    # Show a diff per offending file (capped) to make the change obvious.
    for relpath, original, formatted in violations:
        if not original or not formatted:
            continue  # clang-format error case: nothing to diff
        orig_lines = original.decode("utf-8", "replace").splitlines(keepends=True)
        new_lines = formatted.decode("utf-8", "replace").splitlines(keepends=True)
        diff = list(difflib.unified_diff(
            orig_lines, new_lines, fromfile=relpath, tofile=relpath,
        ))
        if not diff:
            continue
        print("", file=sys.stderr)
        print(f"--- diff for {relpath} ---", file=sys.stderr)
        shown = diff[:MAX_DIFF_LINES]
        for line in shown:
            print(line.rstrip("\n"), file=sys.stderr)
        if len(diff) > MAX_DIFF_LINES:
            print(
                f"  ... ({len(diff) - MAX_DIFF_LINES} more diff lines; run "
                f"`clang-format -i {relpath}` to see all)",
                file=sys.stderr,
            )
    return 1


if __name__ == "__main__":
    sys.exit(main())
