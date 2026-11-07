#!/usr/bin/env python3
"""Copy the projects listed in sources.yml into top-level folders of this repo.

Each project is a snapshot of one private repo: the files at its branch tip,
with no git history. Run with --help for options. Needs Python 3.11+, git,
PyYAML (scripts/requirements.txt), and (for --releases) the gh CLI.

Environment:
  SOURCE_TOKEN  token that can read the source repos (required unless --check)
  GH_TOKEN      token that can create releases in this repo (--releases only)
"""

from __future__ import annotations

import argparse
import base64
import fnmatch
import json
import os
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

import yaml

OWNER = "2702rebels"
ROOT = Path(__file__).resolve().parent.parent
CONFIG = ROOT / "sources.yml"
STATE = ROOT / ".github" / "sync-state.json"
README = ROOT / "README.md"
GITLEAKS_CONFIG = ROOT / ".gitleaks.toml"
INDEX_START = "<!-- projects:start -->"
INDEX_END = "<!-- projects:end -->"

# Top-level names a project may not use, because this repo's own files live there.
RESERVED = {"scripts", "readme.md", "license", "sources.yml"}
NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")
LFS_POINTER = b"version https://git-lfs.github.com/spec/v1"
FIELDS = {"repo", "path", "ref", "subdir", "description", "exclude", "releases"}


class SyncError(Exception):
    pass


@dataclass
class Project:
    repo: str
    path: str
    ref: str = "HEAD"
    subdir: str = ""
    description: str = ""
    exclude: list[str] = field(default_factory=list)
    releases: bool = False

    @property
    def full_name(self) -> str:
        return f"{OWNER}/{self.repo}"


# ---------------------------------------------------------------- config


def load_config() -> list[Project]:
    try:
        data = yaml.safe_load(CONFIG.read_text()) or {}
    except yaml.YAMLError as e:
        raise SyncError(f"sources.yml is not valid YAML: {e}")
    if not isinstance(data, dict):
        raise SyncError("sources.yml must be a mapping with a 'projects' list")
    unknown_top = set(data) - {"projects"}
    if unknown_top:
        raise SyncError(f"sources.yml: unknown top-level keys {sorted(unknown_top)}")

    projects, seen = [], set()
    entries = data.get("projects") or []
    if not isinstance(entries, list):
        raise SyncError("sources.yml: 'projects' must be a list")
    for i, raw in enumerate(entries, 1):
        where = f"sources.yml project #{i}"
        if not isinstance(raw, dict):
            raise SyncError(f"{where}: each project must be a mapping like '- repo: 2026-robot'")
        unknown = set(raw) - FIELDS
        if unknown:
            raise SyncError(f"{where}: unknown keys {sorted(unknown)} (allowed: {sorted(FIELDS)})")
        if not isinstance(raw.get("repo"), str) or not NAME_RE.match(raw["repo"]):
            raise SyncError(f"{where}: 'repo' must be a repo name in {OWNER}, like \"2026-robot\"")
        p = Project(
            repo=raw["repo"],
            path=raw.get("path", raw["repo"]),
            ref=raw.get("ref", "HEAD"),
            subdir=raw.get("subdir", "").strip("/"),
            description=raw.get("description", ""),
            exclude=raw.get("exclude", []),
            releases=raw.get("releases", False),
        )
        where = f"sources.yml project '{p.path}'"
        if not isinstance(p.path, str) or not NAME_RE.match(p.path) or p.path.lower() in RESERVED:
            raise SyncError(f"{where}: 'path' must be a plain folder name, not one of {sorted(RESERVED)}")
        if p.path.lower() in seen:
            raise SyncError(f"{where}: path is used by more than one project")
        seen.add(p.path.lower())
        if not isinstance(p.ref, str) or not p.ref or p.ref.startswith("-"):
            raise SyncError(f"{where}: 'ref' must be a branch, tag, or commit")
        if not isinstance(p.subdir, str) or ".." in p.subdir.split("/"):
            raise SyncError(f"{where}: 'subdir' must be a folder inside the source repo")
        if not isinstance(p.description, str) or "\n" in p.description:
            raise SyncError(f"{where}: 'description' must be a one-line string")
        if not isinstance(p.exclude, list) or not all(isinstance(x, str) and x for x in p.exclude):
            raise SyncError(f"{where}: 'exclude' must be a list of glob strings")
        if not isinstance(p.releases, bool):
            raise SyncError(f"{where}: 'releases' must be true or false")
        projects.append(p)
    return projects


def load_state() -> dict:
    return json.loads(STATE.read_text()) if STATE.exists() else {}


def save_state(state: dict) -> None:
    STATE.parent.mkdir(parents=True, exist_ok=True)
    STATE.write_text(json.dumps(dict(sorted(state.items())), indent=2) + "\n")


# ---------------------------------------------------------------- helpers


def run(cmd: list[str], *, cwd: Path | None = None, env: dict | None = None,
        capture: bool = True, check: bool = True) -> subprocess.CompletedProcess:
    full_env = {**os.environ, **(env or {})}
    result = subprocess.run(cmd, cwd=cwd, env=full_env, text=True,
                            capture_output=capture)
    if check and result.returncode != 0:
        detail = (result.stderr or result.stdout or "").strip()
        raise SyncError(f"`{' '.join(cmd[:3])} ...` failed: {detail}")
    return result


def git(*args: str, cwd: Path = ROOT, **kw) -> str:
    return run(["git", *args], cwd=cwd, **kw).stdout.strip()


def auth_env(token: str) -> dict:
    # Pass the token as git config through the environment, so it never shows up in
    # argv, remote URLs, or error messages. (Actions masks the raw token in logs, but
    # not this base64 form, so it must never be printed.)
    basic = base64.b64encode(f"x-access-token:{token}".encode()).decode()
    return {"GIT_CONFIG_COUNT": "1",
            "GIT_CONFIG_KEY_0": "http.https://github.com/.extraheader",
            "GIT_CONFIG_VALUE_0": f"AUTHORIZATION: basic {basic}",
            "GIT_TERMINAL_PROMPT": "0"}


def excluded(rel: str, patterns: list[str]) -> bool:
    parts = rel.split("/")
    prefixes = ["/".join(parts[: i + 1]) for i in range(len(parts))]
    return any(fnmatch.fnmatchcase(pre, pat.strip("/")) for pat in patterns for pre in prefixes)


def tracked(path: str) -> bool:
    return bool(git("ls-files", "--", path))


# ---------------------------------------------------------------- snapshot


def fetch_snapshot(p: Project, token: str, work: Path) -> tuple[Path, str, str]:
    """Fetch the source tip and unpack its files into work/out. Returns (dir, sha, date)."""
    src = work / "src"
    git("init", "-q", str(src), cwd=work)
    try:
        run(["git", "fetch", "-q", "--depth", "1", "--no-tags",
             f"https://github.com/{p.full_name}.git", p.ref], cwd=src, env=auth_env(token))
    except SyncError as e:
        raise SyncError(f"could not fetch {p.full_name} ({p.ref}); does the sync app have access? {e}")
    sha = git("rev-parse", "FETCH_HEAD", cwd=src)
    date = git("log", "-1", "--format=%cs", "FETCH_HEAD", cwd=src)

    # git archive honours export-ignore in the source's .gitattributes and skips .git.
    tar_path = work / "snap.tar"
    pathspec = [p.subdir] if p.subdir else []
    with open(tar_path, "wb") as fh:
        r = subprocess.run(["git", "archive", "--format=tar", "FETCH_HEAD", *pathspec],
                           cwd=src, stdout=fh, stderr=subprocess.PIPE, text=False)
    if r.returncode != 0:
        raise SyncError(f"git archive failed: {r.stderr.decode().strip()}")

    out = work / "out"
    out.mkdir()
    prefix = f"{p.subdir}/" if p.subdir else ""
    with tarfile.open(tar_path) as tar:
        members = []
        for m in tar.getmembers():
            if prefix and not m.name.startswith(prefix):
                continue
            m.name = m.name[len(prefix):]
            if not m.name or excluded(m.name, p.exclude):
                continue
            members.append(m)
        if not any(m.isfile() for m in members):
            raise SyncError(f"no files to publish (check 'subdir' and 'exclude')")
        tar.extractall(out, members=members, filter="data")
    return out, sha, date


def check_lfs(out: Path) -> None:
    pointers = []
    for f in out.rglob("*"):
        if f.is_file() and not f.is_symlink() and f.stat().st_size < 1024:
            with open(f, "rb") as fh:
                if fh.read(len(LFS_POINTER)) == LFS_POINTER:
                    pointers.append(str(f.relative_to(out)))
    if pointers:
        raise SyncError(
            "these files are stored in Git LFS, which this sync doesn't copy; exclude them "
            "and publish them as release files instead: " + ", ".join(pointers[:10]))


def scan_secrets(out: Path, work: Path, required: bool) -> None:
    if not shutil.which("gitleaks"):
        if required:
            raise SyncError("gitleaks is not installed, and --require-scan was given")
        print("  ! gitleaks not installed; skipping secret scan", file=sys.stderr)
        return
    report = work / "gitleaks.json"
    cmd = ["gitleaks", "dir", str(out), "--no-banner", "--log-level", "error",
           "--redact", "--exit-code", "2", "--report-format", "json", "--report-path", str(report)]
    if GITLEAKS_CONFIG.exists():
        cmd += ["--config", str(GITLEAKS_CONFIG)]
    r = run(cmd, check=False)
    if r.returncode == 0:
        return
    if r.returncode != 2:
        raise SyncError(f"gitleaks failed: {(r.stderr or r.stdout).strip()}")
    # Only print where the finding is, never its content: Actions logs on a public repo are public.
    findings = json.loads(report.read_text() or "[]")
    def rel(name: str) -> str:
        try:
            return str(Path(name).resolve().relative_to(out.resolve()))
        except ValueError:
            return name
    where = sorted({f"{rel(f['File'])}:{f['StartLine']} ({f['RuleID']})" for f in findings})
    raise SyncError("possible secrets found, so nothing was published. Remove them from the source "
                    "repo (and rotate them), or exclude the files: " + "; ".join(where))


# ---------------------------------------------------------------- index


def write_readme(projects: list[Project], state: dict) -> None:
    if not README.exists():
        return
    text = README.read_text()
    if INDEX_START not in text or INDEX_END not in text:
        return
    rows = ["| Project | What it is | Updated |", "| --- | --- | --- |"]
    for p in sorted(projects, key=lambda p: p.path.lower()):
        s = state.get(p.path)
        if not s:
            continue
        updated = f"{s['date']} (`{s['commit'][:7]}`)"
        rows.append(f"| [{p.path}]({p.path}/) | {p.description} | {updated} |")
    table = "\n".join(rows) if len(rows) > 2 else "_Nothing published yet._"
    head, rest = text.split(INDEX_START, 1)
    _, tail = rest.split(INDEX_END, 1)
    README.write_text(f"{head}{INDEX_START}\n{table}\n{INDEX_END}{tail}")


def commit(message: str, paths: list[str], enabled: bool) -> bool:
    paths = [x for x in paths if Path(ROOT, x).exists() or tracked(x)]
    if paths:
        git("add", "-A", "--", *paths)
    if run(["git", "diff", "--cached", "--quiet"], cwd=ROOT, check=False).returncode == 0:
        return False
    if enabled:
        git("commit", "-q", "-m", message)
    return True


# ---------------------------------------------------------------- releases


def gh_json_lines(args: list[str], token: str) -> list[dict]:
    out = run(["gh", "api", "--paginate", *args, "--jq", ".[]"], env={"GH_TOKEN": token}).stdout
    return [json.loads(line) for line in out.splitlines() if line.strip()]


def mirror_releases(p: Project, source_token: str, target_repo: str, write_token: str,
                    dry_run: bool) -> int:
    existing = {r["tag_name"] for r in gh_json_lines([f"repos/{target_repo}/releases"], write_token)}
    source = gh_json_lines([f"repos/{p.full_name}/releases"], source_token)
    created = 0
    for rel in sorted(source, key=lambda r: r.get("published_at") or ""):
        if rel["draft"]:
            continue
        tag = f"{p.path}/{rel['tag_name']}"
        if tag in existing:
            continue
        print(f"  + release {tag} ({len(rel['assets'])} files)")
        created += 1
        if dry_run:
            continue
        with tempfile.TemporaryDirectory() as d:
            files = []
            for a in rel["assets"]:
                dest = Path(d) / a["name"]
                with open(dest, "wb") as fh:
                    r = subprocess.run(
                        ["gh", "api", "-H", "Accept: application/octet-stream",
                         f"repos/{p.full_name}/releases/assets/{a['id']}"],
                        env={**os.environ, "GH_TOKEN": source_token}, stdout=fh, stderr=subprocess.PIPE)
                if r.returncode != 0:
                    raise SyncError(f"could not download {a['name']}: {r.stderr.decode().strip()}")
                files.append(str(dest))
            notes = Path(d) / ".notes.md"
            notes.write_text(f"Copied from the `{rel['tag_name']}` release of [{p.path}]({p.path}/).\n\n"
                             + (rel.get("body") or ""))
            cmd = ["gh", "release", "create", tag, *files, "--repo", target_repo,
                   "--title", f"{p.path} {rel.get('name') or rel['tag_name']}",
                   "--notes-file", str(notes), "--target", "main"]
            if rel["prerelease"]:
                cmd.append("--prerelease")
            run(cmd, env={"GH_TOKEN": write_token})
    return created


# ---------------------------------------------------------------- main


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="only validate sources.yml")
    ap.add_argument("--list-repos", action="store_true",
                    help="print the source repo names, comma-separated, and exit")
    ap.add_argument("--only", metavar="PATH", help="sync just this project")
    ap.add_argument("--no-commit", action="store_true",
                    help="update the files but leave the changes uncommitted (for trying it out)")
    ap.add_argument("--push", action="store_true", help="push the sync commits to origin/main")
    ap.add_argument("--releases", action="store_true", help="also copy releases for projects with releases = true")
    ap.add_argument("--require-scan", action="store_true", help="fail if gitleaks is not installed")
    args = ap.parse_args()

    try:
        projects = load_config()
    except SyncError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    if args.check:
        print(f"sources.yml OK: {len(projects)} project(s)")
        return 0
    if args.list_repos:
        print(",".join(sorted({p.repo for p in projects})))
        return 0

    token = os.environ.get("SOURCE_TOKEN")
    if not token:
        print("error: set SOURCE_TOKEN to a token that can read the source repos", file=sys.stderr)
        return 1
    if args.only and args.only not in {p.path for p in projects}:
        print(f"error: no project with path '{args.only}' in sources.yml", file=sys.stderr)
        return 1

    do_commit = not args.no_commit
    state = load_state()
    failures: list[str] = []
    changed = 0
    selected = [p for p in projects if not args.only or p.path == args.only]

    for p in selected:
        print(f"{p.path}  <-  {p.full_name}" + (f":{p.subdir}" if p.subdir else "") + f" @ {p.ref}")
        if p.path not in state and (ROOT / p.path).exists() and tracked(p.path):
            failures.append(f"{p.path}: folder already exists and wasn't created by sync; pick another path")
            print(f"  ! {failures[-1]}", file=sys.stderr)
            continue
        try:
            with tempfile.TemporaryDirectory() as tmp:
                work = Path(tmp)
                out, sha, date = fetch_snapshot(p, token, work)
                check_lfs(out)
                scan_secrets(out, work, args.require_scan)
                dest = ROOT / p.path
                if dest.exists():
                    shutil.rmtree(dest)
                shutil.move(str(out), dest)
            state[p.path] = {"repo": p.full_name, "ref": p.ref, "commit": sha, "date": date}
            save_state(state)
            write_readme(projects, state)
            if commit(f"Sync {p.path} from {p.repo}@{sha[:7]}\n\nSource-Commit: {sha}",
                      [p.path, str(STATE), str(README)], do_commit):
                changed += 1
                print(f"  updated to {sha[:7]} ({date})")
            else:
                print(f"  up to date ({sha[:7]})")
        except SyncError as e:
            failures.append(f"{p.path}: {e}")
            print(f"  ! {e}", file=sys.stderr)

    # Folders whose entry was removed from sources.yml.
    if not args.only:
        configured = {p.path for p in projects}
        for path in sorted(set(state) - configured):
            print(f"{path}  removed from sources.yml; deleting folder")
            if (ROOT / path).exists():
                shutil.rmtree(ROOT / path)
            del state[path]
            save_state(state)
            write_readme(projects, state)
            if commit(f"Remove {path} (no longer in sources.yml)", [path, str(STATE), str(README)], do_commit):
                changed += 1

    # Description edits in sources.yml only touch the README.
    write_readme(projects, state)
    if commit("Update project index", [str(README)], do_commit):
        changed += 1

    if args.push and do_commit and changed:
        if run(["git", "push", "-q", "origin", "HEAD:main"], cwd=ROOT, check=False).returncode != 0:
            # Someone merged a PR while we ran; replay our commits on top and retry once.
            git("pull", "-q", "--rebase", "origin", "main")
            git("push", "-q", "origin", "HEAD:main")
        print(f"pushed {changed} commit(s)")

    if args.releases:
        target = os.environ.get("GITHUB_REPOSITORY", f"{OWNER}/public")
        write_token = os.environ.get("GH_TOKEN")
        for p in selected:
            if not p.releases or p.path not in state:
                continue
            if not write_token:
                failures.append(f"{p.path}: set GH_TOKEN to copy releases")
                continue
            print(f"{p.path}  releases")
            try:
                mirror_releases(p, token, target, write_token, dry_run=not args.push)
            except SyncError as e:
                failures.append(f"{p.path} releases: {e}")
                print(f"  ! {e}", file=sys.stderr)

    if failures:
        print("\nFailed:\n  " + "\n  ".join(failures), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
