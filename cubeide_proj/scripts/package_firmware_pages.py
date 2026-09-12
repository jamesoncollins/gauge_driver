#!/usr/bin/env python3
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
import shutil
import subprocess
from pathlib import Path

PRESETS = {
    "st7789vi-release": "ST7789VI LCD",
    "s6e63d6-release": "S6E63D6 OLED",
}


def git(repo: Path, *args: str) -> str:
    return subprocess.check_output(["git", "-C", str(repo), *args], text=True).strip()


def safe_ref(name: str) -> str:
    out = []
    for ch in name:
        if ch.isalnum() or ch in "._-":
            out.append(ch)
        else:
            out.append("_")
    return "".join(out).strip("._-") or "unnamed"


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def load_manifest(path: Path) -> dict:
    if not path.exists():
        return {
            "schema": 1,
            "generated_at": None,
            "source": "github-pages",
            "firmware": [],
        }
    return json.loads(path.read_text(encoding="utf-8"))


def write_manifest(path: Path, manifest: dict) -> None:
    manifest["generated_at"] = dt.datetime.now(dt.timezone.utc).isoformat()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def copy_web(root: Path, site: Path) -> None:
    webui = root / "web" / "webui.html"
    if not webui.exists():
        raise SystemExit(f"missing updater page: {webui}")
    site.mkdir(parents=True, exist_ok=True)
    shutil.copy2(webui, site / "webui.html")
    shutil.copy2(webui, site / "index.html")


def add_firmware(root: Path, site: Path, worktree: Path, ref_type: str, ref_name: str, preset: str, pages_base_url: str, run_url: str) -> None:
    if preset not in PRESETS:
        raise SystemExit(f"unsupported preset: {preset}")

    bin_path = worktree / "cubeide_proj" / "build" / preset / "gauge_driver.bin"
    if not bin_path.exists():
        raise SystemExit(f"missing firmware binary: {bin_path}")

    safe_name = safe_ref(ref_name)
    short_sha = git(worktree, "rev-parse", "--short=12", "HEAD")
    commit_sha = git(worktree, "rev-parse", "HEAD")
    commit_date = git(worktree, "show", "-s", "--format=%cI", "HEAD")
    commit_subject = git(worktree, "show", "-s", "--format=%s", "HEAD")

    ref_dir = "branches" if ref_type == "branch" else "tags"
    rel_dir = Path("firmware") / ref_dir / safe_name / preset
    dest_dir = site / rel_dir
    dest_dir.mkdir(parents=True, exist_ok=True)
    dest = dest_dir / "gauge_driver.bin"
    shutil.copy2(bin_path, dest)

    manifest_path = site / "firmware" / "manifest.json"
    manifest = load_manifest(manifest_path)
    url_path = (rel_dir / "gauge_driver.bin").as_posix()
    base = pages_base_url.rstrip("/")
    url = f"{base}/{url_path}" if base else url_path

    entry = {
        "ref_type": ref_type,
        "ref_name": ref_name,
        "ref_safe_name": safe_name,
        "preset": preset,
        "display": PRESETS[preset],
        "build_type": "Release",
        "commit_sha": commit_sha,
        "commit_short_sha": short_sha,
        "commit_date": commit_date,
        "commit_subject": commit_subject,
        "size": dest.stat().st_size,
        "sha256": sha256(dest),
        "path": url_path,
        "url": url,
        "workflow_run_url": run_url,
    }

    manifest["firmware"] = [
        item for item in manifest.get("firmware", [])
        if not (item.get("ref_type") == ref_type and item.get("ref_name") == ref_name and item.get("preset") == preset)
    ]
    manifest["firmware"].append(entry)
    manifest["firmware"].sort(key=lambda item: (item.get("ref_type", ""), item.get("ref_name", ""), item.get("preset", "")))
    write_manifest(manifest_path, manifest)


def default_run_url() -> str:
    server = os.environ.get("GITHUB_SERVER_URL", "").rstrip("/")
    repo = os.environ.get("GITHUB_REPOSITORY", "").strip("/")
    run_id = os.environ.get("GITHUB_RUN_ID", "")
    if not (server and repo and run_id):
        return ""
    return f"{server}/{repo}/actions/runs/{run_id}"


def main() -> int:
    parser = argparse.ArgumentParser(description="Package firmware binaries and manifest for GitHub Pages.")
    sub = parser.add_subparsers(dest="cmd", required=True)

    init = sub.add_parser("init")
    init.add_argument("--root", type=Path, required=True)
    init.add_argument("--site", type=Path, required=True)

    add = sub.add_parser("add")
    add.add_argument("--root", type=Path, required=True)
    add.add_argument("--site", type=Path, required=True)
    add.add_argument("--worktree", type=Path, required=True)
    add.add_argument("--ref-type", choices=["branch", "tag"], required=True)
    add.add_argument("--ref-name", required=True)
    add.add_argument("--preset", choices=sorted(PRESETS), required=True)
    add.add_argument("--pages-base-url", default=os.environ.get("PAGES_BASE_URL", ""))
    add.add_argument("--run-url", default=default_run_url())

    args = parser.parse_args()
    if args.cmd == "init":
        copy_web(args.root.resolve(), args.site.resolve())
        write_manifest(args.site.resolve() / "firmware" / "manifest.json", load_manifest(args.site.resolve() / "firmware" / "manifest.json"))
        return 0
    if args.cmd == "add":
        add_firmware(args.root.resolve(), args.site.resolve(), args.worktree.resolve(), args.ref_type, args.ref_name, args.preset, args.pages_base_url, args.run_url)
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
