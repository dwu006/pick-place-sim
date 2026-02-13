"""
Setup script to download the Franka Emika Panda files we need from mujoco_menagerie.
Run: python setup_mujoco.py
"""
import os
import urllib.request
import json
from pathlib import Path

REPO_ROOT = Path(__file__).parent.parent
MENAGERIE_DIR = REPO_ROOT / "mujoco_menagerie"
PANDA_DIR = MENAGERIE_DIR / "franka_emika_panda"
PANDA_ASSETS_DIR = PANDA_DIR / "assets"

# Files we need from mujoco_menagerie/franka_emika_panda/
PANDA_FILES = [
    "panda.xml",
    "scene.xml",
]


def download_file(url: str, dest: Path, show_progress: bool = True) -> bool:
    """Download a single file from GitHub raw content."""
    if show_progress:
        print(f"Downloading {dest.name}...", end="", flush=True)
    dest.parent.mkdir(parents=True, exist_ok=True)
    try:
        urllib.request.urlretrieve(url, dest)
        if show_progress:
            size_mb = dest.stat().st_size / (1024 * 1024)
            print(f" OK ({size_mb:.1f} MB)")
        return True
    except Exception as e:
        if show_progress:
            print(f" ERROR: {e}")
        return False


def download_panda_assets() -> bool:
    """Download all asset files (mesh files) needed by panda.xml."""
    print("\nDownloading Panda asset files (this may take a minute)...")

    api_url = (
        "https://api.github.com/repos/google-deepmind/mujoco_menagerie"
        "/contents/franka_emika_panda/assets"
    )
    try:
        with urllib.request.urlopen(api_url) as response:
            assets_data = json.loads(response.read())

        downloaded = 0
        skipped = 0
        failed = 0
        for item in assets_data:
            if item["type"] != "file":
                continue
            filename = item["name"]
            url = item["download_url"]
            dest = PANDA_ASSETS_DIR / filename
            if dest.exists():
                skipped += 1
                continue
            if download_file(url, dest, show_progress=True):
                downloaded += 1
            else:
                failed += 1

        print(
            f"\nPanda assets summary: {downloaded} downloaded, "
            f"{skipped} already existed, {failed} failed"
        )
        return failed == 0
    except Exception as e:
        print(f"ERROR: Could not fetch Panda asset list: {e}")
        return False


def setup_panda() -> bool:
    """Download Franka Emika Panda files from mujoco_menagerie GitHub."""
    base_url = (
        "https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/main"
        "/franka_emika_panda"
    )

    print("Setting up Franka Emika Panda model files...")
    print(f"Target directory: {PANDA_DIR}")

    # Download XML files
    for filename in PANDA_FILES:
        url = f"{base_url}/{filename}"
        dest = PANDA_DIR / filename
        if not dest.exists():
            if not download_file(url, dest):
                print(f"ERROR: Failed to download {filename}")
                return False
        else:
            print(f"{filename} already exists, skipping")

    # Download assets (always check, as they might be missing)
    if not download_panda_assets():
        print(
            "\nWARNING: Some Panda assets failed to download. "
            "The scene may not load correctly."
        )
        print(
            "You can try cloning the full repo: "
            "git clone https://github.com/google-deepmind/mujoco_menagerie"
        )

    print(f"\nOK Setup complete! Panda files at: {PANDA_DIR}")
    return True


if __name__ == "__main__":
    success = setup_panda()
    if success:
        print("\nYou can now run: python view_scene.py")
    else:
        print("\nSetup failed. You can also clone the full repo:")
        print(f"  git clone https://github.com/google-deepmind/mujoco_menagerie {MENAGERIE_DIR}")
