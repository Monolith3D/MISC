#!/bin/bash
# Link beacon_accel_homing.py into Klipper or Kalico (same approach as motors-sync).
#   ./install.sh       install, and offer the Moonraker update_manager entry
#   ./install.sh -u    uninstall (removes the link only)
# KLIPPER_PATH and MOONRAKER_CONFIG override the default locations.
set -e
repo=beacon_accel_homing
repo_path="$(cd "$(dirname "$0")" && pwd)"
module_name=beacon_accel_homing.py
klipper_path="${KLIPPER_PATH:-$HOME/klipper}"
moonraker_conf="${MOONRAKER_CONFIG:-$HOME/printer_data/config/moonraker.conf}"
extras="$klipper_path/klippy/extras"
exclude_file="$klipper_path/.git/info/exclude"
exclude_entry="klippy/extras/$module_name"

if [ "$(id -u)" = "0" ]; then
    echo "Run as the Klipper user, not root."
    exit 1
fi
if [ ! -d "$extras" ]; then
    echo "Klipper extras not found at $extras (set KLIPPER_PATH)."
    exit 1
fi

target="$extras/$module_name"
if [ "$1" = "-u" ]; then
    if [ -L "$target" ]; then
        rm "$target"
    elif [ -e "$target" ]; then
        echo "$target is not a link made by this script. Left it in place."
        exit 1
    fi
    echo "Removed the link. Remove [$repo] and the beacon_accel endstop pins from your config, then restart Klipper."
    exit 0
fi

# Beacon links itself into extras on Klipper and Kalico. Kalico also loads plugins/.
if [ ! -f "$extras/beacon.py" ] && [ ! -f "$klipper_path/klippy/plugins/beacon.py" ]; then
    echo "Beacon module not found. Install beacon_klipper first."
    exit 1
fi

if [ -e "$target" ] && [ ! -L "$target" ]; then
    echo "$target already exists and is not a link. Move it away first."
    exit 1
fi
ln -sfn "$repo_path/$module_name" "$target"
if [ -f "$exclude_file" ] && ! grep -qxF "$exclude_entry" "$exclude_file"; then
    echo "$exclude_entry" >> "$exclude_file"
fi
echo "Linked $module_name into $extras."

# Moonraker update_manager entry, only for a git clone with a remote. Moonraker
# updates a whole repository, so the entry points at the root of the clone
# (the MISC repository), not at this folder.
git_root="$(git -C "$repo_path" rev-parse --show-toplevel 2>/dev/null || true)"
origin="$(git -C "$repo_path" remote get-url origin 2>/dev/null || true)"
branch="$(git -C "$repo_path" rev-parse --abbrev-ref HEAD 2>/dev/null || true)"
if [ -f "$moonraker_conf" ] && [ -n "$git_root" ] && [ -n "$origin" ] && [ -n "$branch" ] && [ -t 0 ]; then
    if grep -q "^\[update_manager $repo\]" "$moonraker_conf"; then
        echo "[update_manager $repo] already exists in $moonraker_conf."
    else
        read -r -p "Add [update_manager $repo] to $moonraker_conf? (y/n): " answer
        if [ "$answer" != "${answer#[Yy]}" ]; then
            printf '\n[update_manager %s]\ntype: git_repo\npath: %s\norigin: %s\nprimary_branch: %s\nmanaged_services: klipper\n' \
                "$repo" "$git_root" "$origin" "$branch" >> "$moonraker_conf"
            echo "Added. Restart Moonraker to load it."
        fi
    fi
fi

echo "Next: add [$repo] to your config, set endstop_pin: beacon_accel:x / :y, restart Klipper."
