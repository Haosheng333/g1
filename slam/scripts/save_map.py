#!/usr/bin/env python3
"""Copy FAST-LIO2's accumulated map into slam/maps/ under a chosen name.

FAST-LIO2 (external package, not modified by this script) writes its
accumulated point cloud to a fixed path inside its own source tree,
<fast_lio pkg>/PCD/scans.pcd, once per run (see fastlio.yaml's
pcd_save.pcd_save_en / pcd_save.interval). That file is overwritten on
every run, so this script copies it into a permanent, named location
under g1_slam/maps/ instead, and refuses to overwrite an existing map.
"""
import argparse
import os
import re
import shutil
import sys

NAME_RE = re.compile(r"^[A-Za-z0-9_-]+$")


def resolve_package_path(package_name):
    import rospkg
    return rospkg.RosPack().get_path(package_name)


def default_source_path():
    return os.path.join(resolve_package_path("fast_lio"), "PCD", "scans.pcd")


def default_maps_dir():
    return os.path.join(resolve_package_path("g1_slam"), "maps")


def is_valid_pcd(path):
    try:
        with open(path, "rb") as f:
            header = f.read(11)
    except OSError:
        return False
    return header.startswith(b"# .PCD v")


def main():
    parser = argparse.ArgumentParser(
        description="Save FAST-LIO2's accumulated map into slam/maps/ under a chosen name.")
    parser.add_argument("--name", required=True,
                         help="destination map name, no extension/path (e.g. 'lab_room')")
    parser.add_argument("--source", default=None,
                         help="override the FAST-LIO2 PCD source path "
                              "(default: <fast_lio pkg>/PCD/scans.pcd)")
    parser.add_argument("--maps-dir", default=None,
                         help="override the destination maps directory "
                              "(default: <g1_slam pkg>/maps)")
    parser.add_argument("--dry-run", action="store_true",
                         help="validate name/source/destination without copying")
    args = parser.parse_args()

    if not NAME_RE.match(args.name):
        print("error: --name must match {} (got: {!r})".format(NAME_RE.pattern, args.name),
              file=sys.stderr)
        return 1

    try:
        source_path = args.source or default_source_path()
    except Exception as exc:
        print("error: could not resolve fast_lio package: {}".format(exc), file=sys.stderr)
        return 1

    try:
        maps_dir = args.maps_dir or default_maps_dir()
    except Exception as exc:
        print("error: could not resolve g1_slam package: {}".format(exc), file=sys.stderr)
        return 1

    dest_path = os.path.join(maps_dir, args.name + ".pcd")

    if not os.path.isfile(source_path):
        print("error: no accumulated map found at {} "
              "(run fastlio_mapping.launch with pcd_save_en:true first, "
              "then let it shut down cleanly)".format(source_path), file=sys.stderr)
        return 1

    if os.path.getsize(source_path) == 0:
        print("error: source map {} is empty".format(source_path), file=sys.stderr)
        return 1

    if not is_valid_pcd(source_path):
        print("error: source map {} does not look like a valid PCD file".format(source_path),
              file=sys.stderr)
        return 1

    if os.path.exists(dest_path):
        print("error: refusing to overwrite existing map {} "
              "(choose a different --name)".format(dest_path), file=sys.stderr)
        return 1

    if args.dry_run:
        print("[dry-run] would copy {} -> {} ({} bytes)".format(
            source_path, dest_path, os.path.getsize(source_path)))
        return 0

    if not os.path.isdir(maps_dir):
        print("error: maps directory does not exist: {}".format(maps_dir), file=sys.stderr)
        return 1

    shutil.copy2(source_path, dest_path)
    print("saved map: {} ({} bytes)".format(dest_path, os.path.getsize(dest_path)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
