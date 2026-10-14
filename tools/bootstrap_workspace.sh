#!/usr/bin/env bash
# Resolve this repo's two external catkin dependencies on THIS machine without
# editing any tracked file, and verify the result is unambiguous.
#
#   livox_ros_driver2   Livox Mid-360 ROS1 driver (g1_slam build + sensors.launch)
#   fast_lio            hku-mars FAST-LIO2        (g1_slam build)
#
# WHY THIS EXISTS. The repo is developed on machines with different layouts, and
# g1_ws/src/livox_ros_driver2 is a TRACKED symlink encoding one of them
# (../../../g1_intellect/external/livox_ros_driver2). On any machine where that
# path does not exist the symlink dangles, so `roslaunch g1_slam sensors.launch`
# fails with "Resource not found: livox_ros_driver2". Repointing the tracked
# symlink would fix this machine and break the other one, so this script never
# touches it: it discovers whatever is installed locally and, if needed, adds an
# UNTRACKED overlay symlink beside it.
#
# CONFIGURATION, highest priority first:
#   1. environment:  G1_LIVOX_DIR=/path  G1_FAST_LIO_DIR=/path
#   2. g1_ws/local.conf        (untracked, per-machine; see local.conf.example)
#   3. `rospack find` in the currently sourced environment
#   4. documented candidates RELATIVE to this repo (parent catkin src/, siblings)
# No absolute personal path is ever hard-coded here or written to a tracked file.
#
#   tools/bootstrap_workspace.sh            # --check (default): report only
#   tools/bootstrap_workspace.sh --apply    # create untracked overlays/CATKIN_IGNORE
#
# This script NEVER installs a package, never rebuilds a container, never runs
# catkin, and never writes a tracked file. --apply only creates untracked
# symlinks and CATKIN_IGNORE marker files.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
G1_WS="$REPO/g1_ws"
LOCAL_CONF="$G1_WS/local.conf"
MODE="check"
RC=0

case "${1:-}" in
  --apply) MODE="apply" ;;
  --check|"") MODE="check" ;;
  -h|--help) sed -n '2,32p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
  *) echo "unknown argument: $1 (expected --check or --apply)" >&2; exit 2 ;;
esac

say()  { printf '%s\n' "$*"; }
ok()   { printf '  [OK]    %s\n' "$*"; }
warn() { printf '  [WARN]  %s\n' "$*"; }
fail() { printf '  [FAIL]  %s\n' "$*"; RC=1; }
info() { printf '  [INFO]  %s\n' "$*"; }
todo() { printf '  [TODO]  %s\n' "$*"; }

# ---------------------------------------------------------------- local.conf
# Only KEY=VALUE lines for the two keys below are honoured; nothing is sourced
# as shell, so a stray command in local.conf cannot execute.
conf_get() {
  local key="$1"
  [ -f "$LOCAL_CONF" ] || return 0
  sed -nE "s/^[[:space:]]*${key}[[:space:]]*=[[:space:]]*//p" "$LOCAL_CONF" \
    | tail -1 | sed -E 's/[[:space:]]+$//; s/^"(.*)"$/\1/; s/^'"'"'(.*)'"'"'$/\1/'
}

is_pkg() { [ -f "$1/package.xml" ]; }

pkg_name_at() {
  [ -f "$1/package.xml" ] || return 1
  sed -nE 's;.*<name>[[:space:]]*([^<[:space:]]+).*;\1;p' "$1/package.xml" | head -1
}

# discover <pkgname> <env var name> <conf key> <candidate...>
discover() {
  local pkg="$1" envvar="$2" confkey="$3"; shift 3
  local cand

  cand="${!envvar:-}"
  if [ -n "$cand" ]; then
    if is_pkg "$cand"; then printf '%s\t%s\n' "$cand" "\$$envvar"; return 0; fi
    printf '\t%s\n' "INVALID:\$$envvar=$cand has no package.xml"; return 1
  fi

  cand="$(conf_get "$confkey")"
  if [ -n "$cand" ]; then
    case "$cand" in /*) : ;; *) cand="$G1_WS/$cand" ;; esac
    if is_pkg "$cand"; then printf '%s\t%s\n' "$cand" "local.conf:$confkey"; return 0; fi
    printf '\t%s\n' "INVALID:local.conf $confkey=$cand has no package.xml"; return 1
  fi

  if command -v rospack >/dev/null 2>&1; then
    cand="$(rospack find "$pkg" 2>/dev/null | head -1)"
    if [ -n "$cand" ] && is_pkg "$cand"; then
      printf '%s\t%s\n' "$cand" "rospack find"; return 0
    fi
  fi

  for cand in "$@"; do
    if is_pkg "$cand"; then printf '%s\t%s\n' "$cand" "repo-relative candidate"; return 0; fi
  done
  printf '\t%s\n' "NOTFOUND"; return 1
}

# ------------------------------------------------------------------- layout
say "== layout =="
PARENT_SRC="$(cd "$REPO/.." && pwd)"
OUTER=0
if [ -f "$PARENT_SRC/CMakeLists.txt" ] || [ -d "$PARENT_SRC/../devel" ]; then
  OUTER=1
  info "repo sits inside a parent catkin source space: $PARENT_SRC"
  info "that parent is the build workspace on this machine (packages resolve from g1/* directly)"
else
  info "no parent catkin source space detected; the nested g1_ws is the build workspace"
fi

# --------------------------------------------------------------- dependencies
say ""
say "== external dependencies =="
declare -A RESOLVED=()
for spec in \
  "livox_ros_driver2:G1_LIVOX_DIR:livox_ros_driver2_dir" \
  "fast_lio:G1_FAST_LIO_DIR:fast_lio_dir"
do
  pkg="${spec%%:*}"; rest="${spec#*:}"; envvar="${rest%%:*}"; confkey="${rest#*:}"
  out="$(discover "$pkg" "$envvar" "$confkey" \
        "$PARENT_SRC/$pkg" "$PARENT_SRC/FAST_LIO" "$PARENT_SRC/${pkg^^}" \
        "$G1_WS/src/$pkg")"
  path="$(printf '%s' "$out" | cut -f1)"
  how="$(printf '%s' "$out" | cut -f2)"
  if [ -n "$path" ]; then
    RESOLVED[$pkg]="$path"
    ok "$pkg -> $path  (via $how)"
  elif [ "$how" = "NOTFOUND" ]; then
    fail "$pkg not found. Set $envvar=/path, or add '$confkey = /path' to g1_ws/local.conf"
    todo "see g1_ws/local.conf.example and g1_ws/README.md"
  else
    fail "$pkg: ${how#INVALID:}"
  fi
done

# ------------------------------------------------- nested-workspace overlays
say ""
say "== nested g1_ws source space =="
for pkg in livox_ros_driver2 fast_lio; do
  tracked="$G1_WS/src/$pkg"
  overlay="$G1_WS/src/local_$pkg"
  real="${RESOLVED[$pkg]:-}"

  tracked_ok=0
  [ -e "$tracked" ] && is_pkg "$tracked" && tracked_ok=1

  if [ "$tracked_ok" = 1 ]; then
    ok "src/$pkg resolves (this machine matches the tracked layout)"
    if [ -L "$overlay" ]; then
      fail "src/local_$pkg ALSO exists -> catkin would find two '$pkg' packages. Remove the overlay."
    fi
    continue
  fi

  if [ -L "$tracked" ]; then
    info "src/$pkg is a tracked symlink that dangles here (-> $(readlink "$tracked")); left untouched"
  fi

  if [ -z "$real" ]; then
    warn "no overlay possible for $pkg until it is discovered (above)"
    continue
  fi

  # RELATIVE target: this tree is bind-mounted at a different absolute path
  # inside the container (see CLAUDE.md), so an absolute symlink would resolve
  # on only one side of the mount. Same reason the tracked links are relative.
  rel="$(realpath --relative-to="$G1_WS/src" "$real" 2>/dev/null || printf '%s' "$real")"

  if [ -L "$overlay" ] && [ "$(readlink -f "$overlay")" = "$(readlink -f "$real")" ]; then
    ok "overlay src/local_$pkg -> $(readlink "$overlay") already in place"
  elif [ "$MODE" = "apply" ]; then
    ln -sfn "$rel" "$overlay" && ok "created untracked overlay src/local_$pkg -> $rel"
  else
    todo "would create untracked overlay src/local_$pkg -> $rel   (run with --apply)"
  fi
done

# -------------------------------------------- CATKIN_IGNORE for outer layout
say ""
say "== duplicate-package guard =="
if [ "$OUTER" = 1 ]; then
  marker="$G1_WS/CATKIN_IGNORE"
  if [ -f "$marker" ]; then
    ok "g1_ws/CATKIN_IGNORE present: the parent workspace will not descend into g1_ws"
  elif [ "$MODE" = "apply" ]; then
    cat > "$marker" <<'EOT'
Keeps the PARENT catkin workspace from descending into this nested workspace.

Without this file every package that g1_ws/src symlinks to is discovered twice
(once at g1/<dir>, once through g1_ws/src/<link>) and roslaunch/catkin abort with
"Multiple packages found with the same name".

Untracked on purpose: it is correct only for a machine whose build workspace is
the PARENT source space. Created by tools/bootstrap_workspace.sh --apply.
EOT
    ok "created untracked g1_ws/CATKIN_IGNORE"
  else
    fail "g1_ws/CATKIN_IGNORE missing: the parent workspace would find duplicate packages"
    todo "run with --apply to create it (untracked)"
  fi
  info "add it to .git/info/exclude so it never shows up in git status:"
  info "  grep -qx /g1_ws/CATKIN_IGNORE .git/info/exclude || echo /g1_ws/CATKIN_IGNORE >> .git/info/exclude"
else
  info "nested layout: no CATKIN_IGNORE needed"
fi

# ------------------------------------------------ exactly one package per name
say ""
say "== exactly one discovered package per name =="
SEARCH_ROOT="$PARENT_SRC"; [ "$OUTER" = 1 ] || SEARCH_ROOT="$G1_WS/src"
info "scanning $SEARCH_ROOT the way catkin does (honouring CATKIN_IGNORE)"
python3 - "$SEARCH_ROOT" <<'PY'
import os, re, sys, collections
root = sys.argv[1]
found = collections.defaultdict(list)
for dirpath, dirnames, filenames in os.walk(root, followlinks=True):
    if "CATKIN_IGNORE" in filenames or "AMENT_IGNORE" in filenames:
        dirnames[:] = []
        continue
    if "package.xml" in filenames:
        try:
            txt = open(os.path.join(dirpath, "package.xml"), errors="ignore").read()
        except OSError:
            continue
        m = re.search(r"<name>\s*([^<\s]+)\s*</name>", txt)
        if m:
            found[m.group(1)].append(dirpath)
dups = {k: v for k, v in found.items() if len(v) > 1}
print("  [INFO]  %d package names discovered" % len(found))
if dups:
    print("  [FAIL]  %d duplicated:" % len(dups))
    for k, v in sorted(dups.items()):
        print("            %s" % k)
        for p in v:
            print("              %s" % p)
    sys.exit(1)
print("  [OK]    exactly one discovered package per package name")
PY
[ $? -ne 0 ] && RC=1

say ""
if [ "$RC" -eq 0 ]; then
  say "RESULT: PASS"
else
  say "RESULT: FAIL (see [FAIL] lines above)"
fi
say ""
say "Launch resolution only proves roslaunch can FIND the packages."
say "It is not evidence that the LiDAR, camera, hand or robot will start."
exit "$RC"
