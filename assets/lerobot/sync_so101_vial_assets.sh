#!/usr/bin/env bash
# Download SO101 vial-task assets from NVIDIA Sim-to-Real workshop into this repo.
# No local workshop clone required — fetches real files from GitHub.
#
# Usage:
#   ./assets/lerobot/sync_so101_vial_assets.sh
#
# After copy, commit through Git LFS (see assets/lerobot/README.md).

set -euo pipefail

REPO="isaac-sim/Sim-to-Real-SO-101-Workshop"
BRANCH="main"
BASE="https://media.githubusercontent.com/media/${REPO}/${BRANCH}/source/sim_to_real_so101/assets/usd"
ROOT="$(cd "$(dirname "$0")" && pwd)"
DEST="${ROOT}/so101_vial_task/usd"
TEX_DEST="${DEST}/tex"

USD_FILES=(
  lightbox-simple.usd
  mat.usda
  Vial_opaque.usda
  Vial_rack_simple.usda
)

TEX_FILES=(
  vial_diff.png
  vial_opacity.png
  vial_spec.png
)

mkdir -p "$DEST" "$TEX_DEST"

fetch() {
  local url="$1"
  local out="$2"
  echo "Fetching $(basename "$out")..."
  curl -fsSL "$url" -o "$out"
  if head -1 "$out" | grep -q 'git-lfs.github.com'; then
    echo "ERROR: got LFS pointer instead of asset: $out" >&2
    exit 1
  fi
}

for name in "${USD_FILES[@]}"; do
  fetch "${BASE}/${name}" "${DEST}/${name}"
done

for name in "${TEX_FILES[@]}"; do
  fetch "${BASE}/tex/${name}" "${TEX_DEST}/${name}"
done

echo "OK: vial-task assets in ${DEST}"
echo "Next:"
echo "  export PATH=\"\$HOME/.local/bin:\$PATH\"   # if git-lfs installed locally"
echo "  git lfs install"
echo "  git add assets/lerobot/so101_vial_task/"
