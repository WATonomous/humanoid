#!/usr/bin/env bash
# Download SO101 vial-task assets from NVIDIA Sim-to-Real workshop into this repo.
# No local workshop clone required — fetches real files from GitHub.
#
# Usage:
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> ce24efc2 (add-to-IL-pipeline-and-add-16dof-hand)
#   ./assets/lerobot/sync_so101_vial_assets.sh          # vial props + textures
#   ./assets/lerobot/sync_so101_vial_assets.sh --full    # + arm_camera, tray, HDRI (for --domain_rand)
#   ./assets/lerobot/sync_so101_vial_assets.sh --hdri   # HDRI only
#   ./assets/lerobot/sync_so101_vial_assets.sh --arm    # so101_arm_camera.usd only
<<<<<<< HEAD
#
# After copy, commit the downloaded assets normally (see assets/lerobot/README.md).
=======
#   ./assets/lerobot/sync_so101_vial_assets.sh
=======
>>>>>>> ce24efc2 (add-to-IL-pipeline-and-add-16dof-hand)
#
<<<<<<< HEAD
# After copy, commit through Git LFS (see assets/lerobot/README.md).
>>>>>>> 476bbbcc (Add SO101 vial-task assets under Git LFS.)
=======
# After copy, commit the downloaded assets normally (see assets/lerobot/README.md).
>>>>>>> 43e7a4c2 (fix lfs issue)

set -euo pipefail

REPO="isaac-sim/Sim-to-Real-SO-101-Workshop"
BRANCH="main"
<<<<<<< HEAD
<<<<<<< HEAD
USD_BASE="https://media.githubusercontent.com/media/${REPO}/${BRANCH}/source/sim_to_real_so101/assets/usd"
HDRI_BASE="https://github.com/${REPO}/raw/${BRANCH}/source/sim_to_real_so101/assets/hdri"
ROOT="$(cd "$(dirname "$0")" && pwd)"
DEST="${ROOT}/so101_vial_task/usd"
TEX_DEST="${DEST}/tex"
SO101_DEST="${ROOT}/so101"
HDRI_DEST="${ROOT}/so101_vial_task/hdri"
=======
BASE="https://media.githubusercontent.com/media/${REPO}/${BRANCH}/source/sim_to_real_so101/assets/usd"
ROOT="$(cd "$(dirname "$0")" && pwd)"
DEST="${ROOT}/so101_vial_task/usd"
TEX_DEST="${DEST}/tex"
>>>>>>> 476bbbcc (Add SO101 vial-task assets under Git LFS.)
=======
USD_BASE="https://media.githubusercontent.com/media/${REPO}/${BRANCH}/source/sim_to_real_so101/assets/usd"
HDRI_BASE="https://media.githubusercontent.com/media/${REPO}/${BRANCH}/source/sim_to_real_so101/assets/hdri"
ROOT="$(cd "$(dirname "$0")" && pwd)"
DEST="${ROOT}/so101_vial_task/usd"
TEX_DEST="${DEST}/tex"
SO101_DEST="${ROOT}/so101"
HDRI_DEST="${ROOT}/so101_vial_task/hdri"
>>>>>>> ce24efc2 (add-to-IL-pipeline-and-add-16dof-hand)

USD_FILES=(
  lightbox-simple.usd
  mat.usda
  Vial_opaque.usda
  Vial_rack_simple.usda
<<<<<<< HEAD
<<<<<<< HEAD
  tray.usda
=======
>>>>>>> 476bbbcc (Add SO101 vial-task assets under Git LFS.)
=======
  tray.usda
>>>>>>> ce24efc2 (add-to-IL-pipeline-and-add-16dof-hand)
)

TEX_FILES=(
  vial_diff.png
  vial_opacity.png
  vial_spec.png
)

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> ce24efc2 (add-to-IL-pipeline-and-add-16dof-hand)
HDRI_FILES=(
  yaw_mapping.yaml
  moon_lab_1k.exr
  abandoned_garage_1k.exr
  afrikaans_church_interior_1k.exr
  blaubeuren_church_square_1k.exr
  blaubeuren_night_1k.exr
  blue_photo_studio_1k.exr
  boma_1k.exr
  brown_photostudio_01_1k.exr
  burnt_warehouse_1k.exr
  climbing_gym_1k.exr
  creepy_bathroom_1k.exr
  fireplace_1k.exr
  hospital_room_2_1k.exr
  metro_noord_1k.exr
  monkstown_castle_1k.exr
  moonlit_golf_1k.exr
  outdoor_chapel_1k.exr
  poly_haven_studio_1k.exr
  rogland_moonlit_night_1k.exr
  satara_night_1k.exr
  studio_small_03_1k.exr
  studio_small_09_1k.exr
  university_workshop_1k.exr
  winter_evening_1k.exr
)
<<<<<<< HEAD
=======
mkdir -p "$DEST" "$TEX_DEST"
>>>>>>> 476bbbcc (Add SO101 vial-task assets under Git LFS.)
=======
>>>>>>> ce24efc2 (add-to-IL-pipeline-and-add-16dof-hand)

fetch() {
  local url="$1"
  local out="$2"
<<<<<<< HEAD
<<<<<<< HEAD
  mkdir -p "$(dirname "$out")"
=======
>>>>>>> 476bbbcc (Add SO101 vial-task assets under Git LFS.)
=======
  mkdir -p "$(dirname "$out")"
>>>>>>> ce24efc2 (add-to-IL-pipeline-and-add-16dof-hand)
  echo "Fetching $(basename "$out")..."
  curl -fsSL "$url" -o "$out"
  if head -1 "$out" | grep -q 'git-lfs.github.com'; then
    echo "ERROR: got LFS pointer instead of asset: $out" >&2
    exit 1
  fi
}

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> ce24efc2 (add-to-IL-pipeline-and-add-16dof-hand)
sync_vial_props() {
  mkdir -p "$DEST" "$TEX_DEST"
  for name in lightbox-simple.usd mat.usda Vial_opaque.usda Vial_rack_simple.usda tray.usda; do
    fetch "${USD_BASE}/${name}" "${DEST}/${name}"
  done
  for name in "${TEX_FILES[@]}"; do
    fetch "${USD_BASE}/tex/${name}" "${TEX_DEST}/${name}"
  done
  echo "OK: vial props in ${DEST}"
}
<<<<<<< HEAD

sync_tray() {
  fetch "${USD_BASE}/tray.usda" "${DEST}/tray.usda"
  echo "OK: tray in ${DEST}/tray.usda"
}

sync_arm_camera() {
  mkdir -p "$SO101_DEST"
  fetch "${USD_BASE}/SO-ARM101-USD.usd" "${SO101_DEST}/so101_arm_camera.usd"
  echo "OK: workshop arm (embedded gripper cam mesh) -> ${SO101_DEST}/so101_arm_camera.usd"
}

sync_hdri() {
  mkdir -p "$HDRI_DEST"
  for name in "${HDRI_FILES[@]}"; do
    fetch "${HDRI_BASE}/${name}" "${HDRI_DEST}/${name}"
  done
  echo "OK: HDRI pack (${#HDRI_FILES[@]} files) in ${HDRI_DEST}"
}

case "${1:-}" in
  --full)
    sync_vial_props
    sync_tray
    sync_arm_camera
    sync_hdri
    ;;
  --hdri)
    sync_hdri
    ;;
  --arm)
    sync_arm_camera
    ;;
  --tray)
    sync_tray
    ;;
  "")
    sync_vial_props
    ;;
  *)
    echo "Unknown option: $1" >&2
    echo "Usage: $0 [--full|--hdri|--arm|--tray]" >&2
    exit 1
    ;;
esac

echo "Next:"
echo "  git add assets/lerobot/"
=======
for name in "${USD_FILES[@]}"; do
  fetch "${BASE}/${name}" "${DEST}/${name}"
done
=======
>>>>>>> ce24efc2 (add-to-IL-pipeline-and-add-16dof-hand)

sync_tray() {
  fetch "${USD_BASE}/tray.usda" "${DEST}/tray.usda"
  echo "OK: tray in ${DEST}/tray.usda"
}

sync_arm_camera() {
  mkdir -p "$SO101_DEST"
  fetch "${USD_BASE}/SO-ARM101-USD.usd" "${SO101_DEST}/so101_arm_camera.usd"
  echo "OK: workshop arm (embedded gripper cam mesh) -> ${SO101_DEST}/so101_arm_camera.usd"
}

sync_hdri() {
  mkdir -p "$HDRI_DEST"
  for name in "${HDRI_FILES[@]}"; do
    fetch "${HDRI_BASE}/${name}" "${HDRI_DEST}/${name}"
  done
  echo "OK: HDRI pack (${#HDRI_FILES[@]} files) in ${HDRI_DEST}"
}

case "${1:-}" in
  --full)
    sync_vial_props
    sync_tray
    sync_arm_camera
    sync_hdri
    ;;
  --hdri)
    sync_hdri
    ;;
  --arm)
    sync_arm_camera
    ;;
  --tray)
    sync_tray
    ;;
  "")
    sync_vial_props
    ;;
  *)
    echo "Unknown option: $1" >&2
    echo "Usage: $0 [--full|--hdri|--arm|--tray]" >&2
    exit 1
    ;;
esac

echo "Next:"
<<<<<<< HEAD
echo "  export PATH=\"\$HOME/.local/bin:\$PATH\"   # if git-lfs installed locally"
echo "  git lfs install"
<<<<<<< HEAD
echo "  git add assets/lerobot/so101_vial_task/"
>>>>>>> 476bbbcc (Add SO101 vial-task assets under Git LFS.)
=======
=======
>>>>>>> 43e7a4c2 (fix lfs issue)
echo "  git add assets/lerobot/"
>>>>>>> ce24efc2 (add-to-IL-pipeline-and-add-16dof-hand)
