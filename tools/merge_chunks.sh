#!/usr/bin/env bash
# Merge the chunk point clouds of a lidar_odometry_step_1 result folder into one LAZ in world coordinates.
# Each scan_lio_N.laz is stored relative to its chunk start pose; session_poses.mrp holds those poses.
# Usage: tools/merge_chunks.sh <result_folder> [output.laz]
set -euo pipefail
CC="${CLOUDCOMPARE:-/c/Program Files/CloudCompare/CloudCompare.exe}"
dir="$1"
out="${2:-$dir/scan_lio_merged.laz}"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

# split session_poses.mrp (count, then per chunk: file name + 4 matrix rows) into one matrix file per chunk
awk -v t="$tmp" 'NR>1 { if ((NR-2)%5==0) { f=t "/" $1 ".txt"; next } print > f }' "$dir/session_poses.mrp"

args=()
for m in "$tmp"/scan_lio_*.laz.txt; do
    name="$(basename "$m" .txt)"
    base="${name%.laz}"
    # transform one chunk into world coordinates
    "$CC" -SILENT -AUTO_SAVE OFF -O "$dir/$name" -APPLY_TRANS "$m" \
        -C_EXPORT_FMT LAS -EXT laz -SAVE_CLOUDS FILE "\"$tmp/$base.laz\"" > /dev/null
    args+=(-O "$tmp/$base.laz")
done

"$CC" -SILENT -AUTO_SAVE OFF "${args[@]}" -MERGE_CLOUDS \
    -C_EXPORT_FMT LAS -EXT laz -SAVE_CLOUDS FILE "\"$out\"" > /dev/null
echo "written: $out"
