#!/usr/bin/env bash
# Build the workspace and produce a clangd/VSCode-friendly compile_commands.json at the root.
# Needed because colcon may emit one compile_commands.json per package; this merges them when required.
# Run with: ./scripts/make_compile_commands.sh

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd "${script_dir}/.." && pwd)"
build_dir="${workspace_root}/build"
output_link="${workspace_root}/compile_commands.json"

cd "$workspace_root"

echo "Running colcon build with compile command export..."
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

primary_compile="${build_dir}/compile_commands.json"

if [[ -f "$primary_compile" ]]; then
    final_source="$primary_compile"
else
    mapfile -t compile_files < <(find "$build_dir" -name compile_commands.json -type f | sort)

    if [[ ${#compile_files[@]} -eq 0 ]]; then
        echo "No compile_commands.json files found under ${build_dir}. Nothing to link." >&2
        exit 1
    elif [[ ${#compile_files[@]} -eq 1 ]]; then
        final_source="${compile_files[0]}"
    else
        if ! command -v jq >/dev/null 2>&1; then
            echo "jq is required to merge multiple compile_commands.json files but was not found. Please install jq and rerun." >&2
            exit 1
        fi

        merged_file="${build_dir}/compile_commands.merged.json"
        echo "Merging ${#compile_files[@]} compile_commands.json files into ${merged_file}"
        jq -s 'add' "${compile_files[@]}" > "$merged_file"
        final_source="$merged_file"
    fi
fi

echo "Linking ${final_source} -> ${output_link}"
ln -sfn "$final_source" "$output_link"
echo "compile_commands.json ready at workspace root"
