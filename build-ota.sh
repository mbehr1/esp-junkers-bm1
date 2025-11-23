#!/bin/bash
set -e

proj_name=$(basename "$PWD")

cargo build --release
build_path=$(cargo build --release --message-format=json | jq -r ".executable | select(type==\"string\")")
echo "Build path: $build_path"
espflash save-image --chip esp32c6 -s 8mb \
  $build_path \
  $proj_name.ota

echo "✅ OTA image generated: $proj_name.ota. Flash with 'cat $proj_name.ota|nc <device ip> 65456'"
