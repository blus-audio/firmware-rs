#!/bin/bash
set -euo pipefail

for dir in blus-mini-mk1 blus-mini-mk2 blackpill-usb-dac/v1.2 blackpill-usb-dac/v3.1;
do
    pushd $dir
    cargo fmt
    cargo clippy -- -D warnings
    cargo build --release
    popd
done
