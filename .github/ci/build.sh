#!/bin/bash
set -euo pipefail

for dir in blus_mini_mk1 blus_mini_mk2 blackpill_pcm5102a;
do
    pushd $dir
    cargo fmt
    cargo clippy
    cargo build --release
    popd
done
