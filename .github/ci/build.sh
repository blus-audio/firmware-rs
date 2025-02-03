#!/bin/bash
set -euo pipefail

for dir in blus-mini-mk1 blus-mini-mk2 blackpill-pcm5102a;
do
    pushd $dir
    cargo fmt
    cargo clippy -- -D warnings
    cargo build --release
    popd
done
