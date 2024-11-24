#!/bin/bash
set -euo pipefail

DIRS=("blus_mini_mk1" "blus_mini_mk2")

for dir in $DIRS;
do
    cargo fmt
    cargo clippy
    cargo build --verbose
    cargo test --verbose
done
