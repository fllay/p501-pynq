#!/bin/bash
# Runs chrooted (via qemu-aarch64-static) with real network access at build
# time -- the board itself has no internet once deployed (only a
# point-to-point USB link to a host PC), so this must be baked in now.
set -x
set -e

python3 -m pip install spidev
