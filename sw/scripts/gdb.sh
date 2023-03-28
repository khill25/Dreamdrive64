#!/bin/bash
set -e

OPENOCD=${OPENOCD:-$(which openocd || true)}
PREFIX=${PREFIX:-"arm-none-eabi-"}

# Hacky but apparently reliable way to get the absolute path of the script directory
SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
BUILDPATH=${BUILDPATH:-$SCRIPTPATH/../build}
ELF="$BUILDPATH/dreamdrive64/dreamdrive64.elf"
GDB=${PREFIX}gdb

$GDB $ELF -ex 'target extended-remote :3333'
