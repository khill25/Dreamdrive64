# PicoCart64 v2.2

:exclamation: Latest 2.2 board revisions have been ordered for testing, so build at your own risk :exclamation:

## Errata

### v2.0
- Less than ideal VBus' routing on Inner Layer 1. The power routing should work and passes DRC, but could be better.


## Changelog

### v2.2
- Improved routing
- Removed mcu1 USB-C port
- CS line of all chips in PSRAM array are routed through the demux
- Removed play area
- Added reset button pads connected to mcu2 reset pins for easy reset

### v2.1-unreleased
- Improved power routing for VBus'

