#!/bin/bash
JLinkExe -device RP2040_M0_0 -if SWD -speed 4000 <<EOF
loadbin $1, 0x10000000
r
g
exit
EOF
