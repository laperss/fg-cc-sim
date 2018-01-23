#!/bin/sh

AUTOTESTDIR="/home/laperss/Programs/jsbsim"

nice fgfs \
     --fdm=jsb \
     --fg-aircraft="$AUTOTESTDIR/aircraft" \
     --units-meters \
     --timeofday=noon \
     --geometry=650x550 \
     --shading-flat \
     --bpp=32 \
     --prop:/sim/rendering/multi-sample-buffers=true\
     --prop:/sim/rendering/multi-samples=4\
     --prop:/environment/params/jsbsim-turbulence-model=ttCulp\
     --timeofday=noon \
     --disable-anti-alias-hud \
     --disable-hud-3d \
     --disable-sound \
     --disable-fullscreen \
     --disable-random-objects \
     --disable-anti-alias-hud \
     --wind=0.0@0 \
     $*
