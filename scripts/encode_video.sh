#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
ffmpeg -y -loglevel error -framerate 30 -i outputs/plastic/render/%04d.png \
 -vf "drawtext=text='Newton VBD - UR10 cardboard plasticity':x=28:y=24:fontsize=28:fontcolor=white:box=1:boxcolor=black@0.60,drawtext=text='SQUEEZE - 30 N motor limit per finger':x=28:y=65:fontsize=23:fontcolor=white:box=1:boxcolor=black@0.60:enable='between(t,1,4)',drawtext=text='TWIST - contact torque feedback':x=28:y=65:fontsize=23:fontcolor=white:box=1:boxcolor=black@0.60:enable='between(t,4,6)',drawtext=text='RELEASE':x=28:y=65:fontsize=23:fontcolor=white:box=1:boxcolor=black@0.60:enable='between(t,6,7)',drawtext=text='UNLOADED - permanent crease state retained':x=28:y=65:fontsize=23:fontcolor=white:box=1:boxcolor=black@0.60:enable='gte(t,7)',drawtext=text='Physics simulation - illustrative material parameters - Isaac Sim RTX render':x=28:y=h-44:fontsize=18:fontcolor=white:box=1:boxcolor=black@0.60" \
 -c:v libx264 -crf 19 -pix_fmt yuv420p -movflags +faststart outputs/plastic/cardboard_crush_demo.mp4
