#!/bin/bash

cd ~/.local/share/Steam/steamapps/common/SteamVR/bin/linux64/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH://home/robot/.local/share/Steam/steamapps/common/SteamVR/bin/linux64
/home/robot/.local/share/Steam/steamapps/common/SteamVR/bin/linux64//vrserver  --keepalive

