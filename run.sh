#! /bin/bash

toilet "ALUSTAB!!!"

dts devel build -f -H bestestduckiebot.local
dts devel run -H bestestduckiebot.local -- --privileged

toilet "LÄBI!!!"
