#! /bin/bash

toilet "ALUSTAB!!!"

dts devel build -f -H tera.local
dts devel run -H tera.local -- --privileged

toilet "LÄBI!!!"
