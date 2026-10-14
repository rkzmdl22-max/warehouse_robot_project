#!/bin/bash

export USD_PATH=/usd2sdf_ws/install

export GZ_USD_PATH=/usd2sdf_ws/install/gz-usd

export PATH="$GZ_USD_PATH/bin:$USD_PATH/bin:$PATH"

export LD_LIBRARY_PATH="$GZ_USD_PATH/lib:$USD_PATH/lib:${LD_LIBRARY_PATH:-}"

export CMAKE_PREFIX_PATH="$GZ_USD_PATH:$USD_PATH:${CMAKE_PREFIX_PATH:-}"

export GZ_VERSION=harmonic
