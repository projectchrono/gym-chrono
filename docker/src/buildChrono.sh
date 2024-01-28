#!/bin/bash
mkdir -p $HOME/Desktop
cd $HOME/Desktop
git clone --branch feature/robot_model https://github.com/zzhou292/chrono.git
cd chrono
git submodule init
git submodule update
bash contrib/build-scripts/opengl/buildGL.sh
bash contrib/build-scripts/urdf/buildURDF.sh
# ------------------------------------------------------------------------

SOURCE_DIR="$HOME/Desktop/chrono"
BUILD_DIR="$HOME/Desktop/chrono/build"
INSTALL_DIR="$HOME/Desktop/chrono/install"

OPTIX_INCLUDE_DIR="/opt/optix/include"
URDFDOM_DIR="$HOME/Packages/urdf/lib/urdfdom/cmake"
URDFDOM_HEADERS_DIR="$HOME/Packages/urdf/lib/urdfdom_headers/cmake"
CONSOLE_BRIDGE_DIR="$HOME/Packages/urdf/lib/console_bridge/cmake"
TINYXML2_DIR="$HOME/Packages/urdf/CMake"
NUMPY_INCLUDE_DIR="/usr/lib/python3/dist-packages/numpy/core/include"


BUILDSYSTEM="Ninja Multi-Config"

# ------------------------------------------------------------------------

cmake -G ${BUILDSYSTEM} -B ${BUILD_DIR} -S ${SOURCE_DIR} \
      -DBUILD_BENCHMARKING:BOOL=ON \
      -DBUILD_TESTING:BOOL=OFF \
      -DCMAKE_BUILD_TYPE="Release" \
      -DCMAKE_INSTALL_PREFIX:PATH=${INSTALL_DIR} \
      -DENABLE_MODULE_IRRLICHT:BOOL=ON \
      -DENABLE_MODULE_PARSERS:BOOL=ON \
      -DENABLE_MODULE_PYTHON:BOOL=ON \
      -DENABLE_MODULE_SENSOR:BOOL=ON \
      -DENABLE_MODULE_VEHICLE:BOOL=ON \
      -Durdfdom_DIR:PATH=${URDFDOM_DIR} \
      -Durdfdom_headers_DIR:PATH=${URDFDOM_HEADERS_DIR} \
      -Dconsole_bridge_DIR:PATH=${CONSOLE_BRIDGE_DIR} \
      -Dtinyxml2_DIR:PATH=${TINYXML2_DIR} \
      -DNUMPY_INCLUDE_DIR:PATH=${NUMPY_INCLUDE_DIR} \
      -DOptiX_INCLUDE:PATH=${OPTIX_INCLUDE_DIR}

cmake --build ${BUILD_DIR} --config Release
cd build
ninja install