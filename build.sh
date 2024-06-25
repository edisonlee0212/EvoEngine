#!/bin/bash

quit() {
    echo $1
    popd > /dev/null 
    exit $2;
}

pushd `dirname $0` > /dev/null
ROOT_PATH="${PWD}"

CLEAN=1
NO_TEST=1
VERBOSE=0
BUILD_TYPE="Release"

while [[ "$1" != "" ]] ; do
  case $1 in
    --clean)   CLEAN=1 ;;
    --no-test) NO_TEST=1 ;;
    --verbose) VERBOSE=1 ;;
    debug)     BUILD_TYPE="Debug" ;;
    release)   BUILD_TYPE="Release" ;;
    *) quit "Usage: $0 [--clean] [--no-test] [--verbose] [build_type]" 1 ;;
  esac

  shift
done

BUILD_CONFIG="x64-${BUILD_TYPE}"
echo == Build config $BUILD_CONFIG

OUTPUT_PATH="${ROOT_PATH}/out"
BUILD_PATH="${OUTPUT_PATH}/build/${BUILD_CONFIG}"

if [[ $CLEAN -eq 1 ]]; then
  echo "== Remove ${BUILD_PATH}"
  [[ -d "${BUILD_PATH}" ]] && rm -rf "${BUILD_PATH}"
fi

[[ ! -d "${BUILD_PATH}" ]] && mkdir -p "${BUILD_PATH}"
cd "${BUILD_PATH}"

CMAKE_ARGS="
  -G Ninja
  -DCMAKE_BUILD_TYPE=${BUILD_TYPE}
  -DCMAKE_CXX_COMPILER:FILEPATH=g++
  -DCMAKE_INSTALL_PREFIX:PATH=${OUTPUT_PATH}/install/${BUILD_CONFIG}
  -DCMAKE_PREFIX_PATH=$CONDA_PREFIX
  -S ${ROOT_PATH}
"

[[ $VERBOSE -eq 1 ]] && CMAKE_ARGS="${CMAKE_ARGS} --log-level=VERBOSE"

echo == cmake $CMAKE_ARGS
cmake $CMAKE_ARGS
EXIT_CODE=$?
[[ $EXIT_CODE -ne 0 ]] && quit 'cmake failed' $EXIT_CODE

NINJA_ARGS=""

[[ $VERBOSE -eq 1 ]] && NINJA_ARGS="${NINJA_ARGS} -v"

echo == ninja $NINJA_ARGS
ninja $NINJA_ARGS
EXIT_CODE=$?
[[ $EXIT_CODE -ne 0 ]] && quit 'ninja failed' $EXIT_CODE

[[ $NO_TEST -eq 1 ]] && quit 'Done' 0

CTEST_ARGS=""

[[ $VERBOSE -eq 1 ]] && CTEST_ARGS="${CTEST_ARGS} -V"

echo == ctest $CTEST_ARGS
ctest $CTEST_ARGS
EXIT_CODE=$?
[[ $EXIT_CODE -ne 0 ]] && quit 'ctest failed' $EXIT_CODE

quit 'Done' 0

