#!/usr/bin/env bash

SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  RUNDIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
RUNDIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"
BUILDDIR="$( cd -P "$( dirname "$RUNDIR/../cmake-build-xtensa/./" )" >/dev/null 2>&1 && pwd )"
echo "script from:${RUNDIR}"
echo "build from:${BUILDDIR}"
cmake --build "${BUILDDIR}" --target menuconfig -- -j 4