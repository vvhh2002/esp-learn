#!/usr/bin/env bash
PATH=/Users/Victor/esp/esp-idf/tools:/Users/Victor/esp/xtensa-esp32-elf/bin:$PATH
PROJECT_NAME=start4

SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  RUNDIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
RUNDIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"
BUILDDIR="$( cd -P "$( dirname "$RUNDIR/../cmake-build-xtensa/./" )" >/dev/null 2>&1 && pwd )"
echo "no build ${BUILDDIR}";
cd "${BUILDDIR}" || exit

/Users/Victor/esp/esp-idf/tools/idf_monitor.py -p /dev/cu.SLAB_USBtoUART -b 115200 "${PROJECT_NAME}".elf