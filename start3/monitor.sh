#!/bin/bash
PATH=/Users/Victor/esp/esp-idf/tools:/Users/Victor/esp/xtensa-esp32-elf/bin:$PATH
/Users/Victor/esp/esp-idf/tools/idf_monitor.py -p /dev/cu.SLAB_USBtoUART -b 115200 `pwd`/cmake-build-xtensa/start3.elf