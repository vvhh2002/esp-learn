set(COMPONENT_ADD_INCLUDEDIRS "btstack/3rd-party/bluedroid/decoder/include"
                              "btstack/3rd-party/bluedroid/encoder/include"
                              "btstack/3rd-party/hxcmod-player"
                              "btstack/3rd-party/hxcmod-player/mods"
                              "btstack/3rd-party/md5"
                              "btstack/3rd-party/yxml"
                              "btstack/src/classic"
                              "btstack/src/ble/gatt-service"
                              "btstack/src/ble"
                              "btstack/src/classic"
                              "btstack/src"
                              "btstack/platform/embedded"
                              "btstack/platform/freertos"
                              "./"
        )

set(COMPONENT_SRCDIRS "btstack/3rd-party/bluedroid/decoder/srce"
                      "btstack/3rd-party/bluedroid/encoder/srce"
                      "btstack/3rd-party/hxcmod-player"
                      "btstack/3rd-party/hxcmod-player/mods"
                      "btstack/3rd-party/md5"
                      "btstack/src/ble/gatt-service"
                      "btstack/src/ble"
                      "btstack/src/classic"
                      "btstack/src"
                      "btstack/platform/freertos"
                      )
#set(COMPONENT_SRCS "test.c")
set(COMPONENT_PRIV_INCLUDEDIRS "btstack/3rd-party/micro-ecc")
set(COMPONENT_PRIV_REQUIRES "micro-ecc")
set(PRIV_INCLUDE_DIRS "btstack/3rd-party/micro-ecc")
set(PRIV_REQUIRES "micro-ecc")

set(COMPONENT_REQUIRES "nvs_flash" "bt")
register_component()

