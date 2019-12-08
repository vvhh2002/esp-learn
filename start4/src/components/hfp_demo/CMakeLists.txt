set(COMPONENT_SRCS
        "src/btstack_stdin_esp32.c"
        "src/btstack_tlv_esp32.c"
        "src/btstack_audio_esp32.c"
        "src/hfp_hf_demo.c"
        "src/sco_demo_util.c"
        )
set(COMPONENT_ADD_INCLUDEDIRS "include")
set(COMPONENT_REQUIRES btstack)
register_component()