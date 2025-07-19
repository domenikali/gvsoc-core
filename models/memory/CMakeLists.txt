vp_model(NAME memory.dramsys
    SOURCES "dramsys.cpp"
    )

set_source_files_properties(dramsys.cpp PROPERTIES COMPILE_DEFINITIONS "DRAMSYS_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}\"")

vp_model(NAME memory.pcm_mem
   SOURCES "pcm_mcm.cpp")
set_source_files_properties(pcm_mem.cpp PROPERTIES COMPILE_DEFINITIONS "PCM_MEM_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}\"")
