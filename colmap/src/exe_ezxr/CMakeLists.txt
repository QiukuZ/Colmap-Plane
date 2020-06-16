# Changed by:
# ezxr-sx-zhangqunkang
# 

set(FOLDER_NAME "exe_ezxr")

if(IS_MSVC)
    add_compile_options("/bigobj")
endif()

COLMAP_ADD_EXECUTABLE(optim_ezxr_exe optim_ezxr.cc)
set_target_properties(optim_ezxr_exe PROPERTIES OUTPUT_NAME optim_ezxr)
