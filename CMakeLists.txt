cmake_minimum_required(VERSION 3.16)

# 1. Setup the project paths (CRITICAL: Must be before 'include')
set(EXTRA_COMPONENT_DIRS "main" "components")

# 2. Disable heavy car modules to prevent memory/build errors
set(EXCLUDE_COMPONENTS 
    "vehicle_bmwi3" "vehicle_cadillac_cts" "vehicle_corvette" "vehicle_boltev" 
    "vehicle_dbc" "vehicle_demo" "vehicle_fiat500" "vehicle_hyundai_ioniqvfl"
    "vehicle_jaguaripace" "vehicle_kianiroev" "vehicle_kiasoulev" "vehicle_mercedes_b250e"
    "vehicle_maxus_edeliver3" "vehicle_mgev" "vehicle_trio" "vehicle_nissanleaf"
    "vehicle_empty" "vehicle_renaulttwizy" "vehicle_renaultzoe" "vehicle_smarted"
    "vehicle_smarteq" "vehicle_teslamodel3" "vehicle_thinkcity" "vehicle_teslaroadster"
    "vehicle_teslamodels" "vehicle_toyotarav4ev" "vehicle_voltampera" "vehicle_vweup"
    "vehicle_track" "vehicle_zeva" "vehicle_hyundai_ioniq5" "vehicle_hyundai_ioniq"
    "vehicle_mgev_th" "vehicle_volkswagen_tp" "vehicle_fiat500e"
)

# 3. Global compiler flags
add_compile_options(
    "-w"
    "-Wno-error=format-truncation" 
    "-Wno-maybe-uninitialized"
    "-DCONFIG_OVMS_COMP_CELLULAR_NONE"
)

# 4. Include the ESP-IDF project logic
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ovms3)
