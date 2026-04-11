cmake_minimum_required(VERSION 3.16)

# Global compiler settings to prevent minor warnings from stopping the build
add_compile_options(
    "-w"
    "-Wno-error=format-truncation" 
    "-Wno-error=int-in-bool-context" 
    "-Wno-error=format-overflow"
    "-Wno-maybe-uninitialized"
    "-Wno-error=maybe-uninitialized"
    "-DCONFIG_OVMS_COMP_CELLULAR_NONE"
)

# Tell the builder to look in 'main' for the app_main function
set(EXTRA_COMPONENT_DIRS "components" "main")

# Exclude list to keep the build fast and avoid memory crashes
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

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ovms3)
