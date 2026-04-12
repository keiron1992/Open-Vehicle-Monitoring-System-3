cmake_minimum_required(VERSION 3.16)

# Ensure the build sees your custom stubs and components
set(EXTRA_COMPONENT_DIRS "main" "components")

# EXCLUDE everything that isn't the Maxus T90 or Core OVMS
# Note: I removed the T90 from the exclusion list below
set(EXCLUDE_COMPONENTS 
    "vehicle_byd_atto3" "vehicle_renaultzoe_ph2" "vehicle_vweup" 
    "vehicle_vwegolf" "vehicle_niu_gtevo" "vehicle_voltampera"
    "vehicle_bmwi3" "vehicle_cadillac_cts" "vehicle_corvette" "vehicle_boltev" 
    "vehicle_dbc" "vehicle_demo" "vehicle_fiat500" "vehicle_hyundai_ioniqvfl"
    "vehicle_jaguaripace" "vehicle_kianiroev" "vehicle_kiasoulev" "vehicle_mercedes_b250e"
    "vehicle_maxus_edeliver3" "vehicle_mgev" "vehicle_trio" "vehicle_nissanleaf"
    "vehicle_empty" "vehicle_renaulttwizy" "vehicle_renaultzoe" "vehicle_smarted"
    "vehicle_smarteq" "vehicle_teslamodel3" "vehicle_thinkcity" "vehicle_teslaroadster"
    "vehicle_teslamodels" "vehicle_toyotarav4ev" "vehicle_track" "vehicle_zeva" 
    "vehicle_hyundai_ioniq5" "vehicle_hyundai_ioniq" "vehicle_mgev_th" 
    "vehicle_volkswagen_tp" "vehicle_fiat500e" "vehicle_renault_obd" 
    "vehicle_vweup_obd" "vehicle_mitsubishi"
    # Add these to reduce binary size and potential linker errors
    "console_ssh" 
    "console_telnet"
)

# These flags help bypass the "warnings as errors" that often stop 
# older OVMS code from compiling on newer GCC versions.
add_compile_options("-w" "-Wno-error=format-overflow" "-Wno-error=format-truncation" "-Wno-maybe-uninitialized")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ovms3)
