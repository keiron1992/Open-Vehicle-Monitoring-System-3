- name: Deep Patching & Linker Fixes
        run: |
          cd vehicle/OVMS.V3
          
          # 1. CREATE THE STUB FILE
          # This provides the missing symbols without cluttering existing source files.
          cat <<EOF > main/linker_fix.cpp
          #include <string>
          class OvmsWriter;
          extern "C" {
            bool ovms_partition_table_has_factory() { return false; }
            void* ovms_partition_table_find() { return nullptr; }
            void ovms_partition_table_list(OvmsWriter* writer) {}
            void ovms_partition_table_upgrade(OvmsWriter* writer) {}
            int ovms_partition_table_get_type() { return 0; }
            bool module_check_heap_alert(int type, OvmsWriter* writer) { return false; }
            
            // BackgroundShell Linker Hooks (Mangled names for ESP32/Xtensa)
            void _ZN15BackgroundShellC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEiiiij() {}
            void _ZN15BackgroundShell4InitEv() {}
            void _ZN15BackgroundShell9QueueCharEc() {}
            void _ZN15BackgroundShell10QueueCharsEPKci() {}
          }
          std::string ovms_partition_table_get_type_string(int type) { return "unknown"; }
          EOF

          # 2. REWRITE MAIN CMAKELISTS.TXT (GLOBBING FIX)
          # file(GLOB ...) ensures it only compiles files that ACTUALLY exist.
          cat <<EOF > main/CMakeLists.txt
          file(GLOB srcs *.cpp *.c)
          idf_component_register(SRCS \${srcs}
                                 INCLUDE_DIRS "."
                                 LDFRAGMENTS "linker.lf")
          EOF

          # 3. OVERWRITE TOP-LEVEL CMAKELISTS.TXT
          # This incorporates your exact exclusion list and console fixes.
          cat <<EOF > CMakeLists.txt
          cmake_minimum_required(VERSION 3.16)
          set(EXTRA_COMPONENT_DIRS "main" "components")
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
              "console_ssh" "console_telnet")
          add_compile_options("-w" "-Wno-error=format-overflow" "-Wno-error=format-truncation" "-Wno-maybe-uninitialized")
          include(\$ENV{IDF_PATH}/tools/cmake/project.cmake)
          project(ovms3)
          EOF

          # 4. Final Cleanup
          find . -name "gsmpppos.h" -exec sed -i '1i#define PPP_SUPPORT 1\n#include "lwip/opt.h"\n#include "netif/ppp/ppp.h"' {} +
          find . -name "CMakeLists.txt" -not -path "./CMakeLists.txt" -not -path "./main/CMakeLists.txt" -exec sed -i 's/WHOLE_ARCHIVE//g' {} +
          find . -name "dependencies.lock" -delete
          find . -name "idf_component.yml" -delete
