# Standalone ESP-IDF project for the link-bench remote (P0 calibration).
#
# Why it is a separate project (and NOT part of main/):
#   The bench firmware must be flashable alone, with no UI/joystick/rc_link code, because its only job
#   is to be the RF peer of the flight-controller bench. Keeping it out of the main build means the
#   flight firmware is untouched and the two images can be flashed independently.
#
# Components are reused verbatim from the main project (../../components):
#   radio   : components/radio/radio_e220.c  (UART + M0/M1/AUX + register read/verify)
#   rc_core : components/rc_core/include/app_config.h (pins, baud, channel, packet length -- one source of truth)
#
# Build (from this directory):
#   . E:\ESP_PROJECT\controller\tools\idf_env.ps1     # or any ESP-IDF v6.x export
#   idf.py set-target esp32s3
#   idf.py build
#   idf.py -p COMx flash
cmake_minimum_required(VERSION 3.16)

# Relative on purpose: after the sync this file lives at
#   E:\ESP_PROJECT\controller\bench\link_bench\CMakeLists.txt
# so ../../components is exactly the main project's components tree.
#
# 只引入本工程真正需要的两个组件（radio + rc_core），**不要**把整棵 components/ 塞进来：
# 那会把 ui/bsp 一起拉进构建，而它们依赖主工程的 managed_components
# （espressif__esp_lvgl_port / lvgl__lvgl，位于 controller/managed_components，bench 工程里没有），
# cmake 会直接以 "component 'espressif__esp_lvgl_port' could not be found" 失败。
# bench 只需要：
#   radio   : components/radio/radio_e220.c（UART + M0/M1/AUX + 寄存器只读）
#   rc_core : components/rc_core/include/app_config.h（引脚/波特率/信道/分包长度唯一来源）
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../components/radio"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/rc_core")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(link_bench)
