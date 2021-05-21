cmake_minimum_required(VERSION 3.13)
project(PICoBoot C ASM)
set(CMAKE_C_STANDARD 99)

include(CMake/FindXC16.cmake)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

add_compile_options(-g -O2 -std=c99)

include_directories(Components)
file(GLOB_RECURSE PICo24_Core_SOURCES "Components/PICo24/*.*")
file(GLOB_RECURSE PICoBoot_SOURCES "Components/PICoBoot/*.*")

function(picoboot_add_board BOARD_NAME CPU_NAME HEAP_SIZE)
    set(BOARD_DIR "Boards/${BOARD_NAME}")
    set(OUT_NAME "PICoBoot_${BOARD_NAME}")
    set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/${BOARD_DIR}/PICoBoot.ld)

    file(GLOB_RECURSE BOARD_SOURCES "${BOARD_DIR}/*.*")
    add_executable(${OUT_NAME}.elf ${PICo24_Core_SOURCES} ${PICoBoot_SOURCES} ${BOARD_SOURCES} main.c)
    target_include_directories(${OUT_NAME}.elf PUBLIC ${BOARD_DIR})
    target_compile_options(${OUT_NAME}.elf PUBLIC -mcpu=${CPU_NAME})
    target_link_options(${OUT_NAME}.elf PUBLIC -Wl,--gc-sections,--script=${LINKER_SCRIPT},--heap=${HEAP_SIZE})

    set(HEX_FILE ${PROJECT_BINARY_DIR}/${OUT_NAME}.hex)
    add_custom_command(TARGET ${OUT_NAME}.elf POST_BUILD
            COMMAND ${XC16_BIN2HEX} ${OUT_NAME}.elf -a
            COMMENT "Building ${HEX_FILE}")
endfunction()


picoboot_add_board(PotatoPi_PICo24 24FJ256GB206 16384)
picoboot_add_board(CartBoy_RW_v2 24FJ256GB108 1024)
