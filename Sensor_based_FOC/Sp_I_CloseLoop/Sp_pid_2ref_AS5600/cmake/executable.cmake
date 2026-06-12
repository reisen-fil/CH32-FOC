
# 编译可执行文件
set(SUFFIX ".elf")

add_executable(${PROJECT_NAME} ${APP_SOURCES} ${BSP_SOURCES} ${LINKER_SCRIPT})

set_target_properties(${PROJECT_NAME} PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib_obj"
  SUFFIX "${SUFFIX}"
  LINKER_LANGUAGE C
)

# 链接静态库
target_link_libraries(
  ${PROJECT_NAME}
  -Wl,--start-group
  gcc
  # m   # 用到了三角函数,链接数学库 `libm.a`
  c
  -Wl,--whole-archive
  -Wl,--no-whole-archive
  -Wl,--end-group
  # -Wl,-u,_printf_float
  -Wl,-lIQmath_RV32     # IQmath定点运算浮点
)


# Build target
add_custom_command(
  TARGET ${PROJECT_NAME}
  POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex ${CMAKE_BINARY_DIR}/lib_obj/${PROJECT_NAME}${SUFFIX}
          ${CMAKE_BINARY_DIR}/lib_obj/${PROJECT_NAME}.hex DEPENDS ${PROJECT_NAME}
  COMMENT "Generating .hex file ...")

# Erase and Flash Program
add_custom_command(
  OUTPUT erase_sector
  COMMAND
    # ${CMAKE_SOURCE_DIR}/OpenOCD/bin/openocd -f ${CMAKE_SOURCE_DIR}/OpenOCD/bin/wch-riscv.cfg -c
    # init -c halt -c "flash erase_sector wch_riscv 0 last " -c exit)

    $ENV{OPENOCD_RISCV}/bin/openocd -f $ENV{OPENOCD_RISCV}/bin/wch-riscv.cfg -c
    init -c halt -c "flash erase_sector wch_riscv 0 last " -c exit)    

add_custom_command(
  OUTPUT program_image
  COMMAND
    # ${CMAKE_SOURCE_DIR}/OpenOCD/bin/openocd -f ${CMAKE_SOURCE_DIR}/OpenOCD/bin/wch-riscv.cfg -c
    # init -c halt -c "program ${CMAKE_BINARY_DIR}/lib_obj/${PROJECT_NAME}.hex " -c
    # "verify_image ${CMAKE_BINARY_DIR}/lib_obj/${PROJECT_NAME}.hex " -c exit)

    $ENV{OPENOCD_RISCV}/bin/openocd -f $ENV{OPENOCD_RISCV}/bin/wch-riscv.cfg -c
    init -c halt -c "program ${CMAKE_BINARY_DIR}/lib_obj/${PROJECT_NAME}.hex " -c
    "verify_image ${CMAKE_BINARY_DIR}/lib_obj/${PROJECT_NAME}.hex " -c exit)    

# use wch-link reset MCU
add_custom_command(
  OUTPUT wlink_resume
  COMMAND
    # ${CMAKE_SOURCE_DIR}/OpenOCD/bin/openocd -f ${CMAKE_SOURCE_DIR}/OpenOCD/bin/wch-riscv.cfg -c
    # init -c halt -c wlink_reset_resume -c exit)    
    $ENV{OPENOCD_RISCV}/bin/openocd -f $ENV{OPENOCD_RISCV}/bin/wch-riscv.cfg -c
    init -c halt -c wlink_reset_resume -c exit)     

add_custom_target(flash DEPENDS erase_sector program_image wlink_resume) 

message("")
message("Project: ${PROJECT_NAME}")
message("  LIST_FILE=${CMAKE_PARENT_LIST_FILE}")
message("  TOOLCHAIN=${TOOLCHAIN}")
message("  GCC_SUFFIX=${GCC_SUFFIX}")
message("")
message("  CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")
message("  CMAKE_C_COMPILER=${CMAKE_C_COMPILER}")
message("  CMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}")
message("  CMAKE_ASM_COMPILER=${CMAKE_ASM_COMPILER}")
message("  CMAKE_LINKER=${CMAKE_LINKER}")
message("  CMAKE_OBJCOPY=${CMAKE_OBJCOPY}")
message("  CMAKE_OBJDUMP=${CMAKE_OBJDUMP}")
message("  CMAKE_RANLIB=${CMAKE_RANLIB}")
message("  CMAKE_MAKE_PROGRAM=${CMAKE_MAKE_PROGRAM}")
message("")
message("  CMAKE_C_FLAGS=${CMAKE_C_FLAGS}")
message("  CMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}")
message("  CMAKE_ASM_FLAGS=${CMAKE_ASM_FLAGS}")
message("  LDFLAGS=${LDFLAGS}")
message("  CMAKE_BINARY_DIR=${CMAKE_BINARY_DIR}")
message("Makefile created.")
message("")
