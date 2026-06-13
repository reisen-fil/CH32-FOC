
# 调用构建指令时添加工程名PROJ选项( ... -DPROJ=PROJ)
if(NOT PROJ)
  set(PROJ "empty")
  message(
    STATUS
      "PROJ not set, use ${PROJ} as PROJ. Also, you can set it manually. e.g. -DPROJ=hello_world"
  )
else()
  message("PROJ = ${PROJ}")
endif()

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")
cmake_minimum_required(VERSION 3.20)

# riscv架构设置
set(MARCH rv32imac)
set(MABI ilp32)

# 工具链设置
include(./cmake/toolchain.cmake)

# 项目设置
project(${PROJ} C CXX ASM)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 99)

# 编译参数 一般不用改
add_compile_options(
  -march=${MARCH} -mabi=${MABI}
  -mcmodel=medany 
  -msmall-data-limit=8 
  -mno-save-restore
)
add_compile_options(
  -fmessage-length=0 
  -fsigned-char 
  -ffunction-sections 
  -fdata-sections 
  -fno-common

  -g3
  -gdwarf-4
  -fno-omit-frame-pointer
)

# 编译等级
add_compile_options(-O0)

# 编译信息等级
add_compile_options(-Wall)

# 头文件路径
include_directories(
  # Core
  components/bsp/inc
  components/cmsis

  # user
  APP/AppFunction/inc  
  APP/MC_FOC/inc
  APP/Peri_Driver/inc
)

# 宏定义
set(CMAKE_BUILD_TYPE Debug)   #禁止编译器调试优化,添加-g选项
# add_definitions(-DDEBUG=1)

# 源码文件
file(GLOB_RECURSE APP_SOURCES 

  "APP/AppFunction/src/*.c"
  "APP/MC_FOC/src/*.c"
  "APP/Peri_Driver/src/*.c"

)

file(GLOB_RECURSE BSP_CORE_SOURCES 

  # Core
  "components/cmsis/core_riscv.c" 
  "components/startup/startup_ch32v20x_D6.S"

)

# 定义 BSP 模块开关（TRUE=编译，FALSE=不编译）
# set(ENABLE_RCC     TRUE)        # 系统时钟
# set(ENABLE_GPIO    TRUE)        # GPIO
# set(ENABLE_TIMER   TRUE)        # TIM定时器
# set(ENABLE_USART   TRUE)        # USART串口
# set(ENABLE_ADC     TRUE)        # ADC转换
# set(ENABLE_DMA     TRUE)        # DMA
# set(ENABLE_I2C     TRUE)        # I2C总线
# set(ENABLE_MISC    TRUE)        # NVIC/PFIC杂项
# set(ENABLE_EXTI    FALSE)       # 外部中断（未使用）
# set(ENABLE_SPI     FALSE)       # SPI总线（未使用）
# set(ENABLE_CAN     FALSE)       # CAN总线（未使用）
# set(ENABLE_RTC     FALSE)       # 实时时钟（未使用）
# set(ENABLE_IWDG    FALSE)       # 独立看门狗（未使用）
# set(ENABLE_WWDG    FALSE)       # 窗口看门狗（未使用）
# set(ENABLE_BKP     FALSE)       # 备份寄存器（未使用）
# set(ENABLE_CRC     FALSE)       # CRC校验（未使用）
# set(ENABLE_FLASH   FALSE)       # Flash编程（未使用）
# set(ENABLE_OPA     TRUE)       # 运算放大器
# set(ENABLE_PWR     FALSE)       # 电源管理（未使用）
# set(ENABLE_DBGMCU  FALSE)       # Debug MCU（未使用）

# 定义 BSP 模块开关（TRUE=编译，FALSE=不编译）
set(ENABLE_RCC     TRUE)        # 系统时钟
set(ENABLE_GPIO    TRUE)        # GPIO
set(ENABLE_TIMER   TRUE)        # TIM定时器
set(ENABLE_USART   TRUE)        # USART串口
set(ENABLE_ADC     TRUE)        # ADC转换
set(ENABLE_DMA     TRUE)        # DMA
set(ENABLE_I2C     TRUE)        # I2C总线
set(ENABLE_MISC    TRUE)        # NVIC/PFIC杂项
set(ENABLE_EXTI    TRUE)       # 外部中断（未使用）
set(ENABLE_SPI     TRUE)       # SPI总线（未使用）
set(ENABLE_CAN     TRUE)       # CAN总线（未使用）
set(ENABLE_RTC     TRUE)       # 实时时钟（未使用）
set(ENABLE_IWDG    TRUE)       # 独立看门狗（未使用）
set(ENABLE_WWDG    TRUE)       # 窗口看门狗（未使用）
set(ENABLE_BKP     TRUE)       # 备份寄存器（未使用）
set(ENABLE_CRC     TRUE)       # CRC校验（未使用）
set(ENABLE_FLASH   TRUE)       # Flash编程（未使用）
set(ENABLE_OPA     TRUE)       # 运算放大器
set(ENABLE_PWR     TRUE)       # 电源管理（未使用）
set(ENABLE_DBGMCU  TRUE)       # Debug MCU（未使用）

# 构建 BSP 源文件列表
set(BSP_SOURCES ${BSP_CORE_SOURCES})
list(APPEND BSP_SOURCES "components/bsp/src/system_ch32v20x.c")

# 条件性添加 BSP 模块
if(ENABLE_RCC)
  list(APPEND BSP_SOURCES "components/bsp/src/ch32v20x_rcc.c")
endif()

if(ENABLE_GPIO)
  list(APPEND BSP_SOURCES "components/bsp/src/ch32v20x_gpio.c")
endif()

if(ENABLE_TIMER)
  list(APPEND BSP_SOURCES "components/bsp/src/ch32v20x_tim.c")
endif()

if(ENABLE_USART)
  list(APPEND BSP_SOURCES "components/bsp/src/ch32v20x_usart.c")
endif()

if(ENABLE_ADC)
  list(APPEND BSP_SOURCES "components/bsp/src/ch32v20x_adc.c")
endif()

if(ENABLE_DMA)
  list(APPEND BSP_SOURCES "components/bsp/src/ch32v20x_dma.c")
endif()

if(ENABLE_I2C)
  list(APPEND BSP_SOURCES "components/bsp/src/ch32v20x_i2c.c")
endif()

if(ENABLE_MISC)
  list(APPEND BSP_SOURCES "components/bsp/src/ch32v20x_misc.c")
endif()

if(ENABLE_EXTI)
  list(APPEND BSP_SOURCES "components/bsp/src/ch32v20x_exti.c")
endif()

if(ENABLE_SPI)
  list(APPEND BSP_SOURCES "components/bsp/src/ch32v20x_spi.c")
endif()

if(ENABLE_CAN)
  list(APPEND BSP_SOURCES "components/bsp/src/ch32v20x_can.c")
endif()

if(ENABLE_RTC)
  list(APPEND BSP_SOURCES "components/bsp/src/ch32v20x_rtc.c")
endif()

if(ENABLE_IWDG)
  list(APPEND BSP_SOURCES "components/bsp/src/ch32v20x_iwdg.c")
endif()

if(ENABLE_WWDG)
  list(APPEND BSP_SOURCES "components/bsp/src/ch32v20x_wwdg.c")
endif()

if(ENABLE_BKP)
  list(APPEND BSP_SOURCES "components/bsp/src/ch32v20x_bkp.c")
endif()

if(ENABLE_CRC)
  list(APPEND BSP_SOURCES "components/bsp/src/ch32v20x_crc.c")
endif()

if(ENABLE_FLASH)
  list(APPEND BSP_SOURCES "components/bsp/src/ch32v20x_flash.c")
endif()

if(ENABLE_OPA)
  list(APPEND BSP_SOURCES "components/bsp/src/ch32v20x_opa.c")
endif()

if(ENABLE_PWR)
  list(APPEND BSP_SOURCES "components/bsp/src/ch32v20x_pwr.c")
endif()

if(ENABLE_DBGMCU)
  list(APPEND BSP_SOURCES "components/bsp/src/ch32v20x_dbgmcu.c")
endif()


# 链接参数
set(LINKER_SCRIPT  ${CMAKE_SOURCE_DIR}/components/Ld/Link.ld)
add_link_options(
    -march=${MARCH} -mabi=${MABI}
    -nostartfiles 
    -Xlinker --gc-sections  
    -Wl,--print-memory-usage
    -Wl,-Map,./lib_obj/${PROJECT_NAME}.map 
    --specs=nano.specs 
    --specs=nosys.specs
    # -Wl,--no-warn-rwx-segments #工具链riscv-none-embed(8.2.0)不兼容
)
add_link_options(-T ${LINKER_SCRIPT})

# build and program
include(./cmake/executable.cmake)   
