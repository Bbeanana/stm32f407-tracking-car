#!/bin/bash
# ============================================================
# 寻迹避障小车固件一键编译脚本（Linux，arm-none-eabi-gcc）
#
# 用法:
#   ./build.sh                      # 正常固件 -> build/tracking_car.*
#   CAR_BRINGUP_MODE=3 ./build.sh   # 校准固件 -> build/tracking_car_mode3.*
#   CAR_BRINGUP_MODE=1 ./build.sh   # 电机测试固件 -> build/tracking_car_mode1.*
#   CAR_BRINGUP_MODE=2 ./build.sh   # 传感器测试固件 -> build/tracking_car_mode2.*
# ============================================================
set -e
cd "$(dirname "$0")"

OBJ=build
mkdir -p $OBJ

OUT_NAME=tracking_car
MODE_DEF=
if [ -n "${CAR_BRINGUP_MODE:-}" ]; then
    OUT_NAME="tracking_car_mode${CAR_BRINGUP_MODE}"
    MODE_DEF="-DCAR_BRINGUP_MODE=${CAR_BRINGUP_MODE}"
fi

echo "== [1/3] 编译启动文件 =="
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -x assembler-with-cpp \
    -c Core/Startup/startup_stm32f407ighx.s -o $OBJ/startup.o

echo "== [2/3] 编译+链接全部源文件（$OUT_NAME） =="
SRCS=$(find Core/Src Drivers/STM32F4xx_HAL_Driver/Src Users -name "*.c")
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O0 -g3 \
    -ffunction-sections -fdata-sections \
    -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx $MODE_DEF \
    -ICore/Inc \
    -IDrivers/STM32F4xx_HAL_Driver/Inc \
    -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
    -IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
    -IDrivers/CMSIS/Include \
    -IUsers/device -IUsers/bsp -IUsers/alg -IUsers/application \
    --specs=nano.specs -u _printf_float \
    -T STM32F407IGHX_FLASH.ld -Wl,--gc-sections,-Map=$OBJ/$OUT_NAME.map \
    $SRCS $OBJ/startup.o -o $OBJ/$OUT_NAME.elf

echo "== [3/3] 生成 hex / bin =="
arm-none-eabi-objcopy -Oihex $OBJ/$OUT_NAME.elf $OBJ/$OUT_NAME.hex
arm-none-eabi-objcopy -Obinary $OBJ/$OUT_NAME.elf $OBJ/$OUT_NAME.bin
arm-none-eabi-size $OBJ/$OUT_NAME.elf
echo "✅ 编译完成: $OBJ/$OUT_NAME.elf（烧录用 .elf 或 .hex/.bin）"
