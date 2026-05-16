#!/bin/bash
# Запуск: source set_compiler.sh

# Получаем абсолютный путь к папке со скриптом
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Формируем путь к компилятору
COMPILER_PATH="${SCRIPT_DIR}/sdk/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin/arm-rockchip830-linux-uclibcgnueabihf-"
SDK_PATH="${SCRIPT_DIR}/sdk/"

# Устанавливаем переменную окружения
export GLIBC_COMPILER="$COMPILER_PATH"
export LUCKFOX_SDK_PATH="$SDK_PATH"

echo "✅ GLIBC_COMPILER успешно установлен в: $GLIBC_COMPILER"
echo "✅ LUCKFOX_SDK_PATH успешно установлен в: $LUCKFOX_SDK_PATH"