#!/bin/sh
# SPDX-License-Identifier: MIT
# Fetches the STMicroelectronics STM32G4 HAL, the G4 CMSIS device files and
# CMSIS core at pinned commits into boards/nucleo_g474re/vendor (not
# committed). The same versions are the submodules of STM32CubeG4 v1.6.x.
#   stm32g4xx_hal_driver v1.2.7  BSD-3-Clause
#   cmsis_device_g4      v1.2.6  Apache-2.0
#   cmsis_core           v5.9.0  Apache-2.0
set -eu
HERE="$(cd "$(dirname "$0")" && pwd)"
DEST="${1:-$HERE/vendor}"
mkdir -p "$DEST"

fetch() { # name url sha
  if [ -d "$DEST/$1/.git" ] && [ "$(git -C "$DEST/$1" rev-parse HEAD)" = "$3" ]; then
    echo "$1 already at $3"; return
  fi
  rm -rf "${DEST:?}/$1"
  git init -q "$DEST/$1"
  git -C "$DEST/$1" fetch -q --depth 1 "$2" "$3"
  git -C "$DEST/$1" checkout -q FETCH_HEAD
  echo "$1 $3"
}

fetch stm32g4xx_hal_driver https://github.com/STMicroelectronics/stm32g4xx_hal_driver.git f5929f431f9effe45fbe18f5337e4753ced9ac92
fetch cmsis_device_g4      https://github.com/STMicroelectronics/cmsis_device_g4.git      25664ddc3a7624ae9627ae8c4c672073dc5b2539
fetch cmsis_core           https://github.com/STMicroelectronics/cmsis_core.git           1a2f783395dd72778a98ebe7771f36ca1c30a35e
