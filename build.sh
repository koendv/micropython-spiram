#!/bin/sh
ZIP=$PWD/firmware.zip
cat > readme.txt <<EOD
Micropython firmware for stm32h7a3. Boot in dfu mode and install with:
	dfu-util -a 0 -D firmware.dfu

Compiler info:
EOD
arm-none-eabi-gcc --v >> readme.txt
date >> readme.txt
rm -f ${ZIP}

git clone https://github.com/micropython/micropython/
cd micropython
git checkout -q c8b055717805500494a14870a4200bbc933fe337
wget https://raw.githubusercontent.com/koendv/micropython-spiram/main/stm32h7a3.patch
patch -p1 < stm32h7a3.patch
make -C ports/stm32 submodules
make -C mpy-cross/
for BRD in DEVEBOX_STM32H7A3  WeActStudioSTM32H7A3
do
  make -C ports/stm32 BOARD=$BRD
  (cd ports/stm32; zip -9 ${ZIP} ./build-${BRD}/firmware.dfu ./build-${BRD}/firmware.elf)
done

zipinfo ${ZIP} >> readme.txt
zip ${ZIP} readme.txt
zipinfo ${ZIP}
