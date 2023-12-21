# nav_mecanum_driver

This is driver code for nav_mecanum car.


## Compile

### STM32CubeIDE

Open project with STM32CubeIDE and just click `RUN` button.

### CMake and gcc-arm-none-eabi

Install gcc-arm-none-eabi if you haven't install it.

you can install it by simply `apt install gcc-arm-none-eabi` but it's old version before 2021.

Download latest version of gcc-arm-none-eabi from [official website](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)

For example, download [arm-gnu-toolchain-13.2.rel1-x86_64-arm-none-eabi.tar.xz](https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/arm-gnu-toolchain-13.2.rel1-x86_64-arm-none-eabi.tar.xz)

```bash
tar -xf arm-gnu-toolchain-13.2.rel1-x86_64-arm-none-eabi.tar.xz
cp -r arm-gnu-toolchain-13.2.rel1-x86_64-arm-none-eabi/* /usr
```

And then gcc-arm-none-eabi is installed to your environment.

```bash
git clone https://github.com/Pansamic/nav_mecanum_driver
git submodule init && git submodule update
mkdir build && cd build
# `cmake ..` is set default to debug build type which is -Og -g3 -ggdb optimization.
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j
```

## Flash

Use OpenOCD or st-tools to flash.

For convenience, take OpenOCD and tool with ST-Link for example.

```bash
sudo apt update && sudo apt install -y openocd
cd nav_mecanum_driver/build
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program nav_mecanum_driver.elf verify reset exit"
```

## Debug

Use VSCode and Cortex-Debug extension to debug.

First you need to install 
