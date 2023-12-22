# nav_mecanum_driver

This is code for nav_mecanum driver board. This branch uses bare-mental Micro-XRCE-DDS-Client to communicate with ROS2.


## Prerequisite

### ARM toolchain

Use gcc-arm-none-eabi to compile this repo.

Download in [gcc-arm-none-eabi official website](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)

It's recommended to install 13.2.1 or latest.

### Debug tool

You can use different tools to debug code, take **OpenOCD** for example.

Doenload lastest OpenOCD in [OpenOCD download page](https://github.com/xpack-dev-tools/openocd-xpack/releases)

## Compile

```bash
git clone -b develop_xrcedds https://github.com/Pansamic/nav_mecanum_driver
git submodule init && git submodule update
cd nav_mecanum_driver
mkdir build && cd build

# Use -DCMAKE_BUILD_TYPE=Debug if you want to debug program.
cmake -DCMAKE_BUILD_TYPE=Release ..

make -j
```


## Flash

You can use many tools to flash program into MCU, take OpenOCD and ST-Link for example.

```bash
# To flash .elf program
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/nav_mecanum_driver.elf verify reset exit"

# To flash .bin program
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/nav_mecanum_driver.bin verify reset exit 0x08000000"
```

## Debug

Use VSCode and its Cortex-Debug extension.

make a directory under this repo called `.vscode` and make files `launch.json`and `setting.json`.

`launch.json`:

```json
{
"configurations": [
    {
        "cwd": "${workspaceRoot}",
        "executable": "${workspaceFolder}/build/nav_mecanum_driver.elf",
        "name": "Cortex-Debug",
        "request": "launch",
        "type": "cortex-debug",
        "servertype": "openocd",
        "interface": "swd",
        "configFiles": [
            "interface/stlink.cfg",
            "target/stm32f4x.cfg",],
        "searchDir": [],
        "runToEntryPoint": "main",
        "showDevDebugOutput": "raw"
    }
]
}
```

`settings.json`:

```json
{
    "cortex-debug.armToolchainPath": "",
    "cortex-debug.gdbPath": "arm-none-eabi-gdb",
    "cortex-debug.openocdPath": "openocd",
    "cortex-debug.variableUseNaturalFormat": false,
    "files.associations": {
        "cpu_utils.h": "c",
        "main.h": "c"
    },
}
```