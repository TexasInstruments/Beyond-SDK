# Low Power Human-Machine Interface Application

The Low Power Human-Machine Interface application on AM62L demonstrates low
active power on Linux while driving a display.

## Prerequisites
 - [AM62L EVM](https://www.ti.com/tool/TMDS62LEVM)
 - [7-inch Raspberry Pi Touch Display panel](https://www.raspberrypi.com/products/raspberry-pi-touch-display/)
 - 32 GB SD Card

## Directory Setup

The `prebuilt/` directory contains all files needed for the demo to install onto
the SD card. If the `prebuilt/` directory is used, skip to the [Installing the
Demo Files on the SD Card section](#for-prebuilt-files).

All instructions apply to the HOST machine unless otherwise stated.

The following repositories will be used for this demo. The repositories are
cloned and built in later instructions:
 - `arm-trusted-firmware`
 - `linux`
 - `ti-lvgl-demo`
 - `ti-u-boot`
 - `ti-linux-firmware`
 - `optee_os`

The `patches/` directory contains all of the patches that need to be applied on
top of the repository that the individual `.patch` files are in.

Patches are applied to the repository by using the following command:

`git am <patch-file>`

Alternatively, the following repositories can be used which already have the
needed patch files applied. If these are used ignore the applying patches steps:
 - [arm-trusted-firmware on branch am62l-hmi-demo](https://github.com/kwillis01/arm-trusted-firmware/tree/am62l-hmi-demo)
 - [linux on branch am62l-hmi-demo](https://github.com/kwillis01/linux/tree/am62l-hmi-demo)
 - [ti-lvgl-demo on branch hmi-simple-demo](https://github.com/kwillis01/ti-lvgl-demo/tree/hmi-simple-demo)

**NOTE:** The `ti-lvgl-demo` has submodules which are linked git repositories.
Submodules have to be initialized before they can be interacted with. To
initialize the submodules, run the following command in the `ti-lvgl-demo`
directory:

`git submodule update --init --recursive`

## Flash the SD Card

Flash an SD card with the [tisdk-default-image-am62lxx-evm-12.00.00.07.04.rootfs.wic.xz image](https://www.ti.com/tool/download/AM62L-LINUX-SDK/12.00.00.07.04)

## Build the Demo Files

Install the necessary [toolchains](https://software-dl.ti.com/processor-sdk-linux/esd/AM62LX/12_00_00_07_04/exports/docs/linux/Overview/GCC_ToolChain.html#external-arm-toolchain)

For ease of use, create the following environment variables for the paths to
each directory. Users will have to wait to clone the repository before creating
the environment variables.

```
export UBOOT_DIR=<path-to-ti-u-boot>
export TI_LINUX_FW_DIR=<path-to-ti-linux-firmware>
export TFA_DIR=<path-to-arm-trusted-firmware>
export LINUX_DIR=<path-to-linux>
export TI_LVGL_DEMO_DIR=<path-to-ti-lvgl-demo>
export OPTEE_DIR=<path-to-optee_os>
```

### Build Arm Trusted Firmware

Refer to the [ATF build instructions](https://software-dl.ti.com/processor-sdk-linux/esd/AM62LX/12_00_00_07_04/exports/docs/linux/Foundational_Components_ATF.html)
for cloning and setup. Apply the patches in `patches/arm-trusted-firmware/` to
the `arm-trusted-firmware` directory, then build ATF:

```
cd $TFA_DIR
make CROSS_COMPILE="$CROSS_COMPILE_64" ARCH=aarch64 PLAT=k3low TARGET_BOARD=am62lx SPD=opteed
```

### Build OP-TEE

Refer to the [OP-TEE build instructions](https://software-dl.ti.com/processor-sdk-linux/esd/AM62LX/12_00_00_07_04/exports/docs/linux/Foundational_Components_OPTEE.html)
for cloning and setup. Build OP-TEE using the following command:

```
cd $OPTEE_DIR
make CROSS_COMPILE="$CROSS_COMPILE_32" CROSS_COMPILE64="$CROSS_COMPILE_64" ARCH=arm PLATFORM=k3-am62lx CFG_ARM64_core=y
```

The built OP-TEE binary is at `$OPTEE_DIR/out/arm-plat-k3/core/tee-pager_v2.bin`.

### Build U-Boot

Refer to the [U-Boot build instructions](https://software-dl.ti.com/processor-sdk-linux/esd/AM62LX/12_00_00_07_04/exports/docs/linux/Foundational_Components/U-Boot/BG-Build-K3.html)
for cloning and setup. Once ATF and OP-TEE are built, build U-Boot using the
following commands:

```
cd $UBOOT_DIR
make CROSS_COMPILE="$CROSS_COMPILE_64" am62lx_evm_defconfig O=$UBOOT_DIR/out
make CROSS_COMPILE="$CROSS_COMPILE_64" O=$UBOOT_DIR/out BL1=$TFA_DIR/build/k3low/am62lx/release/bl1.bin BL31=$TFA_DIR/build/k3low/am62lx/release/bl31.bin TEE=$OPTEE_DIR/out/arm-plat-k3/core/tee-pager_v2.bin BINMAN_INDIRS=$TI_LINUX_FW_DIR
```

The built images are in the `$UBOOT_DIR/out` directory.
The files are:
 - `tiboot3.bin`
 - `tispl.bin`
 - `u-boot.img`

### Build Linux

Refer to the [Linux kernel build instructions](https://software-dl.ti.com/processor-sdk-linux/esd/AM62LX/12_00_00_07_04/exports/docs/linux/Foundational_Components_Kernel_Users_Guide.html)
for cloning and setup. Apply the patches in `patches/linux/` to the `linux`
directory, then build the Linux Image, modules, and device tree using the
following commands:

```
cd $LINUX_DIR
make ARCH=arm64 CROSS_COMPILE="$CROSS_COMPILE_64" defconfig ti_arm64_prune.config
make ARCH=arm64 CROSS_COMPILE="$CROSS_COMPILE_64" Image modules
make ARCH=arm64 CROSS_COMPILE="$CROSS_COMPILE_64" dtbs
```

The Image file is at this path: `$LINUX_DIR/arch/arm64/boot/Image`
The HMI device tree file is at this path: `$LINUX_DIR/arch/arm64/boot/dts/ti/k3-am62l3-evm-hmi.dtb`
The RPI DSI device tree overlay file is at this path: `$LINUX_DIR/arch/arm64/boot/dts/ti/k3-am62l3-evm-dsi-rpi-7inch-panel.dtbo`

### Build ti-lvgl-demo

Clone the [ti-lvgl-demo repository](https://github.com/TexasInstruments/ti-lvgl-demo)
with this command:

`git clone --recurse-submodules https://github.com/texasinstruments/ti-lvgl-demo.git`

Then checkout [commit 231f5d956c83](https://github.com/TexasInstruments/ti-lvgl-demo/commit/231f5d956c83f934c6b4175870d867a61ae5bc32)
and re-initialize the submodules, since checking out a different commit may
change which submodule commits are referenced:

`git submodule update --init --recursive`

Apply the patches in the `patches/ti-lvgl-demo/` to the repository, and to the
submodule repositories. Re-initialize the submodules to pick up any submodule
changes introduced by the patches:

`git submodule update --init --recursive`

Setup docker by following this [tutorial](https://github.com/TexasInstruments/ti-lvgl-demo#install-docker)

Build ti-lvgl-demo with the following commands:

```
cd $TI_LVGL_DEMO_DIR
./scripts/docker_setup.sh --create-image
./scripts/docker_setup.sh --build-app
```

The built application (lvglsim) is at this path: `$TI_LVGL_DEMO_DIR/lv_port_linux/bin/lvglsim`

## Installing the Demo Files on the SD Card

### For Manually Built Files

Use the following commands to install the needed files on the SD card.

```
cp $UBOOT_DIR/out/tiboot3.bin /media/$USER/boot
cp $UBOOT_DIR/out/tispl.bin /media/$USER/boot
cp $UBOOT_DIR/out/u-boot.img /media/$USER/boot
cp uEnv.txt /media/$USER/boot

sudo cp $LINUX_DIR/arch/arm64/boot/Image /media/$USER/root/boot
sudo cp $LINUX_DIR/arch/arm64/boot/dts/ti/k3-am62l3-evm-hmi.dtb /media/$USER/root/boot/dtb/ti/k3-am62l3-evm.dtb
sudo cp $LINUX_DIR/arch/arm64/boot/dts/ti/k3-am62l3-evm-dsi-rpi-7inch-panel.dtbo /media/$USER/root/boot/dtb/ti/
sudo rm -rf /media/$USER/root/lib/modules/*; sudo make ARCH=arm64 INSTALL_MOD_PATH=/media/$USER/root modules_install

sudo cp wkup_periph_clk_freqs.sh /media/$USER/root/root 
sudo cp hmi-script.sh /media/$USER/root/root
sudo cp $TI_LVGL_DEMO_DIR/lv_port_linux/bin/lvglsim /media/$USER/root/root

sudo mkdir /media/$USER/root/usr/share/ti-lvgl-demo
sudo mkdir /media/$USER/root/usr/share/ti-lvgl-demo/assets
sudo cp $TI_LVGL_DEMO_DIR/lv_port_linux/lvgl/demos/simple_high_res/img_lv_demo_simple_high_res_ti_logo.png /media/$USER/root/usr/share/ti-lvgl-demo/assets
```

### For Prebuilt Files

Flash the SD card with the [tisdk-default-image-am62lxx-evm-12.00.00.07.04.rootfs.wic.xz image](https://www.ti.com/tool/download/AM62L-LINUX-SDK/12.00.00.07.04),
then use the following commands to install the demo files:

```
cp prebuilt/tiboot3.bin /media/$USER/boot
cp prebuilt/tispl.bin /media/$USER/boot
cp prebuilt/u-boot.img /media/$USER/boot
cp uEnv.txt /media/$USER/boot

sudo cp prebuilt/Image /media/$USER/root/boot
sudo cp prebuilt/k3-am62l3-evm-hmi.dtb /media/$USER/root/boot/dtb/ti/k3-am62l3-evm.dtb
sudo cp prebuilt/k3-am62l3-evm-dsi-rpi-7inch-panel.dtbo /media/$USER/root/boot/dtb/ti/
sudo cp -r prebuilt/modules/ /media/$USER/root

sudo cp wkup_periph_clk_freqs.sh /media/$USER/root/root 
sudo cp hmi-script.sh /media/$USER/root/root
sudo cp prebuilt/lvglsim /media/$USER/root/root

sudo mkdir /media/$USER/root/usr/share/ti-lvgl-demo
sudo mkdir /media/$USER/root/usr/share/ti-lvgl-demo/assets
sudo cp prebuilt/img_lv_demo_simple_high_res_ti_logo.png /media/$USER/root/usr/share/ti-lvgl-demo/assets
```

Eject the SD card.

## Run the Demo

1. After building all files and installing them onto the SD card, eject it from the host computer.
2. Insert the SD card into the AM62L EVM.
3. Plug the ribbon cable into the RPI Touch Display. The connector pins should
   be facing away from the display.
4. On the backside of the AM62L EVM, connect the ribbon cable to the DSI_CONN
   port. The connector pins should be facing the white part of the port.
5. From the host computer, plug two micro USB cables into the AM62L UART and
   JTAG ports.
6. Open the [AM62L Power Visualizer](https://dev.ti.com/gallery/view/SitaraMPU/AM62L_Power_Visualizer/ver/1.0.0/)
   and follow the "How to Setup the Tool" instructions.
7. On the host computer, open `/dev/ttyUSB0` for the Linux UART console.
8. Plug in power to the USB-C port on the AM62L EVM.
9. When Linux starts to boot, power on the RPI Touch Display.
   - **Note:** Do not delay powering the display or it will not be detected.
10. Start streaming power measurements on the AM62L Power Visualizer.
11. Once Linux has booted, run the hmi-script.sh on AM62L:

    `./hmi-script.sh`

    The RPI Touch Display should show the current time and TI logo.
12. When shutting down, run `shutdown -h now` on AM62L.

### RPI Touch Display troubleshooting

If a white screen appears and nothing displays, exit the script and run
`shutdown -h now`on AM62L. Once the AM62L EVM shuts down, power cycle it while
leaving the RPI Touch Display connected. Do not disconnect the RPI Touch Display
between boots.

If Linux cannot detect the RPI touchscreen, remove power from both the AM62L
EVM and the RPI Touch Display. Disconnect the ribbon cable from both ends,
reconnect it, then power up as described above.
