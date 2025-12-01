# Low Power Human-Machine Interface Application

The Low Power Human-Machine Interface application on AM62L demonstrates low active power on Linux while driving a display.

## Prerequisites
 - [AM62L EVM](https://www.ti.com/tool/TMDS62LEVM)
 - [7-inch Raspberry Pi Touch Display panel](https://www.raspberrypi.com/products/raspberry-pi-touch-display/)
 - 16 gB SD Card

## Directory Setup

This directory contains all the files needed to build the demo. If the user does not want to build the files for the demo, the prebuilt directory contains all files needed for the demo to install onto the SD card.

Submodules are linked git repositories. The following directories are submodules:
 - `arm-trusted-firmware`
 - `linux`
 - `ti-lvgl-demo`
 - `ti-u-boot`
 - `ti-linux-firmware`

The submodules `arm-trusted-firmware`, `linux`, and `ti-lvgl-demo` have additional patches applied within them that allow this demo to work. The `ti-lvgl-demo` has additional submodules inside of the repository, each of those submodules also have additional patches applied within them. To see each patch without looking at the log of each submodule, the patches directory has them in patch file format.

The submodules have to be initialized before they can be interacted with. To initialize the submodules, run the following command in this directory:

`git submodule update --init --recursive`

## Flash the SD card

Flash an sd card with the [tisdk-default-image-am62lxx-evm-11.00.15.05.rootfs.wic.xz image](https://www.ti.com/tool/download/AM62L-LINUX-SDK/11.00.15.05)

## Build the demo files

Install the necessary [toolchains](https://software-dl.ti.com/processor-sdk-linux/esd/AM62LX/11_00_15_05/exports/docs/linux/Overview/GCC_ToolChain.html#external-arm-toolchain)


For ease of use, create the following environment variables for the paths to each directory. All of these will be within this directory.

```
export UBOOT_DIR=<path-to-ti-u-boot>
export TI_LINUX_FW_DIR=<path-to-ti-linux-firmware>
export TFA_DIR=<path-to-arm-trusted-firmware>
export LINUX_DIR=<path-to-linux>
export TI_LVGL_DEMO_DIR=<path-to-ti-lvgl-demo>
```

### Build Arm Trusted Firmware

Enter the `arm-trusted-firmware` directory, then enter the following command to build Arm Trusted Firmware (ATF)

```
cd $TFA_DIR
make ARCH=aarch64 CROSS_COMPILE="$CROSS_COMPILE_64" PLAT=k3 TARGET_BOARD=am62l
```

### Build U-Boot

Once ATF is built, build U-Boot using the following commands:

```
cd $UBOOT_DIR
make CROSS_COMPILE="$CROSS_COMPILE_64" am62lx_evm_defconfig O=$UBOOT_DIR/out
make CROSS_COMPILE="$CROSS_COMPILE_64" O=$UBOOT_DIR/out BL1=$TFA_DIR/build/k3/am62l/release/bl1.bin BL31=$TFA_DIR/build/k3/am62l/release/bl31.bin BINMAN_INDIRS=$TI_LINUX_FW_DIR
```

The built images are in the `$UBOOT_DIR/out` direcotry.
The files are:
 - `tiboot3.bin`
 - `tispl.bin`
 - `u-boot.img`

### Build Linux

Build the Linux Image, modules, and device tree using the following commands:

```
cd $LINUX_DIR
make ARCH=arm64 CROSS_COMPILE="$CROSS_COMPILE_64" defconfig ti_arm64_prune.config
make ARCH=arm64 CROSS_COMPILE="$CROSS_COMPILE_64" Image modules
make ARCH=arm64 CROSS_COMPILE="$CROSS_COMPILE_64" dtbs
```

The Image file is at this path: `$LINUX_DIR/arch/arm64/boot/Image`
The k3-am62l3-evm.dtb file is at this path: `$LINUX_DIR/arch/arm64/boot/dts/ti/k3-am62l3-evm.dtb`

### Build ti-lvgl-demo

Setup docker by following this [tutorial](https://github.com/kwillis01/ti-lvgl-demo?tab=readme-ov-file#install-docker)

Build ti-lvgl-demo with the following commands:

```
cd $TI_LVGL_DEMO_DIR
./scripts/docker_setup.sh --create-image
./scripts/docker_setup.sh --build-app
```

The built application (lvglsim) is at this path: `$TI_LVGL_DEMO_DIR/lv_port_linux/bin/lvglsim`

## Install the demo files on the SD card

### For Manually Built Files

Use the following commands to install the needed files on the SD card.

```
cp $UBOOT_DIR/out/tiboot3.bin /media/<whoami>/boot
cp $UBOOT_DIR/out/tispl.bin /media/<whoami>/boot
cp $UBOOT_DIR/out/u-boot.img /media/<whoami>/boot
cp uEnv.txt /media/<whoami>/boot

sudo cp $LINUX_DIR/arch/arm64/boot/Image /media/<whoami>/root/boot
sudo cp $LINUX_DIR/arch/arm64/boot/dts/ti/k3-am62l3-evm.dtb /media/<whoami>/root/boot/dtb/ti
sudo rm -rf /media/a0506412/root/lib/modules/*; sudo make ARCH=arm64 INSTALL_MOD_PATH=/media/<whoami>/root modules_install

sudo cp hmi-script.sh /media/<whoami>/root/root
sudo cp $TI_LVGL_DEMO_DIR/lv_port_linux/bin/lvglsim /media/<whoami>/root/root

sudo mkdir /media/<whoami>/root/usr/share/ti-lvgl-demo
sudo mkdir /media/<whoami>/root/usr/share/ti-lvgl-demo/assets
sudo cp $TI_LVGL_DEMO_DIR/lv_port_linux/lvgl/demos/simple_high_res/img_lv_demo_simple_high_res_ti_logo.png /media/<whoami>/root/usr/share/ti-lvgl-demo/assets
```

### For Prebuilt Files

Use the following commands to install the needed files on the SD card.

```
cp prebuilt/tiboot3.bin /media/<whoami>/boot
cp prebuilt/tispl.bin /media/<whoami>/boot
cp prebuilt/u-boot.img /media/<whoami>/boot
cp uEnv.txt /media/<whoami>/boot

sudo cp prebuilt/Image /media/<whoami>/root/boot
sudo cp prebuilt/k3-am62l3-evm.dtb /media/<whoami>/root/boot/dtb/ti
sudo cp -r prebuilt/modules/ /media/<whoami>/root

sudo cp hmi-script.sh /media/<whoami>/root/root
sudo cp prebuilt/lvglsim /media/<whoami>/root/root

sudo mkdir /media/<whoami>/root/usr/share/ti-lvgl-demo
sudo mkdir /media/<whoami>/root/usr/share/ti-lvgl-demo/assets
sudo cp $TI_LVGL_DEMO_DIR/lv_port_linux/lvgl/demos/simple_high_res/img_lv_demo_simple_high_res_ti_logo.png /media/<whoami>/root/usr/share/ti-lvgl-demo/assets
```

Eject the SD card.

## Run the Demo

Without the AM62L EVM or the 7-inch Raspberry Pi (RPI) Touch Display panel powered on, connect the RPI display panel to the AM62L EVM with the ribbon. The ribbon connects to the DSI_CONN port which is underneath the AM62L EVM.

Connect the AM62L UART port to the HOST computer. Refer to this [guide](https://dev.ti.com/tirex/content/tirex-product-tree/processors-devtools/am62lx_evm_quick_start_guide.html#establishing-a-serial-connection-with-the-evm) on how to establish a serial connection with the AM62L EVM.

To use the ["AM62L Power Visualizer" Application](https://dev.ti.com/gallery/view/SitaraMPU/AM62L_Power_Visualizer/ver/1.0.0/) to track the power usage during the demo, refer to "How To Setup the Tool" page in the "AM62L Power Visualizer" Application.

Insert the SD card into the AM62L EVM. Power on the AM62L EVM, then power on the RPI display panel.

Once Linux boots, execute the hmi-script.sh script on the AM62L EVM with the following command:

`./hmi-script.sh`

The RPI display panel should show the current time along with a TI logo.