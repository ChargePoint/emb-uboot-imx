#! /bin/sh
source ../toolchain/environment-setup-aarch64-poky-linux
cores=`cat /proc/cpuinfo | grep processor | wc -l`
threads=`expr $cores + 2`
export ARCH=arm64
mkdir -p build
make O=build mrproper
make O=build distclean
make O=build clean
make O=build imx8dxp_ucb_defconfig
#make O=build ARCH=arm mx6dlsabresd_pbc_defconfig
make O=build -j $threads
