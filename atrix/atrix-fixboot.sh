#!/bin/bash
# fetch mkbootimg from https://github.com/pbatard/bootimg-tools.git to bootimg-tools and compile.
# also unmkbootimg from existing atrix boot.img to extract ramdisk and keep it at boot.img-ramdisk
ROM=ZKK_V3M
function makekernel() {
(cd .. && rm -f *.tar && 
 sed -i 's@.*CONFIG_MACH_OLYMPUS_HPORCH32.*$@# CONFIG_MACH_OLYMPUS_HPORCH32 is not set@' .config &&
 grep HPORCH32 .config &&
 ARCH=arm CROSS_COMPILE=arm-none-eabi- make -j4 zImage)
 cp ../arch/arm/boot/zImage boot.img-kernel 
 ./bootimg-tools/mkbootimg/mkbootimg --base 0 --pagesize 2048 --kernel_offset 0x10008000 --ramdisk_offset 0x11000000 --second_offset 0x10f00000 --tags_offset 0x10000100 --kernel boot.img-kernel --ramdisk boot.img-ramdisk -o boot.img
 echo "boot.img is ready."
(cd .. && 
 sed -i 's@.*CONFIG_MACH_OLYMPUS_HPORCH32.*$@CONFIG_MACH_OLYMPUS_HPORCH32=y@' .config &&
 grep HPORCH32 .config  &&
 ARCH=arm CROSS_COMPILE=arm-none-eabi- make -j4 tar-pkg)
 cp ../arch/arm/boot/zImage boot.img-kernel 
 ./bootimg-tools/mkbootimg/mkbootimg --base 0 --pagesize 2048 --kernel_offset 0x10008000 --ramdisk_offset 0x11000000 --second_offset 0x10f00000 --tags_offset 0x10000100 --kernel boot.img-kernel --ramdisk boot.img-ramdisk -o boot.img-hporch32
 echo "boot.img-hporch32 is ready"
}

function buildpkg() {
extra=$1
(cd kupd && rm -f boot.img && ln ../boot.img${extra} boot.img)
ZIP=PolesApart-krnl-${ROM}-`date '+%Y%m%d'`${extra}.zip
(cd kupd && rm -f ../${ZIP} && zip -9r ../${ZIP} .)
}
set -e -x
echo "Proceeding. If \"ok\" isn't printed at end, something went wrong."
makekernel
(rm -rf mod && mkdir mod && cd mod && tar xvf ../../*.tar && mv `find -name '*.ko'` .  && tar cvf modules.tar *.ko)
# Update modules.
mkdir -p kupd/system/lib/modules || true
(cd kupd/system/lib/modules && rm -rf *.ko && tar xvf ../../../../mod/modules.tar)
buildpkg
buildpkg "-hporch32"
#scp boot.img root@192.168.25.6:/storage/sdcard1/
echo ok
