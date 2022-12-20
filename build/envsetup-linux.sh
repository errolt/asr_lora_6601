#/bin/sh

export TREMO_SDK_PATH=$(pwd)

GNU_ARM_EMBEDDED_TOOLCHAIN_URL='https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2020q2/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2'

which arm-none-eabi-gcc >/dev/null 2>&1
if [ $? -ne 0 ]; then
    if [ ! -e $TREMO_SDK_PATH/tools/toolchain/bin/arm-none-eabi-gcc ]; then
        if [ ! -e $TREMO_SDK_PATH/tools/toolchain/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2 ]; then
            wget $GNU_ARM_EMBEDDED_TOOLCHAIN_URL -O $TREMO_SDK_PATH/tools/toolchain/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2
        fi
    	cd ./tools/toolchain/
    	tar -xf gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2
        mv gcc-arm-none-eabi-9-2020-q2-update/* .
        rm -rf gcc-arm-none-eabi-9-2020-q2-update
    	cd -
    fi
    
    which arm-none-eabi-gcc >/dev/null 2>&1
    if [ $? -ne 0 ]; then
        export PATH="$PATH:$TREMO_SDK_PATH/tools/toolchain/bin"
    fi
fi
