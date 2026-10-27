#!/bin/sh
SCRIPT_DIR=`dirname $0`
ROS_PKG=`basename $SCRIPT_DIR`

FW_DIR=NONE
IFS=:
for p in $COLCON_PREFIX_PATH 
do
    b=`dirname $p`  
    
    if [ -d ${b}/firmware ]
    then
     echo FOUND FIRMWARE
     FW_DIR="${b}/firmware"
     WS_DIR="${b}"
    fi
    b=`dirname $b`  
    if [ -d ${b}/firmware ]
    then
     echo FOUND FIRMWARE
     FW_DIR="${b}/firmware"
     WS_DIR="${b}"
    fi
done

if [ "${FW_DIR}" = "NONE" ]
then
    echo FAILED to FIND firmware folder
    exit 1
fi

PKG_DIR=${WS_DIR}/src/${ROS_PKG}

echo ROS WORKSPACE: $WS_DIR
echo PKG NAME: $ROS_PKG
echo PKG DIR: $PKG_DIR
echo FIRMWARE: $FW_DIR

echo Building Firmware Structure for $ROS_PKG
TARGET="${FW_DIR}/mcu_ws/${ROS_PKG}"
mkdir -p "${TARGET}"
# srv/ and include/ are optional, only copy what the package actually has
for d in msg srv include scripts
do
    if [ -d "${PKG_DIR}/${d}" ]
    then
        cp -r "${PKG_DIR}/${d}" "${TARGET}"
    fi
done
cp "${PKG_DIR}/CMakeLists.txt" "${TARGET}/"
cp "${PKG_DIR}/package.xml" "${TARGET}/"
