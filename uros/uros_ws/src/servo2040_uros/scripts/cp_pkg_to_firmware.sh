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
if [ ! -d ${TARGET} ]
then
	mkdir ${TARGET}
	mkdir ${TARGET}/include
	mkdir ${TARGET}/msg
	mkdir ${TARGET}/srv
	mkdir ${TARGET}/scripts
fi
cp -r ${PKG_DIR}/msg ${TARGET} 
cp -r ${PKG_DIR}/srv ${TARGET}
cp -r ${PKG_DIR}/include ${TARGET} 
cp -r ${PKG_DIR}/scripts ${TARGET} 
cp ${PKG_DIR}/CMakeLists.txt ${TARGET}/
cp ${PKG_DIR}/package.xml ${TARGET}/
