#!/bin/sh

########################################################################
# Copyright (c) 2017 Mellanox Technologies.
# Copyright (c) 2017 Oleksandr Shamray <oleksandrs@mellanox.com>
#
# Licensed under the GNU General Public License Version 2
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
#
# Script to update CPLD firmware
#
# Usage:
#    a) Local: /run/initramfs/update_cpld <cpld-firmware-file.svf>
#    b) Remote: sshpass -p "<root-password>" ssh root@<ip> '/run/initramfs/update_cpld <cpld-firmware-file.svf>'
#
# Assumptions: 
#    <cpld-firmware-file.svf> is a SVF firmware file (one for all CPLD's)
#

CPLD_FIRMWARE_BKP=0
CPLD_FIRMWARE_BKP_PATH=/usr/share

# gpio32 and gpio33 - JTAG MUX control pins.
# To enable CPLD JTAG programming via CPU this pins should be:
# gpio32 - low  (output dir)
# gpio33 - high (output dir)

GPIO_PATH=/sys/class/gpio
GPIO_PIN0=gpio32
GPIO_PIN1=gpio33

if [ ! -e /dev/aspeed-jtag ]; then
	echo "Can't find JTAG(/dev/aspeed-jtag) interface"
	exit
fi

if [ ! -f $1 ]; then
	echo "Can't found or not exists SVF firmware file"
	exit
fi

echo "Configure JTAG mux control pins"
if [ ! -d $GPIO_PATH/$GPIO_PIN0 ]; then
	echo 32 > /sys/class/gpio/export
fi
if [ ! -d $GPIO_PATH/$GPIO_PIN1 ]; then
	echo 33 > /sys/class/gpio/export
fi

echo out > $GPIO_PATH/$GPIO_PIN0/direction
echo out > $GPIO_PATH/$GPIO_PIN1/direction
echo 0 > $GPIO_PATH/$GPIO_PIN0/value
echo 1 > $GPIO_PATH/$GPIO_PIN1/value
mlnx_cpldprog -infile $1 -prog /dev/aspeed-jtag

if [ CPLD_FIRMWARE_BKP = 1 ]; then
	#create buckup file
	SVF_FILE=$(basename $1)
	tar cvzf $CPLD_FIRMWARE_BKP_PATH/$SVF_FILE.tar.gz $1 &2>null
fi

echo "Disable JTAG mux control pins"
echo in > $GPIO_PATH/$GPIO_PIN0/direction
echo in > $GPIO_PATH/$GPIO_PIN1/direction

echo "CPLD succesfully updated."
sleep 1


