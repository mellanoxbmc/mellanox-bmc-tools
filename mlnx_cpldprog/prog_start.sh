#! /bin/sh
GPIO_PATH=/sys/class/gpio/
echo "Set out GPIOE0:0 GPIOE1:1"
if [ ! -d $GPIO_PATH/gpio32 ]; then
	echo 32 > /sys/class/gpio/export
fi
if [ ! -d $GPIO_PATH/gpio33 ]; then
	echo 33 > /sys/class/gpio/export
fi
echo out > /sys/class/gpio/gpio32/direction
echo out > /sys/class/gpio/gpio33/direction
echo 0 > /sys/class/gpio/gpio32/value
echo 1 > /sys/class/gpio/gpio33/value
