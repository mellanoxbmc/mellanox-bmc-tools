#! /bin/sh
GPIO_PATH=/sys/class/gpio/
echo "Set IN GPIOE0, GPIOE1"
if [ ! -d $GPIO_PATH/gpio32 ]; then
        echo 32 > /sys/class/gpio/export
fi
if [ ! -d $GPIO_PATH/gpio33 ]; then
        echo 33 > /sys/class/gpio/export
fi
echo in > /sys/class/gpio/gpio32/direction
echo in > /sys/class/gpio/gpio33/direction
