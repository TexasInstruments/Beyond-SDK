#!/bin/bash

# Slow A53 to 200 MHz
echo userspace > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
echo 200000 > /sys/devices/system/cpu/cpu0/cpufreq/scaling_setspeed

# Disable A53 core 1
echo 0 > /sys/devices/system/cpu/cpu1/online
 
sleep .5

# Slow all clocks in WKUP domain
./wkup_periph_clk_freqs.sh
 
sleep .5

systemctl stop emptty
sleep .5
#Exectute Display Demo
./lvglsim
