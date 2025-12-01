#!/bin/bash
  
#auto clock gating
devmem2 0x43054050 w 0x0
  
sleep .5
  
#PER Sys Clock to less than 20MHz
devmem2 0x04040080 w 0x0000807F
  
sleep .5
  
#Disable pbist0 LPSC
devmem2 0x00400A08 w 0x00000100
   
#Disable main_gp_test LPSC
devmem2 0x00400A04 w 0x00000100
     
#Disable main_gp_wkperi LPSC
devmem2 0x00400A48 w 0x00000100
   
#go command register to push LPSCs into next state
devmem2 0x00400120 w 0xFFFFFFFF
  
sleep .5
  
# Enable Aggressive DDR Auto Refresh
devmem2 0x0F3082A0 w 0x00000f07
devmem2 0x0F3082A4 w 0x0303000f
devmem2 0x0F30829C w 0x07074007  # enable active and long self refresh w. mem gate
  
sleep .5
  
#Disable DPI enable bit
devmem2 0x3020A004 w 0x00000301
  
sleep .5
 
# Disable A53 core 1
echo 0 > /sys/devices/system/cpu/cpu1/online
 
sleep .5
 
systemctl stop emptty
sleep 1
#Exectute Display Demo
./lvglsim
