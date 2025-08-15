## AM243x FSI Loopback

### HARDWARE:
- 2x AM243x EVM (TMDS243EVM - https://www.ti.com/tool/TMDS243EVM)
- 2x TMDSFSIADAPEVM Daughter cards (https://www.ti.com/lit/ug/swru555/swru555.pdf)
- FSI Interface (https://www.ti.com/lit/ug/spruj63a/spruj63a.pdf J5 or J7 depending on the EVM revision)
  
  <!-- Center aligned image -->
  <div align="center">
    <img src="fsi_connector_am243x_evm.png" alt="FSI connector pin-out" width="500">
  </div>

- Connect each AM243x EVM with a "Fast Serial Interface (FSI) adapter board evaluation module" daughter card using ribbon cables. Follow the above pin-out connection.
- Use two Ethernet cables to connect between daughter cards (TMDSFSIADAPEVM)
  - Note: Between daughter cards, connect LVDS RX to LVDS TX
#### SETUP:
<!-- Center aligned image - make sure the image file exists in the same directory -->
<div align="center">
  <img src="AM243x_EVM_FSI_setup_ribbonCable.png" alt="FSI_AM243x_EVM_Setup" width="800">
</div>

### SOFTWARE:
- Install CCS
- Install AM243x MCU+SDK >9.2
    - Tested with MCU + SDK v10.1.0.32 and SysConfig v1.21.2
- Load "fsi_loopback_interrupt_d1" application in CCS and rebuild
    - For testing, use CCS JTAG to load the fsi_loopback_interrupt_d1 image. Alternatively, you can flash the image using the OSPI script (tools\boot\sbl_prebuilt\am243x-evm\default_sbl_ospi.cfg)
- Load "fsi_loopback_interrupt_d2" application in CCS and rebuild it
    - For testing, use CCS JTAG to load the fsi_loopback_interrupt_d2 image. Alternatively, you can flash the image using the OSPI script (tools\boot\sbl_prebuilt\am243x-evm\default_sbl_ospi.cfg)

#### FSI LEAD (D1)
##### Application fsi_loopback_interrupt_d1
Below is a brief explanation of fsi_loopback_interrupt_d1:
- Configure I2C IO expander for FSI connection
- Initialize FSI
- Lead (D1) starts FSI handshake with node (D2)
- After handshake, FSI data frames are sent in loopback to node (D2)

#### FSI Node (D2)
##### Application fsi_loopback_interrupt_d2
Below is a brief explanation of fsi_loopback_interrupt_d2:
- Configure I2C IO expander for FSI connection
- Initialize FSI
- Perform the FSI handshake with D1
- After handshake is received, D2 starts receiving and sending back data frames to lead (D1)

### CONSOLE OUTPUT D1 example:
```
Starting NULL Bootloader ...

DMSC Firmware Version 9.2.7--v09.02.07 (Kool Koala)
DMSC Firmware revision 0x9
DMSC ABI revision 3.1

INFO: Bootloader_runCpu:155: CPU r5f1-0  is initialized to 800000000 Hz !!!
INFO: Bootloader_runCpu:155: CPU r5f1-1 is initialized to 800000000 Hz !!!
INFO: Bootloader_runCpu:155: CPU m4f0-0 is initialized to 400000000 Hz !!!
INFO: Bootloader_loadSelfCpu:207: CPU r5f0-0 is initialized to 800000000 Hz !!!
INFO: Bootloader_loadSelfCpu:207: CPU r5f0-1 is initialized to 800000000 Hz !!!
INFO: Bootloader_runSelfCpu:217: All done, reseting self ...

[FSI] Lead - Loopback Interrupt application started at 1000000Hz ...
[FSI] Lead - starting handshake
[FSI] Lead - Handshake done
Lead RX/TX starting at Loopcnt 99
Lead RX/TX starting at Loopcnt 98
........
Lead RX/TX starting at Loopcnt 1
Lead RX/TX starting at Loopcnt 0
[FSI] 100 frames successfully received!!!
All tests have passed!!
```

### CONSOLE OUTPUT D2 example:
```
DMSC Firmware Version 10.0.8--v10.00.08 (Fiery Fox)
DMSC Firmware revision 0xa
DMSC ABI revision 4.0

FSI started sending image application...
FSI Node - Loopback Interrupt application started at 1000000Hz...
FSI Node - initFSI
FSI Node - Starting handshake
FSI Node - Handshake done
Node RX/TX finished, 99 loopcnt
Node RX/TX finished, 98 loopcnt
........
Node RX/TX finished, 1 loopcnt
Node RX/TX finished, 0 loopcnt
[FSI] 100 frames successfully received!!!
All tests have passed!!
```