# ENET FOTA example

## Steps to run the ENET FOTA example

1. The SBL_OSPI_ENET_FOTA example is used with this ENET FOTA example.
2. The enet_fota_file_transfer.py file is used to transfer the new application image from the Host to the EVM. 
3. The enet_fota_app.cfg file is used to add the location of the new application image that will be used by the enet_fota_file_transfer.py file.

4. Apply the 2 patches present in the patches folder before running the ENET FOTA example.
5. After applying the patches, rebuild all the libraries and then build the sbl_ospi_enet_fota and the enet_fota example.

6. After flashing the example images, power ON the EVM and run the enet_fota__file_transfer.py file.

## Notes

- The ENET FOTA example is compatible with the 12.00 version of the MCU+ SDK.
- All the 2 patches are required for the Ethernet FOTA example to work properly.
