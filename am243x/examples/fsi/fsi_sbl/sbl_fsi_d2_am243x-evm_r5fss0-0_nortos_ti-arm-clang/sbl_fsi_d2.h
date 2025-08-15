#ifndef SBL_FSI_D2_H
#define SBL_FSI_D2_H

#include <stdint.h>

#define BOOTLOADER_APPIMAGE_MAX_FILE_SIZE   (0x20000U)
extern uint8_t gAppImageBuf[BOOTLOADER_APPIMAGE_MAX_FILE_SIZE];

#endif // SBL_FSI_D2_H