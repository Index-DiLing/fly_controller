#include "SDIO/stm324x7i_eval_sdio_sd.h"
#include "Fatfs/diskio.h"
#include "dl_sd_disk.h"
#include "global.h"
#include "dl_log.h"

uint32_t DL_SD_CardBlockSize = 0;
uint64_t DL_SD_CardCapacity  = 0;
uint8_t DL_SD_CardType       = 0;

#ifdef __cplusplus
extern "C" {
#endif
DSTATUS SD_disk_status()
{
    return 0;
}

DSTATUS SD_disk_initialize()
{
    SD_Error stat = SD_Init();

    SD_CardInfo info;
    SD_GetCardInfo(&info);
    DL_SD_CardBlockSize = info.CardBlockSize;
    DL_SD_CardCapacity  = info.CardCapacity;
    DL_SD_CardType      = info.CardType;
    if (stat == SD_OK) {
        return 0;
    }
    return STA_NOINIT;
}

DRESULT SD_disk_read(
    BYTE *buff,   /* Data buffer to store read data */
    LBA_t sector, /* Start sector in LBA */
    UINT count /* Number of sectors to read */)
{
    // SD_Error err = SD_ReadMultiBlocks(buff, sector, DL_SD_CardBlockSize, count);
    // SD_WaitReadOperation();
    // uint16_t cnt = 0xffff;
    // while(SD_GetStatus() != SD_TRANSFER_OK);
    // if(cnt == 0){
    //     return RES_ERROR;
    // }
    // if (err == SD_OK) {
    //     return RES_OK;
    // }
    // if (err == SD_INVALID_PARAMETER) {
    //     return RES_PARERR;
    // }
    // return RES_ERROR;
    DRESULT status    = RES_PARERR;
    SD_Error SD_state = SD_OK;
    if ((DWORD)buff & 3) {
        DRESULT res = RES_OK;
        DWORD scratch[DL_SD_CardBlockSize / 4];

        while (count--) {
            res = SD_disk_read((BYTE *)scratch, sector++, 1);
            if (res != RES_OK) {
                break;
            }
            memcpy(buff, scratch, DL_SD_CardBlockSize);
            buff += DL_SD_CardBlockSize;
        }
        return res;
    }
    SD_state = SD_ReadMultiBlocks(buff, sector * DL_SD_CardBlockSize,
                                  DL_SD_CardBlockSize, count);
    if (SD_state == SD_OK) {
        /* Check if the Transfer is finished */
        SD_state = SD_WaitReadOperation();
        while (SD_GetStatus() != SD_TRANSFER_OK);
    }
    if (SD_state != SD_OK)
        status = RES_PARERR;
    else
        status = RES_OK;
    return status;
}
//2026.3.10 发现并修复BUG
DRESULT SD_disk_write(
    const BYTE *buff, /* Data to be written */
    LBA_t sector,     /* Start sector in LBA */
    UINT count /* Number of sectors to write */)
{
    // logger << "Writing to" << sector << "CNT:" << count << LCMD::NFLUSH;
    // SD_Error err = SD_WriteMultiBlocks((uint8_t *)buff, sector, DL_SD_CardBlockSize, count);
    // SD_WaitWriteOperation();
    // uint16_t cnt = 0xffff;
    // while (SD_GetStatus() != SD_TRANSFER_OK);

    // if (cnt == 0) {
    //     return RES_ERROR;
    // }

    // if (err == SD_OK) {
    //     return RES_OK;
    // }
    // if (err == SD_INVALID_PARAMETER) {
    //     return RES_PARERR;
    // }
    // return RES_ERROR;
    DRESULT status    = RES_PARERR;
    SD_Error SD_state = SD_OK;

    if (!count) {
        return RES_PARERR; /* Check parameter */
    }
    if ((DWORD)buff & 3) {
        DRESULT res = RES_OK;
        DWORD scratch[DL_SD_CardBlockSize / 4];
        while (count--) {
            memcpy(scratch, buff, DL_SD_CardBlockSize);
            res = SD_disk_write((BYTE *)scratch, sector++, 1);
            if (res != RES_OK) {
                break;
            }
            buff += DL_SD_CardBlockSize;
        }
        return res;
    }

    SD_state = SD_WriteMultiBlocks((uint8_t *)buff, sector * DL_SD_CardBlockSize,
                                   DL_SD_CardBlockSize, count);
    if (SD_state == SD_OK) {
        /* Check if the Transfer is finished */
        SD_state = SD_WaitWriteOperation();
        /* Wait until end of DMA transfer */
        while (SD_GetStatus() != SD_TRANSFER_OK);
    }
    if (SD_state != SD_OK)
        status = RES_PARERR;
    else
        status = RES_OK;
    return status;
}

DRESULT SD_disk_ioctl(
    BYTE cmd,  /* Control code */
    void *buff /* Buffer to send/receive control data */
)
{
    switch (cmd) {
        // Get R/W sector size (WORD)
        case GET_SECTOR_SIZE:
            *(WORD *)buff = DL_SD_CardBlockSize;
            break;
        // Get erase block size in unit of sector (DWORD)
        case GET_BLOCK_SIZE:
            *(DWORD *)buff = DL_SD_CardBlockSize;
            break;
        case GET_SECTOR_COUNT:
            *(DWORD *)buff = DL_SD_CardCapacity / DL_SD_CardBlockSize;
            break;
        case CTRL_SYNC:
            break;
    }

    return RES_OK;
}
DWORD get_fattime(void)
{
    return SystemClockMilliseconds + 100000;
}

#ifdef __cplusplus
}
#endif