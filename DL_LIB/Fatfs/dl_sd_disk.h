
#include "ff.h"


#ifdef __cplusplus
extern "C" {
#endif
DSTATUS SD_disk_status();

DSTATUS SD_disk_initialize();

DRESULT SD_disk_read(
    BYTE *buff,   /* Data buffer to store read data */
    LBA_t sector, /* Start sector in LBA */
    UINT count /* Number of sectors to read */);

DRESULT SD_disk_write(
    const BYTE *buff, /* Data to be written */
    LBA_t sector,     /* Start sector in LBA */
    UINT count /* Number of sectors to write */);

DRESULT SD_disk_ioctl(
    BYTE cmd,  /* Control code */
    void *buff /* Buffer to send/receive control data */
);
#ifdef __cplusplus
}
#endif