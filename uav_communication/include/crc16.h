    
#ifndef _CRC_CRC16_H
#define _CRC_CRC16_H

#include "uavtype.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus  */

#define CRC16_BYTES 2

UINT16 crc16(const char *buf, size_t len);

#ifdef __cplusplus
}
#endif /* __cplusplus  */
#endif