#ifndef __CRCTHINGS__
#define __CRCTHINGS__
#include "smart_car_public.h"

//CRC 校验相关函数




uint32 reflect (uint32 crc, int bitnum);

void generate_crc_table();


uint32 crctablefast (uint8* p, uint32 len);


uint32 crctable (uint8* p, uint32 len);



uint32 crcbitbybit(uint8* p, uint32 len);

uint32 crcbitbybitfast(uint8* p, uint32 len);

//crc32实现函数
uint32 makeFrameCheck( uint8 *buf, int len);

#endif //CRCTHINGS_H