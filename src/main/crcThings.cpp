#include "crcThings.h"


const int order = 32;
uint32 polynom = 0x4c11db7;
const int direct = 1;
uint32 crcinit = 0xffffffff;
uint32 crcxor = 0x00000000;
const int refin = 0;
const int refout = 0;

// 'order' [1..32] is the CRC polynom order, counted without the leading '1' bit
// 'polynom' is the CRC polynom without leading '1' bit
// 'direct' [0,1] specifies the kind of algorithm: 1=direct, no augmented zero bits
// 'crcinit' is the initial CRC value belonging to that algorithm
// 'crcxor' is the final XOR value
// 'refin' [0,1] specifies if a data byte is reflected before processing (UART) or not
// 'refout' [0,1] specifies if the CRC will be reflected before XOR

// internal global values:

uint32 crcmask;
uint32 crchighbit;
uint32 crcinit_direct;
uint32 crcinit_nondirect;
uint32 crctab[256];

// subroutines

uint32 reflect (uint32 crc, int bitnum) {

	// reflects the lower 'bitnum' bits of 'crc'

	uint32 i, j=1, crcout=0;

	for (i=(uint32)1<<(bitnum-1); i; i>>=1) {
		if (crc & i) crcout|=j;
		j<<= 1;
	}
	return (crcout);
}



void generate_crc_table() {

	// make CRC lookup table used by table algorithms

	int i, j;
	uint32 bit, crc;

	for (i=0; i<256; i++) {

		crc=(uint32)i;
		if (refin) crc=reflect(crc, 8);
		crc<<= order-8;

		for (j=0; j<8; j++) {

			bit = crc & crchighbit;
			crc<<= 1;
			if (bit) crc^= polynom;
		}			

		if (refin) crc = reflect(crc, order);
		crc&= crcmask;
		crctab[i]= crc;
	}
}


		
uint32 crctablefast (uint8* p, uint32 len) {

	// fast lookup table algorithm without augmented zero bytes, e.g. used in pkzip.
	// only usable with polynom orders of 8, 16, 24 or 32.

	uint32 crc = crcinit_direct;

	if (refin) crc = reflect(crc, order);

	if (!refin) while (len--) crc = (crc << 8) ^ crctab[ ((crc >> (order-8)) & 0xff) ^ *p++];
	else while (len--) crc = (crc >> 8) ^ crctab[ (crc & 0xff) ^ *p++];

	if (refout^refin) crc = reflect(crc, order);
	crc^= crcxor;
	crc&= crcmask;

	return(crc);
}



uint32 crctable (uint8* p, uint32 len) {

	// normal lookup table algorithm with augmented zero bytes.
	// only usable with polynom orders of 8, 16, 24 or 32.

	uint32 crc = crcinit_nondirect;

	if (refin) crc = reflect(crc, order);

	if (!refin) while (len--) crc = ((crc << 8) | *p++) ^ crctab[ (crc >> (order-8))  & 0xff];
	else while (len--) crc = ((crc >> 8) | (*p++ << (order-8))) ^ crctab[ crc & 0xff];

	if (!refin) while (++len < order/8) crc = (crc << 8) ^ crctab[ (crc >> (order-8))  & 0xff];
	else while (++len < order/8) crc = (crc >> 8) ^ crctab[crc & 0xff];

	if (refout^refin) crc = reflect(crc, order);
	crc^= crcxor;
	crc&= crcmask;

	return(crc);
}



uint32 crcbitbybit(uint8* p, uint32 len) {

	// bit by bit algorithm with augmented zero bytes.
	// does not use lookup table, suited for polynom orders between 1...32.

	uint32 i, j, c, bit;
	uint32 crc = crcinit_nondirect;

	for (i=0; i<len; i++) {

		c = (uint32)*p++;
		if (refin) c = reflect(c, 8);

		for (j=0x80; j; j>>=1) {

			bit = crc & crchighbit;
			crc<<= 1;
			if (c & j) crc|= 1;
			if (bit) crc^= polynom;
		}
	}	

	for (i=0; i<order; i++) {

		bit = crc & crchighbit;
		crc<<= 1;
		if (bit) crc^= polynom;
	}

	if (refout) crc=reflect(crc, order);
	crc^= crcxor;
	crc&= crcmask;

	return(crc);
}



uint32 crcbitbybitfast(uint8* p, uint32 len) {

	// fast bit by bit algorithm without augmented zero bytes.
	// does not use lookup table, suited for polynom orders between 1...32.

	uint32 i, j, c, bit;
	uint32 crc = crcinit_direct;

	for (i=0; i<len; i++) {

		c = (uint32)*p++;
		if (refin) c = reflect(c, 8);

		for (j=0x80; j; j>>=1) {

			bit = crc & crchighbit;
			crc<<= 1;
			if (c & j) bit^= crchighbit;
			if (bit) crc^= polynom;
		}
	}	

	if (refout) crc=reflect(crc, order);
	crc^= crcxor;
	crc&= crcmask;

	return(crc);
}



//crc32校验函数
uint32 makeFrameCheck( uint8 *buf, int len)
{
	//高位补零，高低位替换
	uint8 buff[2500] ;//= {0x00};
	uint8 temp[2500] ;//=  {0x00}; //2500 modify by andy 20210606
	memset(buff,0,sizeof(buff));
	memset(temp,0,sizeof(temp));
	
	memcpy(temp,buf,len);
	if (len%4 != 0)
	{
		int ys = len%4;
		for (int j = 0;j<4 - ys;++j)
		{
			temp[len+j] = 0x00;
		}

		len += 4-ys;
	}
	for (int ii = 0; ii< (len/4);++ii)
	{
		buff[ii*4+0] = temp[ii*4+3];
		buff[ii*4+1] = temp[ii*4+2];
		buff[ii*4+2] = temp[ii*4+1];
		buff[ii*4+3] = temp[ii*4+0];
	}

	int i;
	uint32 bit, crc;

	crcmask = ((((uint32)1<<(order-1))-1)<<1)|1;
	crchighbit = (uint32)1<<(order-1);

	
	if (!direct) 
	{
		crcinit_nondirect = crcinit;
		crc = crcinit;
		for (i=0; i<order; i++) {
			
			bit = crc & crchighbit;
			crc<<= 1;
			if (bit) crc^= polynom;
		}
		crc&= crcmask;
		crcinit_direct = crc;
	}
	else 
	{	
		crcinit_direct = crcinit;
		crc = crcinit;
		for (i=0; i<order; i++) {
			
			bit = crc & 1;
			if (bit) crc^= polynom;
			crc >>= 1;
			if (bit) crc|= crchighbit;
		}	
		crcinit_nondirect = crc;
	}
	//crcbitbybitfast、crcbitbybit、crctablefast、crctable select anyone to make crc32 check!
    return crcbitbybitfast(buff,len);  
}