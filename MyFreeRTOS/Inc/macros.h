/*******************************************************************************
*  FILENAME: macros.h
*
*  DESCRIPTION: Макросы для работы с битами
*
*  Created on: 21 мая 2018 г.
*  Author: Alexander Chashkin
******************************************************************************/

#ifndef MACROS_H
#define MACROS_H

#include "types.h"

#define BIT(n) ((tU32) 1 << (n)) 

#define BIT0 (tU32) 0x00000001
#define BIT1 (tU32) 0x00000002
#define BIT2 (tU32) 0x00000004
#define BIT3 (tU32) 0x00000008
#define BIT4 (tU32) 0x00000010
#define BIT5 (tU32) 0x00000020
#define BIT6 (tU32) 0x00000040
#define BIT7 (tU32) 0x00000080
#define BIT8 (tU32) 0x00000100
#define BIT9 (tU32) 0x00000200
#define BIT10 (tU32) 0x00000400
#define BIT11 (tU32) 0x00000800
#define BIT12 (tU32) 0x00001000
#define BIT13 (tU32) 0x00002000
#define BIT14 (tU32) 0x00004000
#define BIT15 (tU32) 0x00008000
#define BIT16 (tU32) 0x00010000
#define BIT17 (tU32) 0x00020000
#define BIT18 (tU32) 0x00040000
#define BIT19 (tU32) 0x00080000
#define BIT20 (tU32) 0x00100000
#define BIT21 (tU32) 0x00200000
#define BIT22 (tU32) 0x00400000
#define BIT23 (tU32) 0x00800000
#define BIT24 (tU32) 0x01000000
#define BIT25 (tU32) 0x02000000
#define BIT26 (tU32) 0x04000000
#define BIT27 (tU32) 0x08000000
#define BIT28 (tU32) 0x10000000
#define BIT29 (tU32) 0x20000000
#define BIT30 (tU32) 0x40000000
#define BIT31 (tU32) 0x80000000

#define ALLBITS (tU32) 0xFFFFFF

#define BITS_PER_BYTE (tU8)8
#define BITS_PER_WORD (tU8)16

#define SETBIT(DEST,BITMASK) ((DEST) = (DEST) | (BITMASK))
#define CLRBIT(DEST,BITMASK) ((DEST) = (DEST) & ~(BITMASK))
#define CHECK_BIT_SET(DEST, BITMASK) ((DEST) & (BITMASK))
#define CHECK_BITS_SET(DEST, BITMASK) (((DEST) & (BITMASK)) == (BITMASK))
#define EXTRACT_BIT_FIELD(DEST, BITMASK, BITSHIFT_RIGHT) (((DEST) & (BITMASK)) >> (BITSHIFT_RIGHT))
#define TOGGLEBIT(DEST,BITMASK) ((DEST) = (DEST) ^ (BITMASK))

#endif
