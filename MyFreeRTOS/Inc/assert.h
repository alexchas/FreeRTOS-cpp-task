/*******************************************************************************
*  FILENAME: assert.h
*
*  DESCRIPTION:
*
*  Created on: 21 мая 2018 г.
*      Author: Alexander Chashkin
******************************************************************************/
#ifndef ASSERT_H
#define ASSERT_H

#include "types.h"

#ifdef   __cplusplus
extern "C" {
#endif

#ifdef DEBUG

#pragma inline=forced
inline void assertBreak(void)
{
  asm("BREAK");
}

#define ASSERT(e)    ((e) ? (void)0 : assertBreak())

#define ALLEGE(e, val)  ASSERT(e == val)
#define NALLEGE(e, val)  ASSERT(e != val)

#else /* !DEBUG */
/* If not debugging, ASSERT does nothing.  */
#define ASSERT(e)    ((void)0)

#define ALLEGE(e, val)  (void) e
#define NALLEGE(e, val) (void) e

#endif   /* DEBUG */

#ifdef   __cplusplus
}
#endif

#endif // ASSERT_H
