/**
 * @file ci_assert.h
 * @brief assert
 * @version 1.0
 * @date 2019-02-21
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */
#ifndef _CI_ASSERT_H_
#define _CI_ASSERT_H_

#include <stdio.h>
#include "ci_log.h"

/* mprintf is now defined in ci_log.h via _mprintf; avoid redefining it here
   to prevent circular-include redefinition ordering issues under LTO. */

#define CI_ASSERT(x,msg)                                                                                                    \
    if( ( x ) == 0 )                                                                                                        \
    {                                                                                                                       \
        mprintf("%s",msg);                                                                                                   \
        mprintf("ASSERT: %d in %s\n",__LINE__,__FUNCTION__);                                                                \
        while(1)  asm volatile ("ebreak");                                                                                  \
    }

#endif /* _CI_ASSERT_H_ */
