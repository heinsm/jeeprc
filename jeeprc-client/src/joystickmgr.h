#ifndef JOYSTICKMGR_H_
#define JOYSTICKMGR_H_

#include "multiqueue.h"

typedef struct
{
    char  msg[1024];
} joystick_msg_t;


int joystickmgr_init();
int joystickmgr_fini();
int joystickmgr_getqueue(multiqueue_t** q);

#endif
