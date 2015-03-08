#include "joystickmgr.h"
#include "util.h"

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include <pthread.h>
#include <stdbool.h>
#include <errno.h>

#define DEADBAND_VALUE              10000
#define JOY_DEV                     "/dev/input/js0"
#define JOY_ALIVE_TEST_INTERVAL     2

static pthread_t pid_joy = -1;
static volatile bool joy_shutdown = false;
static volatile bool joy_errored = false;
static multiqueue_t joy_q = MULTIQUEUE_INITIALIZER;

static void* joystickmgr_thread_handler(void* a)
{
	int joy_fd, *axis=NULL, num_of_axis=0, num_of_buttons=0, x;
	char *button=NULL, name_of_joystick[80];
	struct js_event js;
	joystick_msg_t msg;
	int result;

	printf( "Testing for joystick...\n" );
	while ( (!joy_shutdown) )
	{
	    joy_errored = false;

        if( ( joy_fd = open( JOY_DEV , O_RDONLY)) == -1 )
        {
            sleep(JOY_ALIVE_TEST_INTERVAL);
            continue;
        }

        ioctl( joy_fd, JSIOCGAXES, &num_of_axis );
        ioctl( joy_fd, JSIOCGBUTTONS, &num_of_buttons );
        ioctl( joy_fd, JSIOCGNAME(80), &name_of_joystick );

        axis = (int *) calloc( num_of_axis, sizeof( int ) );
        button = (char *) calloc( num_of_buttons, sizeof( char ) );

        printf("Joystick detected: %s\n\t%d axis\n\t%d buttons\n\n"
            , name_of_joystick
            , num_of_axis
            , num_of_buttons );

        //fcntl( joy_fd, F_SETFL, O_NONBLOCK );	/* use non-blocking mode */

        while ( (!joy_shutdown) && (!joy_errored) )
        {

            /* read the joystick state */
            result = read(joy_fd, &js, sizeof(struct js_event));

            switch (result)
            {
                case -1:
                {
                    if (errno == EINTR)
                    {
                        continue;
                    }

                    //read problem...
                    printf("joystick read error...\n");
                    printf( "Testing for joystick...\n" );
                    joy_errored = true;

                    //set cmds as if in deadband
                    snprintf(msg.msg, sizeof(joystick_msg_t), "c m 0 0\n");
                    multiqueue_datapush(&joy_q, &msg, sizeof(joystick_msg_t));

                    break;
                }

                default:
                {
                    /* see what to do with the event */
                    switch (js.type & ~JS_EVENT_INIT)
                    {
                        case JS_EVENT_AXIS:
                            axis   [ js.number ] = js.value;
                            break;
                        case JS_EVENT_BUTTON:
                            button [ js.number ] = js.value;
                            break;
                    }

                        /* print the results */
                    printf( "X: %6d  Y: %6d  ", axis[0], axis[1] );

                    if( num_of_axis > 2 )
                        printf("Z: %6d  ", axis[2] );

                    if( num_of_axis > 3 )
                        printf("R: %6d  ", axis[3] );

                    for( x=0 ; x<num_of_buttons ; ++x )
                        printf("B%d: %d  ", x, button[x] );

                    printf("  \r");
                    fflush(stdout);


                    //Y axis (joy1)
                    if (axis[1] > DEADBAND_VALUE)
                    {
                        snprintf(msg.msg, sizeof(joystick_msg_t), "c m 2 %d\n", axis[1]);
                        multiqueue_datapush(&joy_q, &msg, sizeof(joystick_msg_t));
                    }
                    else if (axis[1] < -DEADBAND_VALUE)
                    {
                        snprintf(msg.msg, sizeof(joystick_msg_t), "c m 1 %d\n", axis[1]);
                        multiqueue_datapush(&joy_q, &msg, sizeof(joystick_msg_t));
                    }
                    else
                    {
                        //in deadband
                        snprintf(msg.msg, sizeof(joystick_msg_t), "c m 0 0\n");
                        multiqueue_datapush(&joy_q, &msg, sizeof(joystick_msg_t));
                    }

                    //X axis (joy2)
                    if (axis[2] > DEADBAND_VALUE)
                    {
                        snprintf(msg.msg, sizeof(joystick_msg_t), "c t 1 %d\n", axis[2]);
                        multiqueue_datapush(&joy_q, &msg, sizeof(joystick_msg_t));
                    }
                    else if (axis[2] < -DEADBAND_VALUE)
                    {
                        snprintf(msg.msg, sizeof(joystick_msg_t), "c t 2 %d\n", axis[2]);
                        multiqueue_datapush(&joy_q, &msg, sizeof(joystick_msg_t));
                    }
                    else
                    {
                        //in deadband
                        snprintf(msg.msg, sizeof(joystick_msg_t), "c t 0 0\n");
                        multiqueue_datapush(&joy_q, &msg, sizeof(joystick_msg_t));
                    }

                    break;
                }
            }
        }
	}

	close( joy_fd );
	return NULL;
}

int joystickmgr_init()
{
    multiqueue_init(&joy_q);
    pthread_create(&pid_joy, NULL, joystickmgr_thread_handler, NULL);

    return EOK;
}

int joystickmgr_fini()
{
    joy_shutdown = true;
    pthread_join(pid_joy, NULL);

    multiqueue_fini(&joy_q);

    return EOK;
}

int joystickmgr_getqueue(multiqueue_t** q)
{
    *q = &joy_q;

    return EOK;
}
