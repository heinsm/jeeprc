#ifndef MULTIQUEUE_H_
#define MULTIQUEUE_H_

#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>

#define MULTIQUEUE_INITIALIZER   { true, 0, false, NULL }

struct q;                   //is private; defined in multiqueue.c
typedef struct q q_t;

typedef struct
{
    bool                    q_isenabled;
    uint                    q_max_size;
    bool                    q_max_size_enabled;
    q_t*                    pqueue;
} multiqueue_t;

/*
 * initializes multiqueue
 *
 * returns EOK on success;
 */
int multiqueue_init(multiqueue_t* pqid);

/*
 * cleans up multiqueue
 *
 * returns EOK;
 */
int multiqueue_fini(multiqueue_t* pqid);

/*
 * Helper function to initialize a multiqueue struct
 * Is same as using MULTIQUEUE_INITIALIZER to initialize static vars.
 */
int multiqueue_queue_init(multiqueue_t* pqid);

/*
 * clears the msg queue; is thread-safe
 */
int multiqueue_clear(multiqueue_t* pqid);

/*
 * Frees payload; for use after using _datapop methods
 */
void multiqueue_free_payload(void* payload);


/*
 * Cancels a blocked caller to multiqueue_waitfor_datapop
 */
int multiqueue_cancel_waitfor(multiqueue_t* pqid);

/*
 * Blocks until data available; Pops next msg payload off queue;
 * payload contains pointer to msg payload
 * ***user must dealloc payload when finished with data (ie. multiqueue_free_payload)
 * returns length of payload otherwise if no msg available returns -1
 */
int multiqueue_waitfor_datapop(multiqueue_t* pqid, void** payload);

/*
 * Pops next msg payload off queue;
 * payload contains pointer to msg payload
 * ***user must dealloc payload when finished with data (ie. multiqueue_free_payload)
 * returns length of payload otherwise if no msg available returns -1
 */
int multiqueue_datapop(multiqueue_t* pqid, void** payload);

/*
 * Pushes src msg onto queue
 * queue may block until queue can accept len bytes
 */
int multiqueue_datapush(
        multiqueue_t* pqid,
        void*   src,
        int     len
        );

#endif
