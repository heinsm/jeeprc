
#include "multiqueue.h"
#include "util.h"

#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <sys/queue.h>
#include <sys/types.h>
#include <malloc.h>

#define STDPRINT_NAME                       __FILE__ ":"
#define GET_Q(x)                            ((q_t*)x->pqueue)

typedef struct q_entry
{
    STAILQ_ENTRY(q_entry)                   entries;
    uint                                    len;
    void*                                   payload;
} q_entry_t;

typedef STAILQ_HEAD(, q_entry) q_head_t;

struct q
{
    q_head_t                qhead;
    volatile uint           q_current_size;
    pthread_mutex_t         q_mutex;
    pthread_cond_t          signal_q_hasdata;
    pthread_cond_t          signal_q_haspopped;
    volatile int            flag_msgq_hasdata;
    volatile int            flag_msgq_haspopped;
};  //q_t

/*
 * Internal function to handle q init
 */
static int multiqueue_init_q(q_t* pq)
{
    STAILQ_INIT( &pq->qhead );
    pq->q_current_size = 0;
    pthread_mutex_init(&pq->q_mutex, NULL);
    pthread_cond_init(&pq->signal_q_hasdata, NULL);
    pthread_cond_init(&pq->signal_q_haspopped, NULL);
    pq->flag_msgq_hasdata = false;
    pq->flag_msgq_haspopped = false;

    return EOK;
}

int multiqueue_init(multiqueue_t* pqid)
{
    int ret = EOK;
    q_t* pq;

    pq = (q_t*) malloc( sizeof( q_t ) );
    if ( pq != NULL )
    {
        ret = multiqueue_init_q(pq);
        pqid->pqueue = pq;
    }
    else
    {
        //entry malloc failed
        pqid->pqueue = NULL;
        ret = errno;
    }

    return ret;
}

int multiqueue_fini(multiqueue_t* pqid)
{
    int ret = EOK;
    ret = multiqueue_clear(pqid);

    free(pqid->pqueue);
    pqid->pqueue = NULL;

    return ret;
}

int multiqueue_queue_init(multiqueue_t* pqid)
{
    multiqueue_t pqid_master = MULTIQUEUE_INITIALIZER;

    memcpy(pqid, &pqid_master, sizeof(pqid_master));

    return EOK;
}

/*
 * Internal function to handle waiting for conditional variable signalling
 * and msgq has popped data
 */
static int multiqueue_waitfor_msgq_haspopped(multiqueue_t* pqid, uint len)
{
    //if there is no room in the queue for len bytes
    //and queue max size is enabled
    //we wait
    while ( ( ( pqid->pqueue->q_current_size + len ) >= pqid->q_max_size )
        && ( pqid->q_max_size_enabled ) )

    {
        pthread_cond_wait( &pqid->pqueue->signal_q_haspopped, &pqid->pqueue->q_mutex );
    }

    return EOK;
}

/*
 * Internal function to handle setting conditional variable signalling
 * and msgq has popped data
 */
static int multiqueue_set_msgq_haspopped(multiqueue_t* pqid)
{
    //signal ALL waiting for pop events
    return pthread_cond_broadcast( &pqid->pqueue->signal_q_haspopped );
}

int multiqueue_clear(multiqueue_t* pqid)
{
    //in a thread-safe way
    //dealloc all msg entries
    pthread_mutex_lock( &pqid->pqueue->q_mutex );
    q_entry_t *n1, *n2;

    // using "Faster TailQ Deletion."
    n1 = STAILQ_FIRST( &pqid->pqueue->qhead );
    while (n1 != NULL)
    {
        n2 = STAILQ_NEXT(n1, entries);

        free(n1->payload);
        free(n1);
        n1 = n2;
    }

    //reinit the head
    STAILQ_INIT( &pqid->pqueue->qhead );

    pqid->pqueue->q_current_size = 0;
    multiqueue_set_msgq_haspopped(pqid);

    pthread_mutex_unlock( &pqid->pqueue->q_mutex );

    return EOK;
}

/*
 * Internal function to handle waiting for conditional variable signalling
 * and q data ready state
 */
static int multiqueue_waitfor_q_hasdata(multiqueue_t* pqid)
{
    while ( !pqid->pqueue->flag_msgq_hasdata )
    {
        pthread_cond_wait( &pqid->pqueue->signal_q_hasdata, &pqid->pqueue->q_mutex );
    }

    //check for emptiness and reset hasData flag
    //otherwise we don't reset this flag since
    //the next waitfor should succeed immediately
    if ( STAILQ_EMPTY( &pqid->pqueue->qhead ) )
    {
        pqid->pqueue->flag_msgq_hasdata = 0;
    }
    return EOK;
}

/*
 * Internal function to handle setting conditional variable signalling
 * and q data ready state
 */
static int multiqueue_set_q_hasdata(multiqueue_t* pqid)
{
    pqid->pqueue->flag_msgq_hasdata = 1;
    return pthread_cond_signal( &pqid->pqueue->signal_q_hasdata );
}

void multiqueue_free_payload(void* payload)
{
    free( payload );
}

int multiqueue_cancel_waitfor(multiqueue_t* pqid)
{
    return multiqueue_set_q_hasdata( pqid );
}

/*
 * Internal datapop function to support reuse of data pop functionality
 * As a caller - remember to lock mutex prior to calling this function!
 */
static int multiqueue_datapop_nolock(multiqueue_t* pqid, void** payload)
{
    //pop head off FIFO
    //free the queue entry
    //**user must dealloc payload when finished with data
    //returns length of payload, *payload has pointer to data; otherwise -1 if no data

    int ret_len = -1;
    q_entry_t* p;

    p = STAILQ_FIRST( &pqid->pqueue->qhead );
    if ( NULL != p )
    {
        STAILQ_REMOVE_HEAD( &pqid->pqueue->qhead, entries );

        *payload = p->payload;
        ret_len = p->len;
        pqid->pqueue->q_current_size -= p->len;
        free(p);
    }

    multiqueue_set_msgq_haspopped(pqid);

    return ret_len;
}


int multiqueue_waitfor_datapop(multiqueue_t* pqid, void** payload)
{
    int ret_len = -1;

    if ( pqid->q_isenabled )
    {
        pthread_mutex_lock( &pqid->pqueue->q_mutex );

        //block and wait for data ready signal
        multiqueue_waitfor_q_hasdata(pqid);

        //read data
        ret_len = multiqueue_datapop_nolock( pqid, payload );

        pthread_mutex_unlock( &pqid->pqueue->q_mutex );
    }

    return ret_len;
}

int multiqueue_datapop(multiqueue_t* pqid, void** payload)
{
    //mutex lock (protect cipher context ordering instep with queue ordering)
    //call multiqueue_datapop_nolock
    //mutex unlock
    //**user must dealloc payload when finished with data
    //returns length of payload, *payload has pointer to data; otherwise -1 if no data

    int ret_len = -1;

    if ( pqid->q_isenabled )
    {
        pthread_mutex_lock( &pqid->pqueue->q_mutex );

        ret_len = multiqueue_datapop_nolock( pqid, payload );

        pthread_mutex_unlock( &pqid->pqueue->q_mutex );
    }

    return ret_len;
}


int multiqueue_datapush(
        multiqueue_t* pqid,
        void*   src,
        int     len
        )
{
    int ret = EAGAIN;

    if ( pqid->q_isenabled )
    {
        pthread_mutex_lock( &pqid->pqueue->q_mutex );
        q_entry_t* entry = (q_entry_t*) malloc( sizeof( q_entry_t ) );
        if (NULL != entry)
        {
            entry->len = len;
            entry->payload =  malloc( len );
            if (NULL != entry->payload)
            {
                //do following:
                //-block until queue can accept len bytes
                //-copy without encryption srcbuf into heap dst buffer
                //-push result into msg q tail

                multiqueue_waitfor_msgq_haspopped( pqid, len );
                memcpy( entry->payload, src, len );
                STAILQ_INSERT_TAIL( &pqid->pqueue->qhead, entry, entries );
                pqid->pqueue->q_current_size += len;

                ret = EOK;
            }
            else
            {
                free( entry );
            }
        }

        multiqueue_set_q_hasdata(pqid);
        pthread_mutex_unlock( &pqid->pqueue->q_mutex );
    }

    return ret;
}
