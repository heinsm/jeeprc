#include <netinet/in.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <sys/select.h>

#include <wiringPi.h>
#include <piFace.h>

#include <assert.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <stdarg.h>
#include <errno.h>
#include <pthread.h>
#include <sys/queue.h>
#include <sys/types.h>


//gpsd
#include "gps.h"
#include "gpsdclient.h"

#if !defined( EOK )  //see http://www.ibm.com/developerworks/aix/library/au-errnovariable/
#  define EOK 0         /* no error */
#endif

#define MAX_LINE                        16384
#define MAX_CMD_ARG_COUNT               4
#define MAX_CMD_ARG_LENGTH              10
#define MAX_CMD_CHAIN                   4
#define EOL                            "\r\n"
#define QUEUE_COUNT                     3

#define QUEUE_NAME_NA                   -2
#define QUEUE_NAME_IMMEDIATE            -1
#define QUEUE_NAME_PRI_1                0
#define QUEUE_NAME_PRI_2                1
#define QUEUE_NAME_TX                   2
#define ARRAY_SIZE(x)                   ((sizeof x) / (sizeof *x))


#define CMD_RESPONSE_IMMEDIATE_TOKEN    "r T"
#define CMD_RESPONSE_PROCESSED_TOKEN    "R T"
#define CMD_RESPONSE_PROCESSED_RESULT   "R R"
#define CMD_RESPONSE_RECORD_DELIMITER   0x1F

#define PIFACE_BASE     200
#define RL1             (PIFACE_BASE+0) //drive motor dir cntl
#define RL2             (PIFACE_BASE+0) //drive motor dir cntl
#define RL3             (PIFACE_BASE+1) //drive motor cntl

#define RL4             (PIFACE_BASE+2) //turn motor dir cntl
#define RL5             (PIFACE_BASE+2) //turn motor dir cntl
#define RL6             (PIFACE_BASE+3) //turn motor cntl
//#define RL7             (PIFACE_BASE+4)
//#define RL8             (PIFACE_BASE+5)



/*
 * All cntrl cmds that can be recieved
 */
typedef enum
{
    cc_CMD_CHAIN_START = -1,
    cc_single_cmd = 0,
    cc_move,
    cc_turn,
    cc_pause,
    cc_batch,
    cc_sensor,
    cc_script,
    cc_job,
    cc_help,
    cc_debug,
    cc_version,
    cc_batch_create,
    cc_batch_delete,
    cc_batch_append,
    cc_batch_insert,
    cc_batch_list,
    cc_batch_run,
    cc_batch_pause,
    cc_job_list,
    cc_job_timeouts,
    cc_job_save,
    cc_job_recall,
    cc_sensor_waitfor,
    cc_sensor_read,
    cc_sensor_list,
    cc_sensor_set,
    cc_script_if_scalar,
    cc_script_goto,
    cc_script_label,
} cntrl_cmd_e;

typedef struct ctrl_cmd cntrl_cmd_t; //prototype for ctrlcmd
typedef struct cmd_mapping_record
{
    int             cmd_lvl;                                    //command chain lvl
    cntrl_cmd_e     cmd;                                        //cmd enum
    const char*     literal;                                    //ascii literal (recieved)
    cntrl_cmd_e     linked_cmd;                                 //linked to command;
    uint            min_arg_count;                              //sanity check - recvd command must have at least min args
    uint            max_arg_count;                              //sanity check - revd command cannot have greater than max args
    int             queuePriority;                              //indicates queue for cmd
    int             (*action)(cntrl_cmd_t*);             //cmd action; normally null unless linked_cmd is chain end
} cmd_mapping_record_t;

typedef struct
{
    int     version;
} cntrl_prot_t;

typedef struct
{
    int             fd;
    char            buffer[MAX_LINE];
    size_t          buffer_used;

    int             writing;
    size_t          n_written;
    size_t          write_upto;
    cntrl_prot_t    cntrl;
} fd_state_t;

/*
 * cntrl_cmd holder
 */
typedef struct ctrl_cmd
{
    cntrl_cmd_e             cmd[MAX_CMD_CHAIN];
    char                    args[MAX_CMD_ARG_COUNT][MAX_CMD_ARG_LENGTH+1];
    uint                    cmd_count;
    uint                    arg_count;
    cmd_mapping_record_t*   mapped_cmd;
    uint                    response_token;
    char                    response_buf[1024];
    uint                    response_sz;
    fd_state_t*             fdstate;
} cntrl_cmd_t;

typedef struct cmdq_entry
{
    STAILQ_ENTRY(cmdq_entry)                entries;
    cntrl_cmd_t                             cntrl_cmd;
} cmdq_entry_t;

typedef STAILQ_HEAD(, cmdq_entry) cmdq_head_t;

typedef struct
{
    cmdq_head_t             qhead;
    uint                    qcount;
    pthread_mutex_t         q_mutex;
    pthread_cond_t          signal_q_hasdata;
} cmdq_t;

int action_dead( cntrl_cmd_t* cc); //TODO: remove this prototype
int action_debug( cntrl_cmd_t* cc); //TODO: remove this prototype
int action_read_gps( cntrl_cmd_t* cc); //TODO: remove this prototype
int action_move( cntrl_cmd_t* cc); //TODO: remove this prototype
int action_turn( cntrl_cmd_t* cc); //TODO: remove this prototype

static cmd_mapping_record_t cmd_mapping[] = {
    { 0, cc_single_cmd,         "c",    cc_CMD_CHAIN_START,             0,  0,  QUEUE_NAME_NA,      NULL },
    { 1, cc_move,               "m",    cc_single_cmd,                  2,  2,  QUEUE_NAME_IMMEDIATE,  action_move },
    { 1, cc_turn,               "t",    cc_single_cmd,                  2,  2,  QUEUE_NAME_IMMEDIATE,  action_turn },
    { 1, cc_pause,              "p",    cc_single_cmd,                  1,  1,  QUEUE_NAME_IMMEDIATE,  action_dead },
    { 1, cc_sensor,             "s",    cc_single_cmd,                  0,  0,  QUEUE_NAME_NA,      NULL },
    { 0, cc_batch,              "b",    cc_CMD_CHAIN_START,             0,  0,  QUEUE_NAME_NA,      NULL },
    { 1, cc_batch_create,       "c",    cc_batch,                       1,  1,  QUEUE_NAME_PRI_2,  action_dead },
    { 1, cc_batch_delete,       "d",    cc_batch,                       1,  1,  QUEUE_NAME_PRI_2,  action_dead },
    { 1, cc_batch_append,       "a",    cc_batch,                       1,  1,  QUEUE_NAME_NA,      NULL },
    { 2, cc_move,               "m",    cc_batch_append,                2,  2,  QUEUE_NAME_PRI_2,  action_dead },
    { 2, cc_turn,               "t",    cc_batch_append,                2,  2,  QUEUE_NAME_PRI_2,  action_dead },
    { 2, cc_pause,              "p",    cc_batch_append,                1,  1,  QUEUE_NAME_PRI_2,  action_dead },
    { 2, cc_sensor,             "s",    cc_batch_append,                0,  0,  QUEUE_NAME_NA,      NULL },
    { 2, cc_script,             "!",    cc_batch_append,                0,  0,  QUEUE_NAME_NA,      NULL },
    { 1, cc_batch_insert,       "i",    cc_batch,                       2,  2,  QUEUE_NAME_NA,      NULL },
    { 2, cc_move,               "m",    cc_batch_insert,                2,  2,  QUEUE_NAME_PRI_2,  action_dead },
    { 2, cc_turn,               "t",    cc_batch_insert,                2,  2,  QUEUE_NAME_PRI_2,  action_dead },
    { 2, cc_pause,              "p",    cc_batch_insert,                1,  1,  QUEUE_NAME_PRI_2,  action_dead },
    { 2, cc_sensor,             "s",    cc_batch_insert,                0,  0,  QUEUE_NAME_NA,      NULL },
    { 2, cc_script,             "!",    cc_batch_append,                0,  0,  QUEUE_NAME_NA,      NULL },
    { 1, cc_batch_list,         "l",    cc_batch,                       1,  1,  QUEUE_NAME_PRI_2,  action_dead },
    { 1, cc_batch_run,          "r",    cc_batch,                       0,  0,  QUEUE_NAME_PRI_2,  action_dead },
    { 1, cc_batch_pause,        "P",    cc_batch,                       0,  0,  QUEUE_NAME_PRI_2,  action_dead },
    { 0, cc_job,                "j",    cc_CMD_CHAIN_START,             0,  0,  QUEUE_NAME_PRI_2,  action_dead },
    { 1, cc_job_list,           "l",    cc_job,                         0,  0,  QUEUE_NAME_PRI_2,  action_dead },
    { 1, cc_job_timeouts,       "t",    cc_job,                         2,  2,  QUEUE_NAME_PRI_2,  action_dead },
    { 1, cc_job_save,           "s",    cc_job,                         2,  2,  QUEUE_NAME_PRI_2,  action_dead },
    { 1, cc_job_recall,         "r",    cc_job,                         1,  1,  QUEUE_NAME_PRI_2,  action_dead },
    { 0, cc_help,               "h",    cc_CMD_CHAIN_START,             0,  0,  QUEUE_NAME_PRI_2,  action_dead },
    { 0, cc_debug,              "d",    cc_CMD_CHAIN_START,             0,  0,  QUEUE_NAME_PRI_2,  action_debug },
    { 0, cc_version,            "v",    cc_CMD_CHAIN_START,             0,  0,  QUEUE_NAME_PRI_2,  action_dead },
    { 3, cc_sensor_waitfor,     "w",    cc_sensor,                      3,  3,  QUEUE_NAME_PRI_2,  action_dead },
    { 3, cc_sensor_read,        "r",    cc_sensor,                      1,  1,  QUEUE_NAME_PRI_2,  action_read_gps },
    { 3, cc_sensor_list,        "l",    cc_sensor,                      0,  0,  QUEUE_NAME_PRI_2,  action_dead },
    { 3, cc_sensor_set,         "s",    cc_sensor,                      2,  2,  QUEUE_NAME_PRI_2,  action_dead },
    { 3, cc_script_if_scalar,   "ifs",  cc_script,                      0,  0,  QUEUE_NAME_PRI_2,  action_dead },
    { 3, cc_script_goto,        "goto", cc_script,                      0,  0,  QUEUE_NAME_PRI_2,  action_dead },
    { 3, cc_script_label,       "label",cc_script,                      0,  0,  QUEUE_NAME_PRI_2,  action_dead },
};

static cmdq_t               cmdq[QUEUE_COUNT];
static fd_state_t*          fd_tx_state[FD_SETSIZE];
static pthread_mutex_t      fd_tx_state_mutex = PTHREAD_MUTEX_INITIALIZER;
static uint                 next_tokenvalue;
static bool                 debug_flag = false;
static volatile int move_speed = 0;
static volatile int turn_speed = 0;

int action_debug( cntrl_cmd_t* cc )
{
    debug_flag = true;
    return EOK;
}


int action_dead( cntrl_cmd_t* cc )
{
    return EOK;
}


/*
 * Appends msg into the fdstate tx buffer
 *
 * return EOK on success; EFBUG is msg cannot fit into state; otherwise EERR type describing failure
 */
static int append_to_fdstate(fd_state_t* state, char* msg, size_t msg_size)
{
    if ( (state->buffer_used + msg_size) < sizeof(state->buffer) )
    {
        memcpy( &state->buffer[state->buffer_used], msg, msg_size );
        state->buffer_used += msg_size;
        state->write_upto = state->buffer_used;
        state->writing = 1;
    }
    else
    {
        return EFBIG;
    }

    return EOK;
}

/*
 * Prepends msg into the fdstate tx buffer
 *
 * return EOK on success; EFBUG is msg cannot fit into state; otherwise EERR type describing failure
 */
static int prepend_to_fdstate(fd_state_t* state, char* msg, size_t msg_size)
{
    if (state->buffer_used != 0)
    {
        if ( (state->buffer_used + msg_size) < sizeof(state->buffer) )
        {
            char tmp[sizeof(state->buffer)];
            memcpy( tmp, msg, msg_size );
            memcpy( &tmp[msg_size], &state->buffer[state->buffer_used], msg_size );
            memcpy( state->buffer, tmp, state->buffer_used + msg_size);

            state->buffer_used += msg_size;
            state->write_upto = state->buffer_used;
            state->writing = 1;
        }
        else
        {
            return EFBIG;
        }
    }
    else
    {
        //appending doesn't need to cpy buffers into tmp
        //is faster...
        return append_to_fdstate(state, msg, msg_size);
    }

    return EOK;
}


/*
 * Pushes cmd onto a FIFO queue
 * returns EOK for success; otherwise EERR type for reason
 */
int push_cmd_to_queue(cmdq_t* q, cmdq_entry_t* cmd)
{
    int result = EOK;

    pthread_mutex_lock( &q->q_mutex );

    cmdq_entry_t* entry = (cmdq_entry_t*) malloc( sizeof( cmdq_entry_t ) );
    if ( NULL != entry )
    {
        memcpy(entry, cmd, sizeof(cmdq_entry_t));
        STAILQ_INSERT_TAIL( &q->qhead, entry, entries );

        q->qcount++;
        pthread_cond_signal( &q->signal_q_hasdata );
    }
    else
    {
        result =  ENOMEM;
    }

    pthread_mutex_unlock( &q->q_mutex );

    return result;
}

/*
 * Pops cmd off a FIFO queue; provided cmd will be filled with cmd payload
 * Note: Caller must hold q mutex prior to call
 * returns EOK for success; otherwise EERR type for reason
 */
int pop_cmd_from_queue_nolock(cmdq_t* q, cmdq_entry_t* cmd)
{
    int result = EOK;
    cmdq_entry_t* p;

    p = STAILQ_FIRST( &q->qhead );
    if ( NULL != p )
    {
        STAILQ_REMOVE_HEAD( &q->qhead, entries );

        memcpy( cmd, p, sizeof( cmdq_entry_t ) );
        free( p );

        q->qcount--;
    }
    else
    {
        //queue is empty
        result =  EAGAIN;
    }

    return result;
}

/*
 * Waits for specified queue to indicate hasdata
 * Note: Caller must hold q mutex prior to call
 */
int waitfor_q_hasdata_nolock(cmdq_t* q)
{
    while (q->qcount == 0)
    {
        pthread_cond_wait( &q->signal_q_hasdata, &q->q_mutex );
    }

    return EOK;
}

int init_cmd_queues()
{
    for (uint i=0; i < QUEUE_COUNT; i++)
    {
        STAILQ_INIT(&cmdq[i].qhead);
        cmdq[i].qcount = 0;
        pthread_mutex_init(&cmdq[i].q_mutex, NULL);
        pthread_cond_init(&cmdq[i].signal_q_hasdata, NULL);
    }

    return EOK;
}

fd_state_t* alloc_fd_state(void)
{
    fd_state_t* state = (fd_state_t*) malloc(sizeof(fd_state_t));
    if (!state)
    {
        return NULL;
    }

    state->buffer_used = state->n_written = state->writing = state->write_upto =
            0;
    return state;
}

void free_fd_state(fd_state_t* state)
{
    free(state);
}

void make_nonblocking(int fd)
{
    fcntl(fd, F_SETFL, O_NONBLOCK);
}

int parse_cmds(char* buf, size_t size, cntrl_cmd_t* cc)
{
    char par_buf[MAX_CMD_ARG_COUNT + MAX_CMD_CHAIN][MAX_CMD_ARG_LENGTH + 1];
    int count = 0;
    char bufcpy[size+1];
    uint min_args = 0;
    uint max_args = 0;

    //make our own cpy of buf and guarantee nullterm
    memcpy(bufcpy, buf, size);
    bufcpy[size] = '\0';

    //guarantee null terminating sscan bufs
    memset(&par_buf, 0, sizeof(par_buf));

    //init cc
    memset(cc, 0, sizeof(cntrl_cmd_t));

    //todo: add code to have scanf dynamically take on more cmd lvls and args following the defines
    //scan for cmds
    count = sscanf(bufcpy, "%10s %10s %10s %10s %10s %10s %10s %10s %10s", &par_buf[0][0],
            &par_buf[1][0], &par_buf[2][0], &par_buf[3][0], &par_buf[4][0],
            &par_buf[5][0], &par_buf[6][0], &par_buf[7][0], &par_buf[8][0]);

    //to be valid cmd; we must have at least one cmd
    if (count >= 1)
    {
        //parse the cmd chain
        //we move left-to-right until linked cmd ==-1
        int srch_lvl = 0;
        for (uint i = 0; i < ARRAY_SIZE(cmd_mapping); i++)
        {
            if ((srch_lvl != 0)     //this isn't the first lvl search
                    && (cmd_mapping[i].linked_cmd != cc->cmd[srch_lvl -1])) //and the current cmdmapping is _NOT_ linked to last cmd
            {
                //this cmd mapping is not linked to last matched cmd
                //we will skip it.
                continue;
            }

            if ((cmd_mapping[i].cmd_lvl == srch_lvl)    //is this mapping for this lvl?
                    && (strcmp(&par_buf[srch_lvl][0],cmd_mapping[i].literal) == 0))  //does this mapping match the cmd literal?
            {
                //record the next cmd chain link
                cc->cmd[srch_lvl] = cmd_mapping[i].cmd;
                min_args += cmd_mapping[i].min_arg_count;
                max_args += cmd_mapping[i].max_arg_count;

                //check if this command is chain end; ie. chain ends have actions
                if ( cmd_mapping[i].action != NULL )
                {
                    //yes - is chain end
                    //record the final cmd as the action cmd
                    //and copy the parsed args as found (starting after last seen cmd)
                    cc->mapped_cmd = &cmd_mapping[i];
                    cc->cmd_count = srch_lvl + 1;
                    cc->arg_count = count - cc->cmd_count;
                    cc->response_token = next_tokenvalue;
                    next_tokenvalue++;

                    memcpy(cc->args, &par_buf[srch_lvl + 1][0], (MAX_CMD_ARG_LENGTH+1) * cc->arg_count);

                    if ((cc->arg_count < min_args)          //missing args
                        || (cc->arg_count > max_args))     //too many args
                    {
                        return EAGAIN;
                    }

                    return EOK;
                }
                else
                {
                    //no - was a chain cmd; but not the chain end.
                    //keep searching...
                    srch_lvl++;
                    i = -1;
                    assert(srch_lvl < MAX_CMD_CHAIN);
                }
            }
        }
    }

    //no chain end cmd found
    //invalid cmd input
    return EAGAIN;
}

size_t debug_print_cc(cntrl_cmd_t* cc, char* buf, size_t size)
{
    const int outsize = 2048;
    char out[outsize];
    int index = 0;

    //setup out buf
    memset(out,0,outsize);

    index += snprintf(&out[index], outsize - index, "******************" EOL);

    index += snprintf(&out[index], outsize - index, "CMD CHAIN:" EOL);
    for (uint i=0; i < cc->cmd_count; i++)
    {
        index += snprintf(&out[index], outsize - index, "CMD%d: %d" EOL, i, cc->cmd[i]);
    }

    index += snprintf(&out[index], outsize - index, EOL);
    index += snprintf(&out[index], outsize - index, "CMD Args:" EOL);
    for (uint i=0; i < cc->arg_count; i++)
    {
        index += snprintf(&out[index], outsize - index, "Args%d: %s" EOL, i, &cc->args[i][0]);
    }

    index += snprintf(&out[index], outsize - index, EOL);
    if (cc->mapped_cmd == NULL)
    {
        index += snprintf(&out[index], outsize - index, "CMD is NULL!!" EOL);
    }
    else
    {
        index += snprintf(&out[index], outsize - index, "CMD Has Action!" EOL);
    }

    index += snprintf(&out[index], outsize - index, "%s" EOL, cc->response_buf);

    index += snprintf(&out[index], outsize - index, "******************" EOL EOL);

    //copy the out buffer into buf (upto maximum size)
    //and force null terminator
    strncpy(buf, out, size);
    buf[size-1] = '\0';

    return index;
}

int respond_cmd_immediate(cntrl_cmd_t* cc)
{
    //responds with immediate cmd response token
    char msg[1024];
    int index = 0;

    index += snprintf( &msg[index], sizeof(msg) - index, "%s %d" EOL, CMD_RESPONSE_IMMEDIATE_TOKEN, cc->response_token);
    index += snprintf( &msg[index], sizeof(msg) - index, "%c" EOL, CMD_RESPONSE_RECORD_DELIMITER);

    pthread_mutex_lock(&fd_tx_state_mutex);
    prepend_to_fdstate(cc->fdstate, msg, index);
    pthread_mutex_unlock(&fd_tx_state_mutex);


    return EOK;
}

int respond_cmd_processed(cntrl_cmd_t* cc, char* buf, size_t bufsz)
{
    //responds with immediate cmd response token
    char msg[MAX_LINE];
    int index = 0;

    index += snprintf( &msg[index], sizeof(msg) - index, "%s %d" EOL, CMD_RESPONSE_PROCESSED_TOKEN, cc->response_token);
    index += snprintf( &msg[index], sizeof(msg) - index, "%s %s" EOL, CMD_RESPONSE_PROCESSED_RESULT, cc->response_buf);
    index += snprintf( &msg[index], sizeof(msg) - index, "%c" EOL, CMD_RESPONSE_RECORD_DELIMITER);

    //copy the out buffer into buf (upto maximum size)
    //and force null terminator
    strncpy(buf, msg, bufsz);
    buf[bufsz-1] = '\0';

    return index;
}

int prioritize_cmd(cntrl_cmd_t* cc, bool is_parse_valid)
{
    int result = EOK;
    cmdq_entry_t ce;

    if (cc->cmd_count > 0)
    {
        //init ce placeholder with cmd
        memcpy( &ce.cntrl_cmd, cc, sizeof( cntrl_cmd_t ) );

        respond_cmd_immediate(&ce.cntrl_cmd);

        if (is_parse_valid)
        {

            //exec immediately or move cmd into priority queue
            switch (cc->mapped_cmd->queuePriority)
            {
                case QUEUE_NAME_NA:
                {
                    assert(false);  //this should never happen - queuePriority==QUEUE_NAME_NA
                    break;
                }
                case QUEUE_NAME_IMMEDIATE:
                {
                    //exec immediately
                    cc->mapped_cmd->action(cc);

                    snprintf(&ce.cntrl_cmd.response_buf[0], sizeof(ce.cntrl_cmd.response_buf), "PROCESSED IMMEDIATELY");
                    push_cmd_to_queue( &cmdq[QUEUE_NAME_TX], &ce);
                    break;
                }

                default:
                {
                    if (cc->mapped_cmd->queuePriority < QUEUE_COUNT)
                    {
                        result = push_cmd_to_queue( &cmdq[cc->mapped_cmd->queuePriority], &ce );
                    }
                    else
                    {
                        result = EINVAL;
                    }

                    break;
                }
            }
        }
        else
        {
            //parsed commands where invalid...
            snprintf(&ce.cntrl_cmd.response_buf[0], sizeof(ce.cntrl_cmd.response_buf), "unrecognized or malformed cmd; ignored");
            push_cmd_to_queue( &cmdq[QUEUE_NAME_TX], &ce);
            result =  EINVAL;
        }
    }
    else
    {
        //no response sent
        //return failure when no parsed commands found
        result =  EINVAL;
    }

    return result;
}

int do_read(fd_state_t *state)
{
    char buf[1024];
    ssize_t result;
    cntrl_cmd_t cc;
    int rc;
    int fd = state->fd;

    while (1)
    {
        result = recv(fd, buf, sizeof(buf), 0);
        if (result <= 0)
        {
            break;
        }

        rc = parse_cmds(buf, result, &cc);

        //*****DEBUG PRINT recvd buffer
        buf[result] = '\0';
        printf("%s %s\n", buf, ((rc==EOK)? "PROCESSED": "IGNORED"));

        cc.fdstate = state;
        rc = prioritize_cmd(&cc, rc==EOK);
    }

    if (result == 0)
    {
        return 1;
    }
    else if (result < 0)
    {
        if (errno == EAGAIN)
        {
            return 0;
        }

        return -1;
    }

    return 0;
}

int do_write(fd_state_t *state)
{
    int fd = state->fd;

    while (state->n_written < state->write_upto)
    {
        ssize_t result = send(fd, state->buffer + state->n_written,
                state->write_upto - state->n_written, 0);
        if (result < 0)
        {
            if (errno == EAGAIN)
                return 0;
            return -1;
        }
        assert(result != 0);

        state->n_written += result;
    }

    if (state->n_written == state->buffer_used)
    {
        state->n_written = state->write_upto = state->buffer_used = 0;
    }

    state->writing = 0;

    return 0;
}

void run(void)
{
    int listener;
    struct sockaddr_in sin;
    int i, maxfd;
    fd_set readset, writeset, exset;
    int fdcount=0;

    sin.sin_family = AF_INET;
    sin.sin_addr.s_addr = 0;
    sin.sin_port = htons(40713);

    for (i = 0; i < FD_SETSIZE; ++i)
    {
        fd_tx_state[i] = NULL;
    }

    listener = socket(AF_INET, SOCK_STREAM, 0);
    make_nonblocking(listener);

    int one = 1;
    setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    if (bind(listener, (struct sockaddr*) &sin, sizeof(sin)) < 0)
    {
        perror("bind");
        return;
    }

    if (listen(listener, 16) < 0)
    {
        perror("listen");
        return;
    }

    FD_ZERO(&readset);
    FD_ZERO(&writeset);
    FD_ZERO(&exset);

    while (1)
    {
        maxfd = listener;

        FD_ZERO(&readset);
        FD_ZERO(&writeset);
        FD_ZERO(&exset);

        FD_SET(listener, &readset);

        pthread_mutex_lock( &fd_tx_state_mutex );
        for (i = 0; i < FD_SETSIZE; ++i)
        {
            if (fd_tx_state[i] != NULL)
            {
                if (fd_tx_state[i]->fd > maxfd)
                    maxfd = fd_tx_state[i]->fd;
                FD_SET(fd_tx_state[i]->fd, &readset);
                if (fd_tx_state[i]->writing)
                {
                    FD_SET(fd_tx_state[i]->fd, &writeset);
                }
            }
        }
        pthread_mutex_unlock( &fd_tx_state_mutex );
        struct timeval tv = {1, 50000};   // sleep for 100 ms
        int rc = select(maxfd + 1, &readset, &writeset, &exset, &tv);
        if ( rc < 0 )
        {
            perror("select");
            return;
        }
        else if (rc == 0) //timedout
        {
            //TODO: should check for global write flag here;
            //if not set just go back to sleep var
        }


        if (FD_ISSET(listener, &readset))
        {
            struct sockaddr_storage ss;
            socklen_t slen = sizeof(ss);
            int fd = accept(listener, (struct sockaddr*) &ss, &slen);
            if (fd < 0)
            {
                perror("accept");
            }
            else if (fd > FD_SETSIZE)
            {
                close(fd);
            }
            else
            {
                make_nonblocking(fd);
                fd_tx_state[fdcount] = alloc_fd_state();
                fd_tx_state[fdcount]->fd = fd;
                assert(fd_tx_state[fdcount]);
                fdcount++;
                /*XXX*/
            }
        }

        for (i = 0; i < fdcount; ++i)
        {
            int r = 0;
            //if (i == listener)
            //{
            //    continue;
            //}

            if (fd_tx_state[i] == NULL)
            {
                continue;
            }

            if (FD_ISSET(fd_tx_state[i]->fd, &readset))
            {
                r = do_read(fd_tx_state[i]);
            }

            if (r == 0 && FD_ISSET(fd_tx_state[i]->fd, &writeset))
            {
                r = do_write(fd_tx_state[i]);
            }

            if (r)
            {
                free_fd_state(fd_tx_state[i]);
                fd_tx_state[i] = NULL;
                close(i);
            }
        }
    }
}

int action_read_gps( cntrl_cmd_t* cc )
{
    static struct fixsource_t source;
    static struct gps_data_t gpsdata;
    unsigned int flags = WATCH_ENABLE;
    int result = EAGAIN;

    source.server = "localhost";
    source.port = "2947";


    if (gps_open(source.server, source.port, &gpsdata) != 0)
    {
        snprintf(&cc->response_buf[0], sizeof(cc->response_buf), "LAT -1, LONG -1, ALT -1");
    }
    else
    {
        if (source.device != NULL)
            flags |= WATCH_DEVICE;

        (void)gps_stream(&gpsdata, flags, source.device);

        //we try a number of reads until a fix is made
        //if no fix is made we fail with std response
        bool isread = false;
        for (int i=0; i<50; i++)
        {
            if (gps_waiting(&gpsdata, 5000000))
            {
                gps_read(&gpsdata);

                if (gpsdata.fix.mode > MODE_NO_FIX)
                {
                    isread=true;
                    break;
                }
            }
        }

        if (isread)
        {
            snprintf(&cc->response_buf[0], sizeof(cc->response_buf), "FIXMODE %d, LAT %f, LONG %f, ALT %f", gpsdata.fix.mode, gpsdata.fix.latitude, gpsdata.fix.longitude, gpsdata.fix.altitude);
            result = EOK;
        }
        else
        {
            snprintf(&cc->response_buf[0], sizeof(cc->response_buf), "FIXMODE %d, LAT -1, LONG -1, ALT -1", gpsdata.fix.mode);
        }
    }

    (void)gps_stream(&gpsdata, WATCH_DISABLE, NULL);
    (void)gps_close(&gpsdata);

    return result;
}

int move_stage1()
{
    //motor power
    digitalWrite(RL3, HIGH);

    return EOK;
}

int move_stage3()
{
    //disable motor power
    digitalWrite(RL3, LOW);

    return EOK;
}

int move_stage2(int dir)
{
    //this is saftey - we should always be in stage3 prior to this call anyways
    //disable motor power
    move_stage3();

    //setup direction
    switch (dir)
    {
        case 1:
        {
            //forward
            digitalWrite(RL1, LOW);
            digitalWrite(RL2, LOW);
            break;
        }

        case 2:
        {
            //reverse
            digitalWrite(RL1, HIGH);
            digitalWrite(RL2, HIGH);
            break;
        }
        default:
        {
            //don't care'
            break;
        }
    }

    return EOK;
}

int action_move( cntrl_cmd_t* cc)
{
    //theory of operation
    //moving forward when previously moving forward; without delay
    //moving reverse when previously moving reverse; without delay
    //when transitioning from forward/reverse; cut power; delay; rearrage relays; delay; apply power
    //when stopped keep last known direction for moving again without delay

    int dir = atoi(cc->args[0]);
    int speed = atoi(cc->args[1]);

    static int last_dir = 0;




    switch (dir)
    {
        case 1:
        case 2:
        {
            //forward/reverse
            if (last_dir != dir)
            {
                move_speed = 0;
                usleep(400000);   //400ms
                move_stage2(dir);
                usleep(100000);   //100ms
            }
            move_speed = speed;

            //save old dir
            last_dir = dir;
            break;
        }

        default:
        {
            //all stop
            move_speed = 0;

            //dont save last dir here - this is all stop
            //last_dir = dir;
            break;
        }
    }

    snprintf(&cc->response_buf[0], sizeof(cc->response_buf), "ACK");

    return EOK;
}

void* movedebug_thread_entry(void* arg)
{
    float m = -1 / 33097.9797979798;
    float b = 1;
    int full_duty=1000000; //usec

    while (1)
    {
        //x = 0; t=1/1 hz=1sec
        //x = 32767; t=1/100hz=0.01
        //y=mx+b
        //m=y1-y2/x1-x2= 1-0.01/0-32767=0.99/-32767
        //y=mx+b;  -b=mx-y;  b=-mx+y

        //m=-1/33097.9797979798
        //b=1 (we want y intercept to be 1sec when x=0
        //y=mx+1


        if (move_speed == 0)
        {
            move_stage3();
            usleep(200000);   //200ms
        }
        else
        {
            float y = m*(abs(move_speed)) + b;
            y=y*1000000;  //bring y to usecs

            //calc off duty
            int off_duty=full_duty-(int)y;
            int on_duty=full_duty-off_duty;

            move_stage1();
            usleep(off_duty/8);
            move_stage3();
            usleep(on_duty/8);
        }
    }

    return NULL;
}

int turn_stage1()
{
    //motor power
    digitalWrite(RL6, HIGH);

    return EOK;
}

int turn_stage3()
{
    //disable motor power
    digitalWrite(RL6, LOW);

    return EOK;
}

int turn_stage2(int dir)
{
    //this is saftey - we should always be in stage3 prior to this call anyways
    //disable motor power
    turn_stage3();

    //setup direction
    switch (dir)
    {
        case 1:
        {
            //forward
            digitalWrite(RL4, LOW);
            digitalWrite(RL5, LOW);
            break;
        }

        case 2:
        {
            //reverse
            digitalWrite(RL4, HIGH);
            digitalWrite(RL5, HIGH);
            break;
        }
        default:
        {
            //don't care'
            break;
        }
    }

    return EOK;
}

int action_turn( cntrl_cmd_t* cc)
{
    //theory of operation
    //moving forward when previously moving forward; without delay
    //moving reverse when previously moving reverse; without delay
    //when transitioning from forward/reverse; cut power; delay; rearrage relays; delay; apply power
    //when stopped keep last known direction for moving again without delay

    int dir = atoi(cc->args[0]);
    int speed = atoi(cc->args[1]);

    static int last_dir = 0;




    switch (dir)
    {
        case 1:
        case 2:
        {
            //forward/reverse
            if (last_dir != dir)
            {
                turn_speed = 0;
                usleep(400000);   //400ms
                turn_stage2(dir);
                usleep(100000);   //100ms
            }
            turn_speed = speed;

            //save old dir
            last_dir = dir;
            break;
        }

        default:
        {
            //all stop
            turn_speed = 0;

            //dont save last dir here - this is all stop
            //last_dir = dir;
            break;
        }
    }

    snprintf(&cc->response_buf[0], sizeof(cc->response_buf), "ACK");

    return EOK;
}

void* turndebug_thread_entry(void* arg)
{
    float m = -1 / 33097.9797979798;
    float b = 1;
    int full_duty=1000000; //usec

    while (1)
    {
        //x = 0; t=1/1 hz=1sec
        //x = 32767; t=1/100hz=0.01
        //y=mx+b
        //m=y1-y2/x1-x2= 1-0.01/0-32767=0.99/-32767
        //y=mx+b;  -b=mx-y;  b=-mx+y

        //m=-1/33097.9797979798
        //b=1 (we want y intercept to be 1sec when x=0
        //y=mx+1


        if (turn_speed == 0)
        {
            turn_stage3();
            usleep(200000);   //200ms
        }
        else
        {
            float y = m*(abs(turn_speed)) + b;
            y=y*1000000;  //bring y to usecs

            //calc off duty
            int off_duty=full_duty-(int)y;
            int on_duty=full_duty-off_duty;

            turn_stage1();
            usleep(off_duty/8);
            turn_stage3();
            usleep(on_duty/8);
        }
    }

    return NULL;
}

void* cmdq1_thread_entry( void* a )
{
    cmdq_t* q = &cmdq[QUEUE_NAME_PRI_1];
    cmdq_entry_t ce;
    int rc;

    while (1)
    {
        pthread_mutex_lock( &q->q_mutex );

        waitfor_q_hasdata_nolock( q );
        pop_cmd_from_queue_nolock( q, &ce);

        //execute the cmd action
        rc = ce.cntrl_cmd.mapped_cmd->action(&ce.cntrl_cmd);
        if (rc != EOK)
        {
            snprintf(ce.cntrl_cmd.response_buf, sizeof(ce.cntrl_cmd.response_buf), "PROCESSED BY QUEUE_NAME_PRI_1: NO ACTION AVAILABLE");
        }

        push_cmd_to_queue( &cmdq[QUEUE_NAME_TX], &ce);
        pthread_mutex_unlock( &q->q_mutex );
    }

    return NULL;
}

void* cmdq2_thread_entry( void* a )
{
    cmdq_t* q = &cmdq[QUEUE_NAME_PRI_2];
    cmdq_entry_t ce;
    int rc;



    while (1)
    {
        pthread_mutex_lock( &q->q_mutex );

        waitfor_q_hasdata_nolock( q );
        pop_cmd_from_queue_nolock( q, &ce);

        //execute the cmd action
        rc = ce.cntrl_cmd.mapped_cmd->action(&ce.cntrl_cmd);
        if (rc != EOK)
        {
            snprintf(ce.cntrl_cmd.response_buf, sizeof(ce.cntrl_cmd.response_buf), "PROCESSED BY QUEUE_NAME_PRI_2: NO ACTION AVAILABLE");
        }

        push_cmd_to_queue( &cmdq[QUEUE_NAME_TX], &ce);
        pthread_mutex_unlock( &q->q_mutex );
    }

    return NULL;
}

void* tx_thread_entry( void* a )
{
    cmdq_t* q = &cmdq[QUEUE_NAME_TX];
    cmdq_entry_t ce;
    char msgbuf[MAX_LINE];
    size_t msgbuf_sz = 0;

    while (1)
    {
        pthread_mutex_lock( &q->q_mutex );
        waitfor_q_hasdata_nolock( q );
        pop_cmd_from_queue_nolock( q, &ce);

        pthread_mutex_lock( &fd_tx_state_mutex );

        msgbuf_sz = respond_cmd_processed( &ce.cntrl_cmd, msgbuf, sizeof(msgbuf) );
        append_to_fdstate( ce.cntrl_cmd.fdstate, msgbuf, msgbuf_sz );

        if (debug_flag)
        {
            msgbuf_sz = debug_print_cc( &ce.cntrl_cmd, msgbuf, sizeof(msgbuf) );
            append_to_fdstate( ce.cntrl_cmd.fdstate, msgbuf, msgbuf_sz );
        }
        pthread_mutex_unlock( &fd_tx_state_mutex );
        pthread_mutex_unlock( &q->q_mutex );
    }

    return NULL;
}

int init_gpio()
{
    // Always initialise wiringPi. Use wiringPiSys() if you don't need
    //      (or want) to run as root
    wiringPiSetupSys();

    // Setup the PiFace board
    piFaceSetup(PIFACE_BASE);

    return EOK;
}

int main(int c, char **v)
{
    setvbuf(stdout, NULL, _IONBF, 0);

    init_cmd_queues();
    init_gpio();
    int i =0;
    pthread_t pid;

    pthread_create( &pid, NULL, cmdq1_thread_entry, &i );
    pthread_create( &pid, NULL, cmdq2_thread_entry, &i );
    pthread_create( &pid, NULL, tx_thread_entry, &i );


    pthread_create( &pid, NULL, movedebug_thread_entry, &i );
    pthread_create( &pid, NULL, turndebug_thread_entry, &i );

    run();
    return 0;
}
