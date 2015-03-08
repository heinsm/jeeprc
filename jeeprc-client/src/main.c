#include "joystickmgr.h"

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>
#include <stdbool.h>

int main(int argc, char *argv[])
{
    int sockfd = 0;
    char recvBuff[1024];
    struct sockaddr_in serv_addr;
    multiqueue_t* joy_q;
    joystick_msg_t* msg;
    int rc;
    bool client_shutdown = false;

    if(argc != 2)
    {
        printf("Usage: %s <ip of server> \n",argv[0]);
        return 1;
    }

    memset(recvBuff, '0',sizeof(recvBuff));
    if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("Error : Could not create socket \n");
        return 1;
    }

    memset(&serv_addr, '0', sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(40713);

    if(inet_pton(AF_INET, argv[1], &serv_addr.sin_addr)<=0)
    {
        printf("inet_pton error occured\n");
        return 1;
    }

    if( connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
       printf("Error : Connect Failed \n");
       return 1;
    }


    joystickmgr_init();
    joystickmgr_getqueue(&joy_q);

    while (!client_shutdown)
    {
        rc = multiqueue_waitfor_datapop(joy_q, (void**)&msg);
        if (rc != -1)
        {
            rc = send(sockfd, msg, sizeof(joystick_msg_t), 0);
            if (rc < 0)
            {
                if (errno != EAGAIN)
                {
                    //this some sorta bad...
                    printf("Error : send error %d\n", errno);
                    client_shutdown = true;
                }
            }
            else if (rc != sizeof(joystick_msg_t))
            {
                //could transmit msg in 1 go...
                //we don't currently support this
                printf("Error : couldn't send total msg - error\n");
                client_shutdown = true;
            }
            multiqueue_free_payload(msg);
        }
    }


    printf("Exiting - goodbye\n");
    close(sockfd);

    return 0;
}
