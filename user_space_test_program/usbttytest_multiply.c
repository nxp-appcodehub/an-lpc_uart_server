// SPDX-License-Identifier: GPL-2.0+
/*
*  LPC USB-10 ports Serial Converters
*
*  Copyright 2018 NXP
*  All rights reserved.
*
*  Shamelessly based on DIGI driver
*
*
*/

#include <pthread.h>
#include <stdio.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <errno.h>
#include <signal.h>

#define PORT_NUMBERS   10
#define BULK_IN_SIZE   253
#define BULK_OUT_SIZE  253
#define EVERY_PRINT    64

typedef struct
{
    int index;
    int baud;
    int loopback;
    const char*  readFileName;
    const char*  writeFileName;
    const char*  devName;
    const char*  loopbackAttrName;

}ReadWriteThreadArgument;

typedef struct
{
    int fd;
    int index;
    const char*  sendFileName;

}WriteThreadArgument;

void startThread(void* p,int isWrite);

void serial_init(int fd,int baud)
{
    struct termios options;
    int realbaud = 0;
    tcgetattr(fd,&options);
    options.c_cflag |= ( CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag |= IGNPAR;
    options.c_iflag &= ~(BRKINT | ICRNL | INPCK| ISTRIP | IXON);
    options.c_oflag = 0;
    options.c_lflag = 0;


    switch (baud) {

    case 4800:
        realbaud = B4800;
        break;
   case 9600:
        realbaud = B9600;
        break;
    case 38400:
        realbaud = B38400;
        break;
    case 115200:
        realbaud = B115200;
        break;

    default:
        realbaud = B115200;
    }

    cfsetispeed(&options,realbaud);
    cfsetospeed(&options,realbaud);

    tcsetattr(fd,TCSANOW,&options);


}


void * writePort(void* data)
{
    WriteThreadArgument* p = (WriteThreadArgument*) data;
    int readfd = 0;
    int everyinfo = EVERY_PRINT * 1024;
    int everysum  = 0;
    int writefd = p->fd;
    int index = p->index;
    const char* sendfile = p->sendFileName;
    unsigned char buffer[BULK_OUT_SIZE];
    unsigned char tmpbuf[1024];
    int length = 0;
    int writelength = 0;
    int readsum = 0;
    int writesum = 0;
    int filesize = 0;
    fd_set rset,wset;


    FD_ZERO(&rset);
    FD_ZERO(&wset);
    FD_SET(writefd,&wset);
    FD_SET(readfd,&rset);


    readfd = open(sendfile,O_RDWR);
    if(-1 == readfd)
    {
        printf("can't open the %s file\n",sendfile);
        pthread_exit(NULL);
    }
    filesize = lseek(readfd,0,SEEK_END);
    lseek(readfd,0,SEEK_SET);
    while((length = read(readfd,buffer,sizeof(buffer))) != 0)
    {
            readsum += length;
            everysum += length;

            while(length)
            {
                select(writefd + 1,NULL,&wset,NULL,NULL);
                if(FD_ISSET(writefd,&wset))
                {
                        writelength= write(writefd,buffer,length);
                        if(writelength == -1 && errno == EAGAIN)
                        {
                            continue;
                        }

                        length -= writelength;
                        writesum += writelength;
                }
            }
           /* if(4 == index )
            {
                memmove(tmpbuf,buffer,writelength);
                tmpbuf[writelength] = '\0';
                printf("%s\n",tmpbuf);
            }*/
            fflush(stdout);
            printf("have send data percent %d%%\r",(int)((readsum*100.0)/filesize));


    }

    printf("--%d--port   read sum is %d -- send sum is %d\n",index,readsum,writesum);
    pthread_exit(NULL);
}

void * readWritePort(void* data)
{
    ReadWriteThreadArgument* p = (ReadWriteThreadArgument*) data;
    WriteThreadArgument writeThreadArguments;
    const char* devname = p->devName;
    const char* sendfile = p->readFileName;
    const char* recvfile = p->writeFileName;
    const char* loopattrname = p->loopbackAttrName;
    int index = p->index;
    int loopback = p->loopback;
    int devfd = 0;
    int loopfd = 0;
    int sendfd = 0;
    int recvfd = 0;
    int readsum = 0;
    int recvsum = 0;
    int writelength = 0;
    int length = 0;
    fd_set rset, wset;
    char loopbool[2] = {'0','1'};
    unsigned char buffer[BULK_IN_SIZE] = {0};


    devfd = open(devname,O_RDWR | O_NOCTTY | O_NDELAY);
    if(-1 == devfd)
    {
        printf("can't open the %s port\n",devname);
        pthread_exit(NULL);
    }
    serial_init(devfd,p->baud);

    //setup the loopback
    loopfd = open(loopattrname,O_RDWR);
    if(-1 == loopfd)
    {
        printf("can't open the loopback attribute %s\n",loopattrname);
        pthread_exit(NULL);
    }
    if(loopback)
    {
       length = write(loopfd,&loopbool[1],1);
    }
    else
    {
       length = write(loopfd,&loopbool[0],1);
    }
    if(-1 == length)
    {
        printf("write  the loopback attribute failed\n");
        pthread_exit(NULL);
    }

    recvfd = open(recvfile, O_RDWR | O_CREAT | O_TRUNC,0666);
    if(-1 == recvfd)
    {
        printf("can't open the %s file\n",recvfile);
        pthread_exit(NULL);
    }

    /*send the data to index port*/
    writeThreadArguments.fd = devfd;
    writeThreadArguments.index = index;
    writeThreadArguments.sendFileName = sendfile;
    startThread(&writeThreadArguments,1);


    FD_ZERO(&rset);
    FD_ZERO(&wset);
    FD_SET(devfd,&rset);
    FD_SET(recvfd,&wset);

    while(1)
    {
        if(select(devfd+1,&rset,NULL,NULL,NULL) < 0)
        {
            printf("--%d-- port select error\n",index);
            pthread_exit(NULL);
        }

        if(FD_ISSET(devfd,&rset))
        {

            length = read(devfd,buffer,sizeof(buffer));
            if(length == 0)
            {
                break;
            }

            readsum += length;

            while(length)
            {
                select(recvfd + 1,NULL,&wset,NULL,NULL);
                if(FD_ISSET(recvfd,&wset))
                {
                        writelength= write(recvfd,buffer,length);

                        length -= writelength;
                        recvsum += writelength;
                }
            }
        }
    }

    printf("--%d--port  read sum is %d -- recv sum is %d\n",index,readsum,recvsum);
    pthread_exit(NULL);
}


void startThread(void* p,int isWrite)
{

    pthread_t tid;
    int index = 0;
    int ret = 0;
    if(!isWrite)
    {
        index = ((ReadWriteThreadArgument*)p)->index;
        ret = pthread_create(&tid, NULL, readWritePort, p);

    }
    else
    {
        index = ((WriteThreadArgument*)p)->index;
        ret = pthread_create(&tid, NULL, writePort, p);
    }
    if(ret)
    {
        printf("create the %d thread failed\n",index);
        pthread_exit(&ret);
    }

}


int main(int argc, const char* argv[])
{
    /**
     *
     * some variables
     *
     */
    int i = 0;
    int loopback = 0;
    int base = 0;
    int numbers = 0;
    int baud = 0;
    int fds[PORT_NUMBERS] = {0};
    char devnames[PORT_NUMBERS][13] = {0};
    char recvfiles[PORT_NUMBERS][20] = {0};
    char loopbackattr[PORT_NUMBERS][64] = {0};
    unsigned char mainloop[5];
    ReadWriteThreadArgument arguments[PORT_NUMBERS] = {0};

     /**
      * some arguments basic deal
      */
   if(argc < 7)
   {
        printf("format is %s <base> <numbers> <sendfilename> <recvfilename> <baud0|1>  <loopback>\n",argv[0]);
        exit(-1);
   }

   base    = atoi(argv[1]);
   numbers = atoi(argv[2]);
   baud    = atoi(argv[5]);
   loopback = atoi(argv[6]);
   if(numbers > 10 || numbers < 1)
   {
        printf("please input the reasonable digital from 1 to 10\n");
        exit(-1);
   }


   for(i = base; i < numbers ; i++)
   {
       snprintf(devnames[i],13,"/dev/ttyUSB%d",i);
       sprintf(recvfiles[i],"%s%d",argv[4],i);
       snprintf(loopbackattr[i],39,"/sys/class/tty/ttyUSB%d/device/loopback",i);
//       printf("devname is %s\n",devnames[i]);
//       printf("recvfile is %s\n",recvfiles[i]);

       arguments[i].index = i;
       arguments[i].baud  = baud;
       arguments[i].loopback = loopback;
       arguments[i].devName = devnames[i];
       arguments[i].readFileName = argv[3];
       arguments[i].writeFileName = recvfiles[i];
       arguments[i].loopbackAttrName = loopbackattr[i];

       startThread(&arguments[i],0);

   }

    /**
     * exit from main thread
     */
    for(;;)
    {
        scanf("%s",mainloop);
        if(strncmp(mainloop,"exit",4) == 0)
        {
            kill(getpid(),SIGINT);
        }

    }

    return 0;
}



