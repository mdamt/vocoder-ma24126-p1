// VOCODER MP24126-P1
// gcc -o vocoder vocoder.c

#include <termios.h>
#include <strings.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>

#define BAUDRATE B115200
#ifndef SERIAL_DEVICE
#define SERIAL_DEVICE "/dev/ttyUSB0"
#endif
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

#define SETUP_ENCODING      1
#define SETUP_WORKING_MODE  2
#define SETUP_UART          4
#define SETUP_CODEC         8
#define SETUP_VAD          16
#define SETUP_ENCODER      32
#define SETUP_DECODER      64
#define SETUP_CHECKSUM    128

#define PACKET_LENGTH      16

typedef unsigned char* packet;
typedef unsigned char* payload;

int echo_write; 

#define ECHO_DELAY 10
unsigned char echo[PACKET_LENGTH * ECHO_DELAY];

void
build_frame(packet p, unsigned char command,
        unsigned char length,
        payload l) {
  unsigned int check_total = 0;
  unsigned int i;

  if (length > PACKET_LENGTH)
    return;

  p[0] = 0x4c; 
  p[1] = 0x4e; 
  p[2] = command;
  p[3] = length;

  void *s = (void *) &p[4];
  memcpy(s, l, length);

  for (i = 0; i < PACKET_LENGTH; i ++) {
    check_total += p[i];
  }
  p[PACKET_LENGTH] = (unsigned char) check_total;
}

void
setup_vocoder(const int fd) {
  unsigned char packet[16];
  unsigned char payload[11];
  int i;

  packet[0] = 121;
  payload[0]  = 0 | 
                SETUP_ENCODING | 
                SETUP_WORKING_MODE | 
                SETUP_UART | 
                SETUP_CODEC | 
                SETUP_ENCODER | 
                SETUP_DECODER | 
                SETUP_CHECKSUM
                ;
  payload[1]  = 1;
  payload[2]  = 0;
  payload[3]  = 0;
  payload[4]  = 0;
  payload[5]  = 0;
  payload[6]  = 0;
  payload[7]  = 0;
  payload[8]  = 1;
  payload[9]  = 1;
  payload[10] = 1;

  build_frame(packet, 0, sizeof(payload), payload);
  printf("Packet setup:", fd);
  for (i = 0; i < sizeof(packet); i ++) {
    printf("-%x", packet[i]);
  }
  printf("\n", fd);

  write(fd, packet, sizeof(packet));
}

void
process_voice(int fd_write, packet p) {
  int length = 7;

  unsigned char data[PACKET_LENGTH];

  void *s = (void *) &p[4];
  memcpy(data, s, length);

  printf("Got voice packet %d/%d\n", echo_write, ECHO_DELAY);
  write(fd_write, p, PACKET_LENGTH);

  if (echo_write >= ECHO_DELAY) {
    int i;
    echo_write = 0;
    printf("Replay voice\n");
    for (i = 0; i < PACKET_LENGTH * ECHO_DELAY; i ++) {
      write(fd_write, (void *) &echo[i * ECHO_DELAY], PACKET_LENGTH);
    }
  } else {
    memcpy(p, (void *) &echo[echo_write * PACKET_LENGTH], PACKET_LENGTH); 
    echo_write ++;
  }
}

void
process_frame(int fd_write, packet p, int length) {
  if (length != PACKET_LENGTH)
    return;

  if (p[2] == 0x01) {
    process_voice(fd_write, p);
  }
}


int
main()
{
  int fd, res;
  struct termios newtio;
  unsigned char buf[255];
  unsigned char packet[16];
  fd_set read_fds; 
  struct timeval timeout;
  timeout.tv_sec = 5;
  timeout.tv_usec = 0;

  echo_write = 0;

  fd = open(SERIAL_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd <0) {perror(SERIAL_DEVICE); exit(-1); }

  bzero(&newtio, sizeof(newtio));
  cfmakeraw(&newtio);
  cfsetospeed (&newtio, BAUDRATE);
  cfsetispeed (&newtio, BAUDRATE);

  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &newtio);

  setup_vocoder(fd);

  while (1) {
    FD_ZERO(&read_fds);
    FD_SET(fd, &read_fds);
    select(FD_SETSIZE, &read_fds, NULL, NULL, &timeout);
    if (FD_ISSET(fd, &read_fds)) {
      int i;
      usleep(50000);
      res = read(fd, buf, sizeof(buf));
      if (res < 0) {
        exit(0);
      }
      if (res == 16)
        process_frame(fd, buf, res);
      else 
        printf("Got garbled data with length: %d\n", res);
    }
  }
}
