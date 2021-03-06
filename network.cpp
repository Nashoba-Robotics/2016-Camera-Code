#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>

#include <sys/socket.h>
#include <resolv.h>
#include <arpa/inet.h>
#include "network.h"

#define BUFFERSIZE		1024

// Print out information about the target to the console
// Send a message about the target to the roborio
int sendMessage(const char *ipAddr, char* sendbuffer) {
  struct sockaddr_in addr;
  int sd;
  
  if ( (sd = socket(PF_INET, SOCK_DGRAM, 0)) < 0 ) {
    perror("Socket");
    return -1;
  }
  addr.sin_family = AF_INET;
  addr.sin_port = htons(1768);
  if ( inet_aton(ipAddr, &addr.sin_addr) == 0 ) {
    perror(ipAddr);
    return -1;
  }
  sendto(sd, sendbuffer, strlen(sendbuffer)+1, 0, (struct sockaddr*)&addr, sizeof(addr));
  close(sd);
  return 0;
}

int sendMessageRect(const char *ipAddr, float distance, float angle) {
  char sendbuffer[BUFFERSIZE];
  sprintf(sendbuffer, "%f:%f", distance, angle);
  return sendMessage(ipAddr, sendbuffer);
}
