#ifndef __MESSAGING_H__
#define __MESSAGING_H__

extern int sendMessageRect(const char *ipAddr, float distance, float angle);
extern int sendMessage(const char *ipAddr, char* buffer);

#endif
