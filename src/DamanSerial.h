#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include "stdio.h"

int open_port();
void dputchar(char c,int fd);
//void dgetchar(char& c, int fd);
//void readSerial(char& c, int fd);
void sendCommands(const int& controlX, const int& controlY, int fd);
