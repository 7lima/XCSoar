/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2011 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

/*
 * This program tries to connect to a TCP server at the localhost at
 * port 4353 and feeds NMEA data read from stdin to it.
 */

#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>

#ifdef HAVE_POSIX
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#else
#include <winsock2.h>
#endif

int main(int argc, char **argv)
{
  int tcp_port;
  if (argc < 2) {
    fprintf(stderr, "Usage: %s PORT\n", argv[0]);
    fprintf(stderr, "Defaulting to port 4353\n");
    tcp_port = 4353;
  } else {
    tcp_port = atoi(argv[1]);
  }

  int sock;
  struct sockaddr_in server_addr;

  if ((server_addr.sin_addr.s_addr = inet_addr("127.0.0.1")) == INADDR_NONE) {
    perror("IP");
    exit(1);
  }

  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
    perror("Socket");
    exit(1);
  }

  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(tcp_port);
  memset(&(server_addr.sin_zero), 0, 8);

  if (connect(sock, (struct sockaddr *)&server_addr,
              sizeof(struct sockaddr)) == -1)
  {
    perror("Connect");
    exit(1);
  }

  char stamp[6] = "";

  char line[1024];
  long baudrate = 9600;
  useconds_t sleep_acc = 0;
  long c_count = 0;
  long l_count = 0;

  while (fgets(line, sizeof(line), stdin) != NULL) {
    int l = strlen(line);
    c_count += l;
    send(sock,line,l, 0);
    long tsleep = l*1e6/(baudrate/10);
    usleep(tsleep);
    sleep_acc += tsleep;

    if (memcmp(line, "$GP", 3) == 0 &&
        (memcmp(line + 3, "GGA", 3) == 0 ||
         memcmp(line + 3, "RMC", 3) == 0) &&
        line[6] == ',' &&
        strncmp(stamp, line + 7, sizeof(stamp)) != 0) {
      /* the time stamp has changed - sleep for one second */
      usleep(1e6-sleep_acc);
      strncpy(stamp, line + 7, sizeof(stamp));
      sleep_acc = 0;
      l_count++;
      printf(".");
      fflush(stdout);
    }
  }
  close(sock);
  printf(">>>> Av %ld\n", c_count/l_count);
  return 0;
}
