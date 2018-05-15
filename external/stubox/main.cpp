/**
 * File: main.cpp
 *
 * Author: seichert
 * Created: 5/14/2018
 *
 * Description: Testing tool
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <string.h>
#include <netdb.h>
#include <net/if.h>


#include <iostream>
#include <random>
#include <string>

#include <cutils/properties.h>

std::string get_ip_address_with_ioctl(const char* if_name) {
  struct ifreq ifr;
  size_t if_name_len=strlen(if_name);
  if (if_name_len<sizeof(ifr.ifr_name)) {
    memcpy(ifr.ifr_name,if_name,if_name_len);
    ifr.ifr_name[if_name_len]=0;
  } else {
    std::cerr << "name is too long" << std::endl;
    return "";
  }

  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd == -1) {
    std::cerr << "socket returned -1" << std::endl;
    return "";
  }
  struct sockaddr_in* sin = (struct sockaddr_in *)&ifr.ifr_addr;
  memset(sin, 0, sizeof(*sin));
  sin->sin_family = AF_INET;
  sin->sin_port = 0;
  (void) inet_aton("220.152.205.127", &(sin->sin_addr));
  if (ioctl(fd, SIOCGIFADDR,&ifr)==-1) {
    std::cerr << "ioctl returned " << strerror(errno) << std::endl;
  }
  close(fd);
  return std::string(inet_ntoa(sin->sin_addr));
}

std::string get_ip_address_with_getifaddrs(const char* if_name) {
  struct ifaddrs* ifaddr = nullptr;
  struct ifaddrs* ifa = nullptr;
  char host[NI_MAXHOST] = {0};

  int rc = getifaddrs(&ifaddr);
  if (rc == -1) {
    std::cerr << "getifaddrs returned " << strerror(errno) << std::endl;
    return "";
  }

  for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr == nullptr) {
      continue;
    }
    
    if (ifa->ifa_addr->sa_family != AF_INET) {
      continue;
    }

    if (strcmp(ifa->ifa_name, if_name)) {
      continue;
    }

    int s = getnameinfo(ifa->ifa_addr,
                        sizeof(struct sockaddr_in),
                        host, sizeof(host),
                        NULL, 0, NI_NUMERICHOST);
    if (s != 0) {
      std::cerr << "getnameinfo() failed: " << gai_strerror(s) << std::endl;
      return "";
    }
    return std::string(host);
  }

  return "";
}

int main(int argc, char *argv[])
{
  std::string ip_by_ioctl = get_ip_address_with_ioctl("wlan0");
  std::string ip_by_getifaddrs = get_ip_address_with_getifaddrs("wlan0");

  std::cout << "ip address with ioctl for wlan0: " << ip_by_ioctl << std::endl;
  std::cout << "ip address with getifaddrs for wlan0: " << ip_by_getifaddrs << std::endl;
  return 0;
}
