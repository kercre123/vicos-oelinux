/**
 * File: user-data-locker.cpp
 *
 * Author: Alexander Kaplan
 * Created: 8/15/2018
 *
 * Secondary Author: Stuart Eichert
 * Modified: 8/24/2018
 *
 * Description: Store/Retrieve passphrase for encrypting partitions
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#include <cctype>
#include <iostream>
#include <random>
#include <string>

#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

const std::string kUserDataLockerBlockDevice = "/dev/block/bootdevice/by-name/switchboard";
const size_t kUserDataLockerBlockLen = 262144; // 256kb should be more than enough
const uint32_t kUserDataLockerVersion = 1;
static uint8_t sBlockBuffer[kUserDataLockerBlockLen] = {0};

const std::string kSerialNumberPath = "/sys/devices/soc0/serial_number";
const size_t kSerialNumberLength = 9;
static char sSerialNumber[kSerialNumberLength] = {0};

const size_t kPassPhraseLength = 65;

struct __attribute__((packed)) UserDataLockerInfo {
  uint8_t magic[8];
  uint32_t version;
  bool hasPassPhrase;
  char passPhrase[kPassPhraseLength + 1];
};


void xordata(size_t length, char *src, char *dest, size_t key_length, char *key)
{
  size_t i,j;
  for (i=0, j=0; i<length; i++, j++) {
    if (j>=key_length) {
      j=0;
    }
    dest[i] = src[i] ^ key[j];
  }
}

/// Read "n" bytes from a descriptor.
/// adapted from Stevens, "Unix Network Programming"
ssize_t readn(int fd, void *vptr, size_t n)
{
  size_t  nleft;
  ssize_t nread;
  char   *ptr;

  ptr = (char*)vptr;
  nleft = n;
  while (nleft > 0) {
    nread = read(fd, ptr, nleft);
    if (nread  < 0) {
      if (errno == EINTR) {
        nread = 0;      /* and call read() again */
      } else {
        return (-1);
      }
    } else if (nread == 0) {
      break;              /* EOF */
    }

    nleft -= nread;
    ptr += nread;
  }
  return (n - nleft);         /* return >= 0 */
}

/// readn bytes from a descriptor and then xor it
ssize_t readn_xor(int fd, void *vptr, size_t n, size_t key_length, char *key)
{
  ssize_t bytesRead = readn(fd, vptr, n);
  if (bytesRead >= 0) {
    xordata((size_t) bytesRead, (char *) vptr, (char *) vptr, key_length, key);
  }
  return bytesRead;
}

/// Write "n" bytes to a descriptor
/// adapted from Stevens, "Unix Network Programming"
ssize_t
writen(int fd, const void *vptr, size_t n)
{
  size_t nleft;
  ssize_t nwritten;
  const char *ptr;

  ptr = (char*)vptr;
  nleft = n;
  while (nleft > 0) {
    nwritten = write(fd, ptr, nleft);
    if (nwritten <= 0) {
      if (nwritten < 0 && errno == EINTR) {
        nwritten = 0;   /* and call write() again */
      } else {
        return (-1);    /* error */
      }
    }

    nleft -= nwritten;
    ptr += nwritten;
  }
  return (n);
}

/// XOR data and then writen it to a descriptor
ssize_t
writen_xor(int fd, const void* vptr, size_t n, size_t key_length, char *key)
{
  char* xorbuf = (char *) malloc(n);
  if (!xorbuf) {
    return -1;
  }
  xordata(n, (char *) vptr, xorbuf, key_length, key);
  ssize_t bytesWritten = writen(fd, xorbuf, n);
  free(xorbuf); xorbuf = nullptr;
  return bytesWritten;
}

static bool IsUserDataLockerInfoValid()
{
  // check magic
  struct UserDataLockerInfo *info = reinterpret_cast<struct UserDataLockerInfo*>(sBlockBuffer);
  uint8_t* m = info->magic;

  if (m[0] == 'A' && m[1] == 'N' && m[2] == 'K' && m[3] == 'I' &&
      m[4] == 'U' && m[5] == 'D' && m[6] == 'L' && m[7] == 'I' &&
      info->version == kUserDataLockerVersion) {
    if (info->hasPassPhrase) {
      for (size_t i = 0 ; i < kPassPhraseLength; i++) {
        if (!std::isalnum(static_cast<unsigned char>(info->passPhrase[i]))) {
          return false;
        }
      }
      return true;
    }
    return true;
  } else {
    return false;
  }
}

static int ReadUserDataLocker()
{
  int fd = open(kUserDataLockerBlockDevice.c_str(), O_RDONLY);
  if (fd == -1) {
    return -1;
  }

  off_t seekResult = lseek(fd, -kUserDataLockerBlockLen, SEEK_END);
  if (seekResult < 0) {
    close(fd);
    return -1;
  }

  ssize_t bytesRead = readn_xor(fd, sBlockBuffer, sizeof(sBlockBuffer), sizeof(sSerialNumber), sSerialNumber);

  int r = close(fd);
  if ((bytesRead <= 0) || (r == -1)) {
    return -1;
  }


  if (bytesRead < (ssize_t) sizeof(struct UserDataLockerInfo)) {
    return -1;
  }

  return IsUserDataLockerInfoValid() ? 0 : -1;
}

static int ReadSerialNumber()
{
  int fd = open(kSerialNumberPath.c_str(), O_RDONLY);
  if (fd == -1) {
    return -1;
  }

  ssize_t bytesRead = readn(fd, sSerialNumber, sizeof(sSerialNumber));
  int r = close(fd);
  if ((bytesRead <= 0) || (r == -1)) {
    return -1;
  }

  return 0;
}

static int WriteUserDataLocker(const std::string& passPhrase)
{
  int fd = open(kUserDataLockerBlockDevice.c_str(), O_WRONLY);
  if (fd == -1) {
    return -1;
  }

  off_t seekResult = lseek(fd, -kUserDataLockerBlockLen, SEEK_END);
  if (seekResult < 0) {
    close(fd);
    return -1;
  }

  struct UserDataLockerInfo* info = reinterpret_cast<struct UserDataLockerInfo*>(sBlockBuffer);
  if (!IsUserDataLockerInfoValid()) {
    info->magic[0] = 'A';
    info->magic[1] = 'N';
    info->magic[2] = 'K';
    info->magic[3] = 'I';
    info->magic[4] = 'U';
    info->magic[5] = 'D';
    info->magic[6] = 'L';
    info->magic[7] = 'I';
    info->version = kUserDataLockerVersion;
  }

  info->hasPassPhrase = !passPhrase.empty();
  if (info->hasPassPhrase) {
    (void) passPhrase.copy(info->passPhrase, sizeof(info->passPhrase) - 1);
  }

  ssize_t bytesWritten = writen_xor(fd, sBlockBuffer, sizeof(sBlockBuffer),
                                    kSerialNumberLength, sSerialNumber);

  int r = close(fd);
  if ((bytesWritten <= 0) || (r == -1)) {
    return -1;
  }

  return 0;
}

static std::string GenerateRandomPassPhrase(std::string::size_type length)
{
  // Per `man cryptsetup`, we want to only use 7-bit ASCII printable values
  // for the passphrase.  Furthermore, to keep things simple, we only use
  // characters from the [A-Za-z0-9] set.  This should still provide sufficient
  // entropy to make the passphrase prohibitively expensive to crack.
  static auto& chars = "0123456789"
      "abcdefghijklmnopqrstuvwxyz"
      "ABCDEFGHIJKLMNOPQRSTUVWXYZ";

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<std::string::size_type> pick(0, sizeof(chars) - 2);


  std::string passPhrase;
  passPhrase.reserve(length);

  for (std::string::size_type i = 0 ; i < length ; i++) {
    passPhrase.push_back(chars[pick(gen)]);
  }

  return passPhrase;
}

static std::string GetPassPhrase()
{
  struct UserDataLockerInfo* info = reinterpret_cast<struct UserDataLockerInfo*>(sBlockBuffer);
  if (!info->hasPassPhrase) {
    return "";
  } else {
    return std::string(info->passPhrase, kPassPhraseLength);
  }
}

int main(int argc, char** argv)
{
  int r = ReadSerialNumber();
  if (r < 0) {
    return 1;
  }

  std::string passPhrase;

  if (argc > 1 && !strcmp(argv[1], "reset")) {
    passPhrase = GenerateRandomPassPhrase(kPassPhraseLength);
    r = WriteUserDataLocker(passPhrase);
    if (r < 0) {
      return 2;
    }
  } else {
    r = ReadUserDataLocker();
    if (r < 0) {
      return 3;
    }
    passPhrase = GetPassPhrase();
  }

  std::cout << passPhrase;
  return 0;
}
