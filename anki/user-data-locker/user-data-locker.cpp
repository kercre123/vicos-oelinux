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
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#include "keymaster.h"
#include "log.h"

const std::vector<uint8_t> kDataToSign = { 0x61, 0x6e, 0x6b, 0x69, 0x64, 0x61, 0x74, 0x61 };
const std::string kUserDataLockerBlockDevice = "/dev/block/bootdevice/by-name/switchboard";
const size_t kUserDataLockerBlockLen = 262144; // 256kb should be more than enough
const uint32_t kUserDataLockerVersion = 2;
static uint8_t sBlockBuffer[kUserDataLockerBlockLen] = {0};

const size_t kMaxKeyBlobLength = 8192;

struct __attribute__((packed)) UserDataLockerInfo {
  uint8_t magic[8];
  uint32_t version;
  bool hasKeyBlob;
  uint32_t keyBlobLength;
  uint8_t keyBlob[kMaxKeyBlobLength];
};

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

static bool IsUserDataLockerInfoValid()
{
  // check magic
  struct UserDataLockerInfo *info = reinterpret_cast<struct UserDataLockerInfo*>(sBlockBuffer);
  uint8_t* m = info->magic;

  if (m[0] == 'A' && m[1] == 'N' && m[2] == 'K' && m[3] == 'I' &&
      m[4] == 'U' && m[5] == 'D' && m[6] == 'L' && m[7] == 'I' &&
      info->version == kUserDataLockerVersion) {
    if (info->hasKeyBlob && !info->keyBlobLength) {
      return false;
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

  ssize_t bytesRead = readn(fd, sBlockBuffer, sizeof(sBlockBuffer));

  int r = close(fd);
  if ((bytesRead <= 0) || (r == -1)) {
    return -1;
  }


  if (bytesRead < (ssize_t) sizeof(struct UserDataLockerInfo)) {
    return -1;
  }

  return IsUserDataLockerInfoValid() ? 0 : -1;
}

static int WriteUserDataLocker(const std::vector<uint8_t>& keyBlob)
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

  info->hasKeyBlob = !keyBlob.empty();
  if (info->keyBlob) {
    info->keyBlobLength = std::min(keyBlob.size(), sizeof(info->keyBlob));
    memcpy(info->keyBlob, keyBlob.data(), info->keyBlobLength);
  }

  ssize_t bytesWritten = writen(fd, sBlockBuffer, sizeof(sBlockBuffer));

  int r = close(fd);
  if ((bytesWritten <= 0) || (r == -1)) {
    return -1;
  }

  return 0;
}

static std::vector<uint8_t> GetKeyBlob()
{
  struct UserDataLockerInfo* info = reinterpret_cast<struct UserDataLockerInfo*>(sBlockBuffer);
  if (info->hasKeyBlob) {
    std::vector<uint8_t> keyBlob(info->keyBlob, info->keyBlob + info->keyBlobLength);
    return keyBlob;
  } else {
    std::vector<uint8_t> empty;
    return empty;
  }
}

static std::string GetPassPhraseFromSignature(const std::vector<uint8_t>& signature)
{
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  std::vector<uint8_t>::const_iterator it;

  for (it = signature.begin(); it != signature.end(); it++) {
    oss << std::setw(2) << static_cast<unsigned>(*it);
  }

  return oss.str();
}

//
// How does this all work?
//
// "Reset" (aka Generating a key blob)
// 1. We ask Qualcomm's TrustZone to load the KeyMaster TrustLet (TODO: point to the keymaster trustlet)
// 2. The KeyMaster TrustLet runs securely inside the chip's TrustZone
// 3. We ask the KeyMaster TrustLet to generate an RSA key pair (1024bit)
// 4. KeyMaster returns a key blob to us containing the public key and an
//    encrypted form of the private key.  Only TrustZone can decrypt and
//    use the private key for operations.    (TODO: Investigate how this is encrypted by TZ)
// 5. We store this key blob on the switchboard partition for use in subsequent
//    runs.
// 6. We take a fixed array of bytes and ask the KeyMaster trustlet to digitally sign it using the key blob
// 7. The KeyMaster TrustLet securely decrypts the private key from the key
//    blob and uses it to produce a signature
// 8. We convert this signature into a 256 hexadecimal character string and return it (This is the passphrase we use for deriving the key in cryptsetup)
//
// "Read" (aka just sign the data)
// 1. We ask Qualcomm's TrustZone to load the KeyMaster TrustLet
// 2. The KeyMaster TrustLet runs securely inside the chip's TrustZone
// 3. We load the key blob from the switchboard partition that we saved up in step 5.
// 4. Follow steps 6 through 8 above
//
// It is expected that the 256 hexadecimal character string will be used as a passphrase
// for cryptsetup when it establishes a LUKS volume on the "data" partition.  With LUKS,
// cryptsetup will generate a master key for encrypting the volume.  The master key is
// then encrypted using our passphrase.
//
// How is this secure?
// If an attacker got a hold of the robot, they could extract the key blob and discover the
// data that we digitally sign.  However, without the robot's TrustZone, they cannot create
// the same digital signature of the data.  Without the digital signature, they
// don't have the passphrase that is used to decrypt the "data" partition.

int main(int argc, char** argv)
{
  setAndroidLoggingTag("udl");
  enableAndroidLogging(true);

  KeyMaster keyMaster;
  std::string passPhrase;

  if (argc > 1 && !strcmp(argv[1], "reset")) {
    std::vector<uint8_t> keyBlob;
    int r = keyMaster.GenerateKeyBlob(keyBlob);
    if (r < 0) {
      return 2;
    }
    std::vector<uint8_t> signature;
    r = keyMaster.SignData(keyBlob, kDataToSign, signature);
    if (r < 0) {
      return 3;
    }
    passPhrase = GetPassPhraseFromSignature(signature);
    r = WriteUserDataLocker(keyBlob);
    if (r < 0) {
      return 4;
    }
  } else {
    int r = ReadUserDataLocker();
    if (r < 0) {
      return 5;
    }
    std::vector<uint8_t> keyBlob = GetKeyBlob();
    if (keyBlob.empty()) {
      return 6;
    }
    std::vector<uint8_t> signature;
    r = keyMaster.SignData(keyBlob, kDataToSign, signature);
    if (r < 0) {
      return 7;
    }
    passPhrase = GetPassPhraseFromSignature(signature);
  }

  std::cout << passPhrase;
  return 0;
}
