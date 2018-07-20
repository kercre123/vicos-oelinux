#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <ctype.h>
#include <string.h>

#include "messages.h"
#include "spine_hal.h"
#include "rampost.h"

extern const struct SpineMessageHeader* hal_read_frame();


#define VERSTRING_LEN 16

#define DEBUG_DFU
#ifdef DEBUG_DFU
#define dprint printf
#else
#define dprint(s, ...)
#endif



static void printhex(const uint8_t* bytes, int len)
{
  while (len-- > 0) {
    printf("%02x", *bytes++);
  }
}


enum DfuAppErrorCode {
  app_SUCCESS = 0,
  app_USAGE = -1,
  app_FILE_OPEN_ERROR = -2,
  app_FILE_READ_ERROR = -3,
  app_SEND_DATA_ERROR = -4,
  app_HAL_INIT_ERROR = -5,
  app_FLASH_ERASE_ERROR = -6,
  app_VALIDATION_ERROR = -7,
};

enum VersionStatus {
  version_UNKNOWN,
  version_OLDER,
  version_OK,
  version_NEWER,
};


FILE* gImgFilep = NULL;


void dfu_cleanup(void)
{
   if (gImgFilep) {
      fclose(gImgFilep);
      gImgFilep = NULL;
   }
}

void show_error(RampostErr err) {
  printf("Dfu Error %d\n", RampostErr err);
  dfu_cleanup();
}



void ShowVersion(const char* typename, const uint8_t version[])
{
  int i;
  printf("%s Version = ", typename);
  printhex(version, VERSTRING_LEN);
  printf(" [");
  for (i = 0; i < VERSTRING_LEN; i++) {
    putchar(isprint(version[i]) ? version[i] : '*');
  }
  printf("]\n");
}

enum VersionStatus  CheckVersion(uint8_t desiredVersion[], const uint8_t firmwareVersion[])
{
  ShowVersion("Installed", firmwareVersion);
  ShowVersion("Provided", desiredVersion);

  // TODO: THIS ISN'T ACTUALLY CHECKING
  return version_OLDER;  // for now, always assume older than desired
}


int IsGoodAck(struct AckMessage* msg)
{
   if (msg->status <= 0) {
    dprint("NACK = %d\n", msg->status);
    return 0;
   }
   return 1;
}

const struct SpineMessageHeader* SendCommand(uint16_t ctype, const void* data, int size, int retries)
 {
  do {
    hal_send_frame(ctype, data, size);

    const struct SpineMessageHeader* hdr = hal_wait_for_frame(PAYLOAD_ACK);
    if (IsGoodAck((struct AckMessage*)(hdr + 1))) {
      return hdr;
    }
  }
  while (retries-- > 0);
  return NULL;
}


void SendData(FILE* imgfile, int start_addr)
{
  while (!feof(imgfile)) {

    struct WriteDFU packet = {0};
    size_t itemcount = fread(&(packet.data[0]), sizeof(uint32_t), 256, imgfile);
    size_t databytes = itemcount * sizeof(uint32_t);
    dprint("read %zd words (%zd bytes)\n", itemcount, databytes);

    if (itemcount < 256) {
      if (ferror(imgfile)) {
        show_error(err_DFU_FILE_READ);
      }
    }

    if (itemcount  > 0) {
      packet.address = start_addr;
      packet.wordCount = itemcount;

      dprint("writing %d words (%zd bytes) for @%x\n", packet.wordCount, databytes, packet.address);
      dprint("\t[%x ...  %x]\n", packet.data[0], packet.data[packet.wordCount - 1]);

      if (SendCommand(PAYLOAD_DFU_PACKET, &packet, sizeof(packet), 1) == NULL) {
        show_error(err_DFU_SEND);
      }
      start_addr += databytes;
    }
  }
  return;
}


int dfu_if_needed(const char* dfu_file)
{
  dprint("requesting installed version\n");

  hal_send_frame(PAYLOAD_VERSION, NULL, 0);

  const uint8_t* version_ptr = NULL;
  const struct SpineMessageHeader* hdr;
  do {
     hdr = hal_read_frame();
     if (hdr && hdr->payload_type == PAYLOAD_ACK) {
        if (!IsGoodAck((struct AckMessage*)(hdr + 1))) {
          version_ptr = (const uint8_t*)"-----Erased-----";
        }
     }
     else if (hdr && hdr->payload_type == PAYLOAD_VERSION) {
       version_ptr = ((struct VersionInfo*)(hdr + 1))->app_version;
     }
  } while (!version_ptr);

  dprint("opening file\n");
  gImgFilep = fopen(dfu_file, "rb");
  if (!gImgFilep) {
    show_error(err_DFU_FILE_OPEN);
  }

  dprint("reading image version\n");
  uint8_t versionString[VERSTRING_LEN];
  size_t nchars = fread(versionString, sizeof(uint8_t), VERSTRING_LEN, gImgFilep);
  if (nchars != VERSTRING_LEN) {
    show_error(err_DFU_FILE_READ);
  }

  if (CheckVersion(versionString, version_ptr) >= version_OK) {
    return 0; //no update needed
  }
  //else needs update

  dprint("erasing installed image\n");
  if (SendCommand(PAYLOAD_ERASE, NULL, 0, 3) == NULL) {
    show_error(err_SYSCON_ERASE);
  }

  SendData(gImgFilep, 0);

  dprint("requesting validation\n");
  if (SendCommand(PAYLOAD_VALIDATE, NULL, 0, 3) == NULL) {
    show_error(err_SYSCON_VALIDATE);
  }

  printf("Success!\n");
  return 1; //Updated
}
