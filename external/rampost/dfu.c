#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <ctype.h>
#include <string.h>

#include "messages.h"
#include "spine_hal.h"
#include "rampost.h"

#define VERSTRING_LEN 16

#define DEBUG_DFU
#ifdef DEBUG_DFU
#define dprint printf
#else
#define dprint(s, ...)
#endif

#define FRAME_WAIT_MS 500 //todo hoist to rampost.h
#define ERASE_WAIT_MS  1000 //up to 1 sec for erase
#define DFU_PACKET_WAIT_MS 1000 //up to 1 sec for write
#define VALIDATE_WAIT_MS  5000 //up to 5 sec for validate

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
  printf("Dfu Error %d\n", err);
//  dfu_cleanup();
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

enum VersionStatus  CheckVersion(const uint8_t desiredVersion[], const uint8_t firmwareVersion[])
{
  ShowVersion("Installed", firmwareVersion);
  ShowVersion("Provided", desiredVersion);

  uint8_t i;
  for (i=0;i<VERSTRING_LEN;i++) {
    if (desiredVersion[i] != firmwareVersion[i]) {
       return (desiredVersion[i] < firmwareVersion[i]) ? version_NEWER : version_OLDER;
    }
  }
  return version_OK;  // all digits same
}


int IsGoodAck(struct AckMessage* msg)
{
   if (msg->status <= 0) {
    dprint("NACK = %d\n", msg->status);
    return 0;
   }
   return 1;
}


bool SendData(const struct WriteDFU* packet) {

  const struct SpineMessageHeader* hdr;

  //wait for boot frame to be sure we are ready to continue
  hdr = hal_wait_for_frame(PAYLOAD_BOOT_FRAME, FRAME_WAIT_MS);
  if (!hdr) {
    //No boot frames means something wrong
    dprint("SendData not seeing boot frames\n");
    return false;
  }

  hal_send_frame(PAYLOAD_DFU_PACKET, packet, sizeof(struct WriteDFU));

  hdr = hal_wait_for_frame(PAYLOAD_ACK, DFU_PACKET_WAIT_MS);
  if (!hdr) {
    dprint("Send Data did not get response\n");
    return false;
  }
  else if (!IsGoodAck((struct AckMessage*)(hdr + 1))) {
    //Noone should send NACK
    dprint("Got NACK in response to DFU PACKET\n");
    return false;
  }
  return true;
}


static const char* const ERASED_INDICATOR = "-----Erased-----";
static uint8_t gInstalledVersion[VERSTRING_LEN] = {0};

const uint8_t* dfu_get_version()
{
  dprint("requesting installed version\n");
  const struct SpineMessageHeader* hdr;

  hal_send_frame(PAYLOAD_VERSION, NULL, 0);


  hdr = hal_wait_for_frame(PAYLOAD_VERSION, FRAME_WAIT_MS*2);
  if (hdr) {
    memcpy(gInstalledVersion, ((struct VersionInfo*)(hdr + 1))->app_version, VERSTRING_LEN);
    return gInstalledVersion;
  }

  //Any other response, including NAK from bootloader, we assume erased and continue.
  //If it was not really ready, the next steps will fail.
  memcpy(gInstalledVersion, ERASED_INDICATOR, VERSTRING_LEN);
  return gInstalledVersion;
}

uint8_t versionString[VERSTRING_LEN];

const uint8_t* dfu_open_binary_file(const char* filename, FILE** fpOut) {
  dprint("opening file %s\n",filename);
  *fpOut = fopen(filename, "rb");
  if (!*fpOut) {
    show_error(err_DFU_FILE_OPEN);
    return NULL;
  }
  dprint("reading image version\n");
  size_t nchars = fread(versionString, sizeof(uint8_t), VERSTRING_LEN, *fpOut);
  if (nchars != VERSTRING_LEN) {
    show_error(err_DFU_FILE_READ);
    return NULL;
  }

  return versionString;
}

bool dfu_erase_image(void) {
  const struct SpineMessageHeader* hdr;

  dprint("erasing installed image\n");

  hal_send_frame(PAYLOAD_ERASE, NULL, 0);

  hdr = hal_wait_for_frame(PAYLOAD_ACK, ERASE_WAIT_MS*5);
  if (!hdr) {
    //No ack means something wrong
    dprint("Erase timeout\n");
    return false;
  }
  if (hdr) {
    if (!IsGoodAck((struct AckMessage*)(hdr + 1))) {
      //Should not fail
      dprint("Got NACK in response to erase\n");
      return false;
    }
  }
  return true;
}

bool dfu_send_image(FILE* imgfile, uint64_t expire_time) {
  struct WriteDFU packet = {0};
  size_t databytes;
  size_t itemcount;
  int start_addr = 0;

  while (!feof(imgfile)) {
    itemcount = fread(&(packet.data[0]), sizeof(uint32_t), 256, imgfile);
    databytes = itemcount * sizeof(uint32_t);
    dprint("read %zd words (%zd bytes)\n", itemcount, databytes);

    if (itemcount < 256 && ferror(imgfile)) {
      show_error(err_DFU_FILE_READ);
      return false;
    }

    if (itemcount  > 0) {
      packet.address = start_addr;
      packet.wordCount = itemcount;

      dprint("writing %d words (%zd bytes) for @%x\n", packet.wordCount, databytes, packet.address);

      if (!SendData(&packet)) {
        show_error(err_DFU_SEND);
        return false;
      }

      start_addr += databytes;
    }
    else {
      dprint("send_data got itemcount of %zd\n", itemcount);
      return false;
    }
  }
  return true;
}

bool dfu_validate_image(void) {
  dprint("validating installed image\n");

  const struct SpineMessageHeader* hdr;

  //wait for boot frame to be sure we are ready to continue
  hdr = hal_wait_for_frame(PAYLOAD_BOOT_FRAME, FRAME_WAIT_MS);
  if (!hdr) {
    //No boot frames means something wrong
    dprint("validate_image not seeing boot frames\n");
    return false;
  }

  hal_send_frame(PAYLOAD_VALIDATE, NULL, 0);

  hdr = hal_wait_for_frame(PAYLOAD_ACK, VALIDATE_WAIT_MS);
  if (!hdr) {
    dprint("Validate did not get ACK response\n");
  }
  else if (!IsGoodAck((struct AckMessage*)(hdr + 1))) {
    //Noone should send NACK
    dprint("Got NACK in response to PAYLOAD_ACK\n");
    return false;
  }
  //else good ack, but let's wait for a app data frame to be sure
  hdr = hal_wait_for_frame(PAYLOAD_DATA_FRAME, FRAME_WAIT_MS);
  if (!hdr) {
    dprint("Validate did not get app DATA_FRAME response\n");
    return false;
  }
  //else we are up and running
  return true;

}

RampostErr dfu_sequence(const char* dfu_file, const uint64_t timeout, bool force_update)
{
  const uint64_t expire_time = steady_clock_now() + timeout;

  const uint8_t* installed_version = dfu_get_version();
  if (!installed_version) {
    return err_DFU_NO_VERSION;
  }

  const uint8_t* desired_version = dfu_open_binary_file(dfu_file, &gImgFilep);

  if (!force_update && CheckVersion(desired_version, installed_version) >= version_OK) {
    return err_SYSCON_VERSION_GOOD; //no update needed
  }
  //else needs update

  if (!dfu_erase_image())
  {
    return err_DFU_ERASE_ERROR;
  }

  if (!dfu_send_image(gImgFilep, expire_time))
  {
    return err_DFU_INSTALL_ERROR;
  }

  if (!dfu_validate_image())
  {
    return err_DFU_VALIDATE_ERROR;
  }

  dprint("Success!\n");

  return err_OK; //Updated
}
