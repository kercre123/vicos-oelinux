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
  for (i=0;i<16;i++) {
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

const struct SpineMessageHeader* SendCommand(uint16_t ctype, const void* data, int size, int retries)
 {
  do {
    printf("Sending command %x\n", ctype);
    hal_send_frame(ctype, data, size);

    const struct SpineMessageHeader* hdr = hal_wait_for_frame(PAYLOAD_ACK,500);
    if (hdr) {
	if (IsGoodAck((struct AckMessage*)(hdr + 1))) {
           return hdr;
        }
        else {
           return NULL; //got a nack
       }
    }
    else {
      printf("Retrying, %d left\n", retries);
    }
  }
  while (retries-- > 0);
  return NULL;
}


void SendData(FILE* imgfile, int start_addr, const uint64_t expire_time)
{
//  int retry = 0;
  struct WriteDFU packet = {0};
  size_t databytes;
  size_t itemcount;
  while (!feof(imgfile) && steady_clock_now() < expire_time) {
//    if (!retry) {
      itemcount = fread(&(packet.data[0]), sizeof(uint32_t), 256, imgfile);
      databytes = itemcount * sizeof(uint32_t);
      dprint("read %zd words (%zd bytes)\n", itemcount, databytes);
//    }
//    else { dprint("resending last packet\n"); }

    if (itemcount < 256) {
      if (ferror(imgfile)) {
        show_error(err_DFU_FILE_READ);
      }
    }

    if (itemcount  > 0) {
      packet.address = start_addr;
      packet.wordCount = itemcount;

      dprint("writing %d words (%zd bytes) for @%x\n", packet.wordCount, databytes, packet.address);
//      dprint("\t[%x ...  %x]\n", packet.data[0], packet.data[packet.wordCount - 1]);

      if (SendCommand(PAYLOAD_DFU_PACKET, &packet, sizeof(packet), 1) == NULL) {
        show_error(err_DFU_SEND);
        return;
//        if (++retry > 3) {printf("giving up dfu\n"); return; }
	//try again?
      }
      else {
         start_addr += databytes;

      }

    }
  }
  return;
}

#define FRAME_WAIT_MS 500 //todo hoist to rampost.h

bool SendData2(const struct WriteDFU* packet, uint64_t expire_time) {
  bool acked = false;
  hal_send_frame(PAYLOAD_DFU_PACKET, packet, sizeof(struct WriteDFU));

  do {
    const struct SpineMessageHeader* hdr = hal_get_next_frame(FRAME_WAIT_MS);
    if (hdr) {
      switch (hdr->payload_type) {
        case PAYLOAD_ACK:
          if (IsGoodAck((struct AckMessage*)(hdr + 1))) {
            acked = true;
          }
          else {  //Noone should send NACK
            dprint("Got NACK in response to DFU PACKET\n");
            return false;
          }
          break;

        case PAYLOAD_BOOT_FRAME:
          if (acked) {
            return true; //Confirmed in bootloader
          }
          break;

        default:
          dprint("Got %04x in response to Payload message\n", hdr->payload_type);
          return false;
          break;
      }
    }
  } while (steady_clock_now() < expire_time);

  dprint("Send Data expired\n");
  return false;
}

static const char* const ERASED_INDICATOR = "-----Erased-----";

int dfu_if_needed(const char* dfu_file, const uint64_t timeout)
{
  const uint64_t expire_time = steady_clock_now() + timeout;
  dprint("requesting installed version\n");

  hal_send_frame(PAYLOAD_VERSION, NULL, 0);

  const uint8_t* version_ptr = NULL;
  const struct SpineMessageHeader* hdr;
  int version_retries = 10;
  int version_resends = 3;
  do {
    hdr = hal_get_next_frame(timeout/1000000);
    if (hdr && hdr->payload_type == PAYLOAD_ACK) {
      if (!IsGoodAck((struct AckMessage*)(hdr + 1))) {
        version_ptr = (const uint8_t*)ERASED_INDICATOR;
      }
    }
    else if (hdr && hdr->payload_type == PAYLOAD_VERSION) {
      version_ptr = ((struct VersionInfo*)(hdr + 1))->app_version;
    }
    else { //if (hdr && hdr->payload_type == PAYLOAD_BOOT_FRAME) {
      if (version_retries--<=0)  {
         if (version_resends-->0) {
           hal_send_frame(PAYLOAD_VERSION, NULL, 0);
           version_retries = 10;
          }
          else {
             show_error(err_SYSCON_READ);
              return 0;
          }
      }
    }
    if (steady_clock_now() > expire_time) return 0;
  } while (!version_ptr);

  dprint("opening file\n");
  gImgFilep = fopen(dfu_file, "rb");
  if (!gImgFilep) {
    show_error(err_DFU_FILE_OPEN);
    return 0;
  }

  dprint("reading image version\n");
  uint8_t versionString[VERSTRING_LEN];
  size_t nchars = fread(versionString, sizeof(uint8_t), VERSTRING_LEN, gImgFilep);
  if (nchars != VERSTRING_LEN) {
    show_error(err_DFU_FILE_READ);
    return 0;
  }

  if (CheckVersion(versionString, version_ptr) >= version_OK) {
    return 1; //no error
  }
  //else needs update

    dprint("erasing installed image\n");
    if (SendCommand(PAYLOAD_ERASE, NULL, 0, 1) == NULL) {
      show_error(err_SYSCON_ERASE);
      return 0;
    }

  SendData(gImgFilep, 0, expire_time);

  uint64_t reboot_time = steady_clock_now() + 1000000; //one sec
  while (steady_clock_now() < reboot_time) {
    hdr = hal_get_next_frame(0);
  }

  dprint("requesting validation\n");
  if (SendCommand(PAYLOAD_VALIDATE, NULL, 0, 1) == NULL) {
    show_error(err_SYSCON_VALIDATE);
    return 1; //keep going
  }

  printf("Success!\n");
  return 1; //Updated
}

const uint8_t* dfu_get_version(uint64_t expire_time)
{
  dprint("requesting installed version\n");

  hal_send_frame(PAYLOAD_VERSION, NULL, 0);

  const uint8_t* version_ptr = NULL;
  const struct SpineMessageHeader* hdr;

  do {
    hdr = hal_get_next_frame(0);
    if (hdr) {
      switch (hdr->payload_type) {
        case PAYLOAD_ACK:
          //bootloader sends NAK
          if (!IsGoodAck((struct AckMessage*)(hdr + 1))) {
            version_ptr = (const uint8_t*)ERASED_INDICATOR;
            return version_ptr;
          }
          else {  //Noone shold send ACK
            dprint("Got ACK in response to version request\n");
            return NULL;
          }
          break;

        case PAYLOAD_VERSION:
          version_ptr = ((struct VersionInfo*)(hdr + 1))->app_version;
          return version_ptr;
          break;

        case PAYLOAD_DATA_FRAME:
        case PAYLOAD_BOOT_FRAME:
          //we will get some of these if it is running normally
	   break;

        default:
          dprint("Got %04x in response to version request\n", hdr->payload_type);
          return NULL;
          break;
      }
    }
  } while (steady_clock_now() < expire_time);
  dprint("Get Version never responded");
  return NULL;
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

bool dfu_erase_image(const uint64_t expire_time) {
  dprint("erasing installed image\n");

  const struct SpineMessageHeader* hdr = hal_get_next_frame(FRAME_WAIT_MS);
  if (!hdr) {
    //we should have heard something...
    return false;
  }
  hal_send_frame(PAYLOAD_ERASE, NULL, 0);

  bool acked = false;

  do {
    const struct SpineMessageHeader* hdr = hal_get_next_frame(FRAME_WAIT_MS);
    if (hdr) {
      switch (hdr->payload_type) {
        case PAYLOAD_ACK:
          if (IsGoodAck((struct AckMessage*)(hdr + 1))) {
            acked = true;
          }
          else {  //Noone shold send NACK
            dprint("Got NACK in response to erase\n");
            return false;
          }
          break;

        case PAYLOAD_BOOT_FRAME:
          if (acked) {
            return true; //Confirmed in bootloader
          }
          break;

        default:
          dprint("Got %04x in response to Erase message\n", hdr->payload_type);
          return false;
          break;
      }
    }
  } while (steady_clock_now() < expire_time);
  dprint("Erase timeout\n");
  return false;
}

bool dfu_send_image(FILE* imgfile, uint64_t expire_time) {
  struct WriteDFU packet = {0};
  size_t databytes;
  size_t itemcount;
  int start_addr = 0;

  while (!feof(imgfile)) {
    if (steady_clock_now() < expire_time) {
      dprint("dfu send timeout\n");
      return false;
    }
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

      if (!SendData2(&packet, expire_time)) {
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

bool dfu_validate_image(uint64_t expire_time) {
  dprint("validating installed image\n");

  const struct SpineMessageHeader* hdr = hal_get_next_frame(FRAME_WAIT_MS);
  if (!hdr) {
    //we should have heard something...
    return false;
  }
  hal_send_frame(PAYLOAD_VALIDATE, NULL, 0);

  bool acked = false;

  do {
    const struct SpineMessageHeader* hdr = hal_get_next_frame(FRAME_WAIT_MS);
    if (hdr) {
      switch (hdr->payload_type) {
        case PAYLOAD_ACK:
          if (IsGoodAck((struct AckMessage*)(hdr + 1))) {
            acked = true;
          }
          else {  //Noone should send NACK
            dprint("Got NACK in response to validate\n");
            return false;
          }
          break;

        case PAYLOAD_DATA_FRAME:
          dprint("Application started %s validation ack\n",acked?"after":"without");
          return true;
          break;

        case PAYLOAD_BOOT_FRAME:
          //ignore any that occur before reboot.
          break;

        default:
          dprint("Got %04x in response to Validate message\n", hdr->payload_type);
          return false;
          break;
      }
    }
  } while (steady_clock_now() < expire_time);
  dprint("Validate timeout\n");
  return false;

}

RampostErr dfu_sequence(const char* dfu_file, const uint64_t timeout)
{
  const uint64_t expire_time = steady_clock_now() + timeout;

  const uint8_t* installed_version = dfu_get_version(expire_time);
  if (!installed_version) {
    return err_DFU_NO_VERSION;
  }

  const uint8_t* desired_version = dfu_open_binary_file(dfu_file, &gImgFilep);

  if (CheckVersion(desired_version, installed_version) >= version_OK) {
    return err_SYSCON_VERSION_GOOD; //no update needed
  }
  //else needs update

  if (!dfu_erase_image(expire_time)) {
    return err_DFU_ERASE_ERROR;
  }

  if (!dfu_send_image(gImgFilep, expire_time))
  {
    return err_DFU_INSTALL_ERROR;
  }

  if (!dfu_validate_image(expire_time))
  {
    return err_DFU_VALIDATE_ERROR;
  }


  printf("Success!\n");
  return err_OK; //Updated
}
