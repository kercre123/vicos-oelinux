#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <termios.h>

#include "spine_hal.h"

#include "messages.h"
#include "rampost.h"

#define SPINE_TTY "/dev/ttyHS0"

#include <stdint.h>
typedef uint32_t crc_t;

crc_t calc_crc(const uint8_t* buf, int len);
/* CRC-32: POLY=0x4C11DB7 IN=0xFFFFFFF, REFIN=TRUE, REFOUT=TRUE, XOROUT=0 */

static const crc_t crc_table[256] = {
  0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
  0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
  0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
  0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
  0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
  0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
  0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
  0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
  0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
  0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
  0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
  0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
  0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
  0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
  0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
  0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
  0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
  0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
  0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
  0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
  0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
  0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
  0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
  0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
  0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
  0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
  0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
  0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
  0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
  0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
  0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
  0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

crc_t calc_crc(const uint8_t* buf, int len)
{
  crc_t crc = 0xFFFFffff;
  while (len--) {
    uint8_t idx = (crc ^ *buf++) & 0xff;
    crc = (crc_table[idx] ^ (crc >> 8));
  }
  return crc;
}


#define SKIP_CRC_CHECK 0

#define SPINE_MAX_BYTES 1280

#define BODY_TAG_PREFIX ((uint8_t*)&SyncKey)
#define SPINE_TAG_LEN sizeof(SpineSync)
#define SPINE_PID_LEN sizeof(PayloadId)
#define SPINE_LEN_LEN sizeof(uint16_t)
#define SPINE_HEADER_LEN (SPINE_TAG_LEN+SPINE_PID_LEN+SPINE_LEN_LEN)
#define SPINE_CRC_LEN sizeof(crc_t)

/* static_assert(1, "true"); */
/* static_assert(SPINE_HEADER_LEN == sizeof(struct SpineMessageHeader), "bad define"); */
/* static_assert(SPINE_MAX_BYTES >= SPINE_HEADER_LEN + sizeof(struct BodyToHead) + SPINE_CRC_LEN, "bad define"); */
/* static_assert(SPINE_MAX_BYTES >= SPINE_HEADER_LEN + sizeof(struct HeadToBody) + SPINE_CRC_LEN, "bad_define"); */


static const SpineSync SyncKey = SYNC_BODY_TO_HEAD;

static struct HalGlobals {
  uint8_t inbuffer[SPINE_MAX_BYTES];
  struct SpineMessageHeader outheader;
  int fd;
  int errcount;
} gHal;


/************* Error Handling *****************/
#define spine_error(code, fmt, args...) (printf(fmt, ##args)?(code):(code))
#define CONSOLE_DEBUG_PRINTF
#ifdef CONSOLE_DEBUG_PRINTF
#define spine_debug(fmt, args...)  printf(fmt, ##args)
#else
#define spine_debug(fmt, args...)  (LOGD( fmt, ##args))
#endif

#define EXTENDED_SPINE_DEBUG 0
#if EXTENDED_SPINE_DEBUG
#define spine_debug_x spine_debug
#else
#define spine_debug_x(fmt, args...)
#endif

#define SpineErr int


/************* SERIAL INTERFACE ***************/

static void hal_serial_close()
{
  spine_debug("close(fd = %d)", gHal.fd);
  //TODO: restore?:  tcsetattr(gHal.fd, TCSANOW, &gHal.oldcfg))
  close(gHal.fd);
  gHal.fd = 0;
}


SpineErr hal_serial_open(const char* devicename, long baudrate)
{
  if (gHal.fd != 0) {
    return spine_error(err_ALREADY_OPEN, "hal serial port in use, close other first");
  }

  spine_debug("opening serial port %s\n", devicename);

  gHal.fd = open(devicename, O_RDWR | O_NONBLOCK);
  if (gHal.fd < 0) {
    return spine_error(err_CANT_OPEN_FILE, "Can't open %s", devicename);
  }

  spine_debug("configuring serial port\n");

  /* Configure device */
  {
    struct termios cfg;
    if (tcgetattr(gHal.fd, &cfg)) {
      hal_serial_close();
      return spine_error(err_TERMIOS_FAIL, "tcgetattr() failed");
    }


//?    memcpy(gHal.oldcfg,cfg,sizeof(gHal.oldcfg)); //make backup;

    cfmakeraw(&cfg);
    cfsetispeed(&cfg, baudrate);
    cfsetospeed(&cfg, baudrate);

    cfg.c_cflag |= (CS8 | CSTOPB);    // Use N82 bit words

    spine_debug("configuring port %s (fd=%d)", devicename, gHal.fd);


    if (tcsetattr(gHal.fd, TCSANOW, &cfg)) {
      hal_serial_close();
      return spine_error(err_TERMIOS_FAIL, "tcsetattr() failed");
    }

  }
  spine_debug("serial port OK\n");
  return err_OK;
}


int hal_serial_read(uint8_t* buffer, int len)   //->bytes_recieved
{

  int result = read(gHal.fd, buffer, len);
  if (result < 0) {
    if (errno == EAGAIN) { //nonblocking no-data
      usleep(HAL_SERIAL_POLL_INTERVAL_US); //wait a bit.
      result = 0; //not an error
    }
  }
  return result;
}


int hal_serial_send(const uint8_t* buffer, int len)
{
  if (len) {
    return write(gHal.fd, buffer, len);
  }
  return 0;
}


/************* PROTOCOL SYNC ***************/

enum MsgDir {
  dir_SEND,
  dir_READ,
};

//checks for valid tag, returns expected length, -1 on err
static int get_payload_len(PayloadId payload_type, enum MsgDir dir)
{
  switch (payload_type) {
  case PAYLOAD_MODE_CHANGE:
    return 0;
    break;
  case PAYLOAD_DATA_FRAME:
    return (dir == dir_SEND) ? sizeof(struct HeadToBody) : sizeof(struct BodyToHead);
    break;
  case PAYLOAD_VERSION:
    return (dir == dir_SEND) ? 0 : sizeof(struct VersionInfo);
    break;
  case PAYLOAD_ACK:
    return sizeof(struct AckMessage);
    break;
  case PAYLOAD_ERASE:
    return 0;
    break;
  case PAYLOAD_VALIDATE:
    return 0;
    break;
  case PAYLOAD_DFU_PACKET:
    return sizeof(struct WriteDFU);
    break;
  case PAYLOAD_SHUT_DOWN:
    return 0;
  case PAYLOAD_LIGHT_STATE:
    return sizeof(struct LightState);
  case PAYLOAD_BOOT_FRAME:
    return sizeof(struct MicroBodyToHead);
    break;
  default:
    break;
  }
  return -1;
}




//Creates header for frame
static const uint8_t* spine_construct_header(PayloadId payload_type,  uint16_t payload_len)
{
#ifndef NDEBUG
  int expected_len = get_payload_len(payload_type, dir_SEND);
  assert(expected_len >= 0); //valid type
  assert(expected_len == payload_len);
  assert(payload_len <= (SPINE_MAX_BYTES - SPINE_HEADER_LEN - SPINE_CRC_LEN));
#endif

  struct SpineMessageHeader* hdr = &gHal.outheader;
  hdr->sync_bytes = SYNC_HEAD_TO_BODY;
  hdr->payload_type = payload_type;
  hdr->bytes_to_follow = payload_len;
  return (uint8_t*) hdr;
}


//Function: Examines `buf[idx]` and determines if it is part of sync seqence.
//Prereq: The first `idx` chars in `buf` are partial valid sync sequence.
//Returns: Length of partial valid sync sequence. possible values = 0..idx+1
static int spine_sync(const uint8_t* buf, unsigned int idx)
{
  if (idx < SPINE_TAG_LEN) {
    if (buf[idx] != BODY_TAG_PREFIX[idx]) {
      return 0; //none of the characters so far are good.
    }
  }
  idx++; //accept rest of characters unless proven otherwise
  if (idx == SPINE_HEADER_LEN) {
    struct SpineMessageHeader* candidate = (struct SpineMessageHeader*)buf;
    int expected_len = get_payload_len(candidate->payload_type, dir_READ);
    if (expected_len < 0 || (expected_len != candidate->bytes_to_follow)) {
//      LOGE("spine_header %x %x %x : %d", candidate->sync_bytes,
           /* candidate->payload_type, */
           /* candidate->bytes_to_follow, */
           /* expected_len); */
      //bad header,
      //we need to check the length bytes for the beginning of a sync word
      unsigned int pos = idx - SPINE_LEN_LEN;
      int match = 0;
      while (pos < idx) {
        if (buf[pos] == BODY_TAG_PREFIX[match]) {
          match++;
          pos++;
        }
        else if (match > 0) {  match = 0; }
        else { pos++; }
      }
      // at this point we know that the last `match` chars rcvd into length position are a possible sync tag.
      // but there is no need to copy them to beginning of `buf` b/c we can't get here unless
      //  buf already contains good tag. So just return the number of matches
      return match;
    }
  }
  return idx;
}


/************* PUBLIC INTERFACE ***************/
SpineErr hal_init(const char* devicename, long baudrate)
{
  gHal.errcount = 0;
  gHal.fd = 0;
  SpineErr r = hal_serial_open(devicename, baudrate);
  return r;
}


// Scan the whole payload for sync, to recover after dropped bytes,
int hal_resync_partial(int start_offset, int len) {
   /*  ::: Preconditions :::
       gHal.inbuffer contains len recieved bytes.
       the first start_offset bytes do not need to be scanned
       ::: Postconditions :::
       the first valid sync header and any following bytes up to `len`
       -  or a partial sync header ending at the `len`th byte -
       have been copied to the beginning of gHal.inbuffer.
       returns the number of copied bytes
   */
    unsigned int i;
    unsigned int index = 0;
    for (i = start_offset; i+index < len; ) {
       spine_debug_x(" %02x", gHal.inbuffer[i+index]);
       unsigned int i2 = spine_sync(gHal.inbuffer+i, index);
       if (i2 <= index) { //no match, or restarted match at `i2` chars before last scanned char
          i+=index-i2+1;
       }
       else if (i2 == SPINE_HEADER_LEN) { //whole sync!
          index = len-i; //consider rest of buffer valid.
          break;
       }
       index = i2;
    }
    //at this point we have scanned `i`+`index` chars. the last `index` of them are valid.
    if (index) {
       memmove(gHal.inbuffer, gHal.inbuffer+i, index);
    }
    spine_debug("\n%u dropped bytes\n", start_offset+i-index);

    return index;
}



//gathers most recently queued frame,
//Spins until valid frame header is recieved.
const struct SpineMessageHeader* hal_read_frame()
{
  static unsigned int index = 0;

  //spin here pulling single characters until whole sync rcvd
  while (index < SPINE_HEADER_LEN) {

    int rslt = hal_serial_read(gHal.inbuffer + index, 1);
    if (rslt > 0) {
      index = spine_sync(gHal.inbuffer, index);
    }
    else {
       return NULL; //wait a bit more
    }
  } //endwhile


  //At this point we have a valid message header. (spine_sync rejects bad lengths and payloadTypes)
  // Collect the right number of bytes.
  struct SpineMessageHeader* hdr = (struct SpineMessageHeader*)gHal.inbuffer;
  unsigned int payload_length = hdr->bytes_to_follow;
  unsigned int total_message_length = SPINE_HEADER_LEN + payload_length + SPINE_CRC_LEN;

  spine_debug_x("%d byte payload\n", payload_length);

  while (index < total_message_length) {
    int rslt = hal_serial_read(gHal.inbuffer + index, total_message_length - index);
    //spine_debug_x("%d bytes rcvd\n", rslt);
    if (rslt > 0) {
      index += rslt;
    }
    else if (rslt < 0) {
      if ((gHal.errcount++ & 0x3FF) == 0) { //TODO: at somepoint maybe we handle this?
//        LOGI("spine_payload_read_error %d", rslt);
      }
    }
    else {
       return NULL; //wait a bit more
    }
  }

  spine_debug_x("%d bytes rcvd\n", index);

  //now we just have to validate CRC;
  crc_t expected_crc = *((crc_t*)(gHal.inbuffer + SPINE_HEADER_LEN + payload_length));
  crc_t true_crc = calc_crc(gHal.inbuffer + SPINE_HEADER_LEN, payload_length);
  if (expected_crc != true_crc && !SKIP_CRC_CHECK) {
    spine_debug("\nspine_crc_error: calc %08x vs data %08x\n", true_crc, expected_crc);
//    LOGI("spine_crc_error %08x != %08x", true_crc, expected_crc);


    // Scan the whole payload for sync, to recover after dropped bytes,
    index = hal_resync_partial(SPINE_HEADER_LEN, total_message_length);
    return NULL;
  }

  spine_debug_x("found frame %04x!\r", ((struct SpineMessageHeader*)gHal.inbuffer)->payload_type);
  spine_debug_x("payload start: %08x!\r", *(uint32_t*)(((struct SpineMessageHeader*)gHal.inbuffer)+1));
  index = 0; //get ready for next one
  return ((struct SpineMessageHeader*)gHal.inbuffer);
}


//pulls out frames
const void* hal_get_next_frame(int32_t timeout_ms) {
  timeout_ms *= 1000 / HAL_SERIAL_POLL_INTERVAL_US;
  const struct SpineMessageHeader* hdr;
  do {
    if (timeout_ms>0 && --timeout_ms==0) {
//      LOGE("TIMEOUT in hal_get_next_frame() TIMEOUT");
      return NULL;
    }
    hdr = hal_read_frame();
  } while (!hdr);
  return hdr;
}

const void* hal_get_frame(uint16_t type, int32_t timeout_ms)
{
  while (1) {
    const struct SpineMessageHeader* hdr;
    hdr = hal_get_next_frame(timeout_ms);
    if (!hdr || hdr->payload_type == type) {
      return hdr;
    }
  }
  return NULL;
}

const void* hal_wait_for_frame(uint16_t type)
{
  const void* ret;
  do {
    ret = hal_get_frame(type, INT32_MAX);
  } while (!ret);
  return ret;
}


void hal_send_frame(PayloadId type, const void* data, int len)
{
  const uint8_t* hdr = spine_construct_header(type, len);
  crc_t crc = calc_crc(data, len);
  if (hdr) {
    spine_debug_x("sending %x packet (%d bytes) CRC=%08x\n", type, len, crc);
    hal_serial_send(hdr, SPINE_HEADER_LEN);
    hal_serial_send(data, len);
    hal_serial_send((uint8_t*)&crc, sizeof(crc));
  }
}

void hal_set_mode(int new_mode)
{
  printf("Sending Mode Change %x\n", PAYLOAD_MODE_CHANGE);
  hal_send_frame(PAYLOAD_MODE_CHANGE, NULL, 0);
}

#ifdef STANDALONE_TEST

//gcc -g -DSTANDALONE_TEST -I ../../syscon spine_hal.c spine_crc.c -o spine_test

int main(int argc, const char* argv[])
{
   gHal.fd = open("unittest.dat", O_RDONLY);
   const struct SpineMessageHeader* hdr = hal_get_frame(PAYLOAD_ACK, 1000);
   assert(hdr && hdr->payload_type == PAYLOAD_ACK);
}
#endif
