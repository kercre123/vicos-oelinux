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

#include "das.h"

#define SPINE_TTY "/dev/ttyHS0"

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


#define SPINE_MAX_BYTES 1280

#define BODY_TAG_PREFIX ((uint8_t*)&SyncKey)
#define SPINE_SYNC_LEN (sizeof(SpineSync))
#define SPINE_HEADER_LEN (sizeof(struct SpineMessageHeader))
#define SPINE_CRC_LEN (sizeof(crc_t))


#define SPINE_B2H_FRAME_LEN (SPINE_HEADER_LEN + sizeof(struct BodyToHead) + SPINE_CRC_LEN)

#define SPINE_BUFFER_MAX_LEN 8192

static const SpineSync SyncKey = SYNC_BODY_TO_HEAD;

static struct HalGlobals {
  uint8_t framebuffer[SPINE_MAX_BYTES]; //for whole frames
  struct SpineMessageHeader outheader;
  int fd;
  uint8_t buf_rx[SPINE_BUFFER_MAX_LEN]; //for incoming bytes
  uint32_t rx_cursor;
  uint32_t rx_size;

  FILE* logFd;

} gHal;


#define EXTENDED_SPINE_DEBUG 0
#if EXTENDED_SPINE_DEBUG
#undef DEBUG_LEVEL
#define DEBUG_LEVEL 0
#endif

/************* SERIAL INTERFACE ***************/

static void hal_serial_close()
{
  DAS_LOG(DAS_INFO, "spine.close", "fd=%d", gHal.fd);
  close(gHal.fd);
  gHal.fd = -1;
}


SpineErr hal_serial_open(const char* devicename, long baudrate)
{
  if (gHal.fd >= 0) {
    DAS_LOG(DAS_ERROR, "spine.already_open", "hal serial port in use, close other first");
    return err_ALREADY_OPEN;
  }

  DAS_LOG(DAS_INFO, "spine.open_serial", "opening serial port %s", devicename);

  gHal.fd = open(devicename, O_RDWR);
  if (gHal.fd < 0) {
    DAS_LOG(DAS_ERROR, "spine.cannot_open", "Cannot open %s", devicename);
    return err_CANT_OPEN_FILE;
  }

  DAS_LOG(DAS_INFO, "spine.configure_serial_port", "configuring port %s (fd=%d)", devicename,
          gHal.fd);

  usleep(10000); // Sleep due to kernel bug which prevents flush from working imeediately after open

  /* Configure device */
  {
    struct termios cfg;
    if (tcgetattr(gHal.fd, &cfg)) {
      hal_serial_close();
      DAS_LOG(DAS_ERROR, "spine.termios_fail", "tcgetattr() failed");
      return err_TERMIOS_FAIL;
    }

    cfmakeraw(&cfg);
    cfsetispeed(&cfg, baudrate);
    cfsetospeed(&cfg, baudrate);

    cfg.c_cflag |= (CS8 | CSTOPB);    // Use N82 bit words


    if (tcsetattr(gHal.fd, TCSANOW, &cfg)) {
      hal_serial_close();
      DAS_LOG(DAS_ERROR, "spine.termios_fail", "tcsetattr() failed");
      return err_TERMIOS_FAIL;
    }

    if (tcflush(gHal.fd, TCIOFLUSH)) {
      DAS_LOG(DAS_ERROR, "spine.tcflush_fail", "Failed to flush with TCIOFLUSH");
    }
  }
  DAS_LOG(DAS_DEBUG, "spine.serial_port_ok", "Serial port is okay");
  return err_OK;
}

ssize_t hal_select()
{
  static uint8_t selectTimeoutCount = 0;

  static fd_set fdSet;
  FD_ZERO(&fdSet);
  FD_SET(gHal.fd, &fdSet);
  static struct timeval timeout;
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;
  ssize_t s = select(FD_SETSIZE, &fdSet, NULL, NULL, &timeout);
  if (s == 0) {
    selectTimeoutCount++;
    DAS_LOG(DAS_INFO, "spine.select_timeout", "No serial data for %d sec", selectTimeoutCount);
  }
  else {
    selectTimeoutCount = 0;
  }
  return s;
}

static inline ssize_t rx_buffer_space() { return sizeof(gHal.buf_rx) - gHal.rx_cursor; }

// send data into spine for processing
static ssize_t hal_receive_data(const uint8_t* bytes, size_t len)
{
  size_t next_offset = gHal.rx_cursor;
  size_t remaining = rx_buffer_space();

  if (len > remaining) {
    DAS_LOG(DAS_ERROR, "spine.receive_data_overflow", "%u", len - remaining);
    gHal.rx_cursor = 0;
    // BRC: add a flag to indicate a reset (for using in parsing?)
  }

  uint8_t* rx = gHal.buf_rx + next_offset;
  memcpy(rx, bytes, len);
  gHal.rx_cursor = next_offset + len;

  return len;
}

#if EXTENDED_SPINE_DEBUG
static const char* ascii[] = {
  "00 ", "01 ", "02 ", "03 ", "04 ", "05 ", "06 ", "07 ",
  "08 ", "09 ", "0a ", "0b ", "0c ", "0d ", "0e ", "0f ",
  "10 ", "11 ", "12 ", "13 ", "14 ", "15 ", "16 ", "17 ",
  "18 ", "19 ", "1a ", "1b ", "1c ", "1d ", "1e ", "1f ",
  "20 ", "21 ", "22 ", "23 ", "24 ", "25 ", "26 ", "27 ",
  "28 ", "29 ", "2a ", "2b ", "2c ", "2d ", "2e ", "2f ",
  "30 ", "31 ", "32 ", "33 ", "34 ", "35 ", "36 ", "37 ",
  "38 ", "39 ", "3a ", "3b ", "3c ", "3d ", "3e ", "3f ",
  "40 ", "41 ", "42 ", "43 ", "44 ", "45 ", "46 ", "47 ",
  "48 ", "49 ", "4a ", "4b ", "4c ", "4d ", "4e ", "4f ",
  "50 ", "51 ", "52 ", "53 ", "54 ", "55 ", "56 ", "57 ",
  "58 ", "59 ", "5a ", "5b ", "5c ", "5d ", "5e ", "5f ",
  "60 ", "61 ", "62 ", "63 ", "64 ", "65 ", "66 ", "67 ",
  "68 ", "69 ", "6a ", "6b ", "6c ", "6d ", "6e ", "6f ",
  "70 ", "71 ", "72 ", "73 ", "74 ", "75 ", "76 ", "77 ",
  "78 ", "79 ", "7a ", "7b ", "7c ", "7d ", "7e ", "7f ",
  "80 ", "81 ", "82 ", "83 ", "84 ", "85 ", "86 ", "87 ",
  "88 ", "89 ", "8a ", "8b ", "8c ", "8d ", "8e ", "8f ",
  "90 ", "91 ", "92 ", "93 ", "94 ", "95 ", "96 ", "97 ",
  "98 ", "99 ", "9a ", "9b ", "9c ", "9d ", "9e ", "9f ",
  "a0 ", "a1 ", "a2 ", "a3 ", "a4 ", "a5 ", "a6 ", "a7 ",
  "a8 ", "a9 ", "aa ", "ab ", "ac ", "ad ", "ae ", "af ",
  "b0 ", "b1 ", "b2 ", "b3 ", "b4 ", "b5 ", "b6 ", "b7 ",
  "b8 ", "b9 ", "ba ", "bb ", "bc ", "bd ", "be ", "bf ",
  "c0 ", "c1 ", "c2 ", "c3 ", "c4 ", "c5 ", "c6 ", "c7 ",
  "c8 ", "c9 ", "ca ", "cb ", "cc ", "cd ", "ce ", "cf ",
  "d0 ", "d1 ", "d2 ", "d3 ", "d4 ", "d5 ", "d6 ", "d7 ",
  "d8 ", "d9 ", "da ", "db ", "dc ", "dd ", "de ", "df ",
  "e0 ", "e1 ", "e2 ", "e3 ", "e4 ", "e5 ", "e6 ", "e7 ",
  "e8 ", "e9 ", "ea ", "eb ", "ec ", "ed ", "ee ", "ef ",
  "f0 ", "f1 ", "f2 ", "f3 ", "f4 ", "f5 ", "f6 ", "f7 ",
  "f8 ", "f9 ", "fa ", "fb ", "fc ", "fd ", "fe ", "ff "
};

#define open_logfile() fopen("/dev/serial.log", "w")
void serial_log(int dir, const uint8_t* buf, int len)
{
  static int lastdir = 1;
  int i;
  if (dir != lastdir) {
    const uint64_t now = steady_clock_now();
    const uint8_t* now_ptr = (uint8_t*)(&now);
    lastdir = dir;
    if (dir == 0) { fwrite("\n--- OUT --- ", 13, 1, gHal.logFd);}
    else { fwrite("\n--- IN  --- ", 13, 1, gHal.logFd); }
    for (i = 0; i < sizeof(uint64_t); i++) {
      fwrite(ascii[now_ptr[i]], 3, 1, gHal.logFd);
    }
    fwrite("\n", 1, 1, gHal.logFd);
  }
  for (i = 0; i < len; i++) {
    fwrite(ascii[buf[i]], 3, 1, gHal.logFd);
  }
}
#else
#define open_logfile() NULL //no-op
#define serial_log(d,b,l)  //no-op
#endif

static ssize_t hal_spine_io()
{
  static uint8_t readBuffer_[4096];
  int fd = gHal.fd;

  ssize_t bytes_to_read = rx_buffer_space();
  if (sizeof(readBuffer_) < bytes_to_read) {
    bytes_to_read = sizeof(readBuffer_);
  }
  if (hal_select() == 0) { //no data
    return -1;
  }
  ssize_t r = read(fd, readBuffer_, sizeof(readBuffer_));

  if (r > 0) {
    serial_log(1, readBuffer_, r);
    r = hal_receive_data((const void*)readBuffer_, r);
  }
  else if (r < 0) {
    if (errno == EAGAIN) {
      r = 0;
    }
    else {
#if EXTENDED_SPINE_DEBUG
      unsigned char ermsg[] = "RE00";
      ermsg[2] = (r >> 8) & 0xFF;
      ermsg[2] = (r) & 0xFF;
      serial_log(1, ermsg, 4);
#endif
    }
  }
  return r;
}


int hal_serial_send(const uint8_t* buffer, int len)
{
  int written = 0;
  while (len > 0) {
    ssize_t wr = write(gHal.fd, buffer, len);
    if (wr <= 0) {
#if EXTENDED_SPINE_DEBUG
      unsigned char ermsg[] = "ER00";
      ermsg[2] = (wr >> 8) & 0xFF;
      ermsg[2] = (wr) & 0xFF;
      serial_log(0, ermsg, 4);
#endif
      DAS_LOG(DAS_ERROR, "spine.write_error", "Serial write error=%d", wr);
      return wr;
    }
    serial_log(0, buffer, wr);
    buffer += wr;
    written += wr;
    len -= wr;
  }
  return written;
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
  DAS_LOG(DAS_DEBUG, "spine.construct_header_lengths", "expected=%d, payload=%d", expected_len,
          payload_len);
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




/************* PUBLIC INTERFACE ***************/
SpineErr hal_init(const char* devicename, long baudrate)
{
  gHal.fd = -1;

  gHal.logFd = open_logfile();
  SpineErr r = hal_serial_open(devicename, baudrate);
  return r;
}



void hal_discard_rx_bytes(ssize_t bytes_to_drop)
{
  if (bytes_to_drop > 0) {
    ssize_t remaining = sizeof(gHal.buf_rx) - bytes_to_drop;
    assert(remaining >= 0);
    assert(gHal.rx_cursor >= bytes_to_drop);
    if (remaining >= 0) {
      const uint8_t* rx = (const uint8_t*)gHal.buf_rx + bytes_to_drop;
      memmove(gHal.buf_rx, rx, remaining);
    }
    gHal.rx_cursor -= bytes_to_drop;
  }
  else if (bytes_to_drop < 0) {
    // discard all data
    gHal.rx_cursor = 0;
    memset(gHal.buf_rx, 0x55, sizeof(gHal.buf_rx));
  }
}


//moves complete frame from internal rx buffer to outbuf;
// returns 0 if waiting for data
// returns -1 if invalid data found and discarded
// returns length of frame if good.
int hal_parse_frame(uint8_t outbuf[], size_t outbuf_len)
{
  size_t rx_len = gHal.rx_cursor;

  // Are there the minimum number of bytes for a full message to process?
  if (rx_len < SPINE_HEADER_LEN + SPINE_CRC_LEN) {
    return 0;
  }

  // Start from the beginning of the rx buffer
  uint8_t* rx = gHal.buf_rx;
  int last_test_idx = rx_len - (SPINE_SYNC_LEN - 1);
  int sync_index;
  for (sync_index = 0; sync_index < last_test_idx; sync_index++) {
    const uint8_t* test_bytes = rx + sync_index;
    if (BODY_TAG_PREFIX[0] == test_bytes[0] &&
        BODY_TAG_PREFIX[1] == test_bytes[1] &&
        BODY_TAG_PREFIX[2] == test_bytes[2] &&
        BODY_TAG_PREFIX[3] == test_bytes[3]) {
      break;
    }
  }

  // No sync sequence in whole buffer
  if (sync_index >= last_test_idx) {
    // throw away all data except unchecked bytes
    DAS_LOG(DAS_DEBUG, "spine.no_sync", "No sync");
    hal_discard_rx_bytes(sync_index);
    return 0;  //There must < 4 bytes remaining in buffer
  }

  //advance our working pointer to start of sync
  const uint8_t* sync_start = rx + sync_index;
  rx_len -= sync_index;

  // Not enough data for valid header
  if (rx_len < SPINE_HEADER_LEN) {
    // throw away stuff before start of header & wait
    hal_discard_rx_bytes(sync_index);
    return 0;
  }

  // Validate payload data
  const struct SpineMessageHeader* header = (const struct SpineMessageHeader*)sync_start;
  int payload_len = get_payload_len(header->payload_type, dir_READ);

  // Payload type is invalid or payload len is invalid
  if (payload_len == -1 || header->bytes_to_follow != payload_len) {
    // skip current sync
    DAS_LOG(DAS_INFO, "spine.invalid_payload_len", "expected=%d | observed=%u", payload_len,
            header->bytes_to_follow);
    //start searching again after SYNC
    hal_discard_rx_bytes(sync_index + SPINE_SYNC_LEN);
    return -1;
  }

  //At this point we have a valid message header.

  // Not enough data to validate payload + CRC
  if (rx_len < SPINE_HEADER_LEN + payload_len + SPINE_CRC_LEN) {
    // partial frame: wait for more data
    hal_discard_rx_bytes(sync_index);
    return 0;
  }

  //now we just have to validate CRC;
  const uint8_t* payload_start = sync_start + SPINE_HEADER_LEN;
  crc_t expected_crc = *((crc_t*)(payload_start + payload_len));
  crc_t true_crc = calc_crc(payload_start, payload_len);
  if (expected_crc != true_crc) {
    DAS_LOG(DAS_WARN, "spine.crc_error", "calc %08x vs data %08x", true_crc, expected_crc);
    //restart after SYNC
    hal_discard_rx_bytes(sync_index + SPINE_SYNC_LEN);
    return -1;
  }

  // At this point we have a valid frame.
  DAS_LOG(DAS_DEBUG, "spine.found_frame", "payload type=%c%c",
          header->payload_type & 0xff, header->payload_type >> 8);

  // Copy data to output buffer
  size_t frame_len = SPINE_HEADER_LEN + payload_len + SPINE_CRC_LEN;
  assert(outbuf && outbuf_len >= frame_len);
  if (outbuf != NULL && outbuf_len >= frame_len) {
    memcpy(outbuf, sync_start, frame_len);
  }

  // Reset RX buffer
  hal_discard_rx_bytes(sync_index + frame_len);
  return frame_len;
}


//pulls out frames
const void* hal_get_a_frame(uint8_t* frame_buffer, int buffer_len)
{

  assert(buffer_len >= SPINE_B2H_FRAME_LEN);

  ssize_t r = 0;

  do {
    r = hal_parse_frame(frame_buffer, buffer_len);

    if (r < 0) { //no sync found, some data discarded
      continue; //there may be more to parse
    }
    else if (r > 0) {   //good data
      return frame_buffer;
    }
    else {   //r==0: out of data
      hal_spine_io();  //get more and return
    }
  }
  while (r != 0);
  return NULL;
}



const void* hal_get_next_frame(const int32_t timeout_ms)
{
  uint64_t expiry = steady_clock_now() + (timeout_ms * 1000000LL);

  do {
    const void* hdr = hal_get_a_frame(gHal.framebuffer, sizeof(gHal.framebuffer));
    if (hdr) {
      return hdr;
    }
  }
  while (steady_clock_now() < expiry);
  return NULL;
}

const void* hal_wait_for_frame(const uint16_t type, const int32_t timeout_ms)
{
  uint64_t expiry = steady_clock_now() + (timeout_ms * 1000000LL);
  const struct SpineMessageHeader* hdr = NULL;

  while (steady_clock_now() < expiry) {
    hdr = hal_get_next_frame(timeout_ms);
    if (hdr && hdr->payload_type == type) {
      return hdr;
    }
  }
  return NULL;
}

const void* hal_wait_for_frame_or_nack(const uint16_t type, const int32_t timeout_ms)
{
  uint64_t expiry = steady_clock_now() + (timeout_ms * 1000000LL);
  const struct SpineMessageHeader* hdr = NULL;

  while (steady_clock_now() < expiry) {
    hdr = hal_get_next_frame(timeout_ms);
    if (hdr && hdr->payload_type == type) {
      return hdr;
    }
    else if (hdr && hdr->payload_type == PAYLOAD_ACK) {
      if (*(Ack*)(hdr + 1) < 0) { return hdr; }
    }
  }
  return NULL;
}


void hal_send_frame(PayloadId type, const void* data, int len)
{
  const uint8_t* hdr = spine_construct_header(type, len);
  crc_t crc = calc_crc(data, len);
  if (hdr) {
    DAS_LOG(DAS_DEBUG, "spine.send_frame", "sending %c%c packet (%d bytes) CRC=%08x", type & 0xFF,
            type >> 8, len, crc);
    hal_serial_send(hdr, SPINE_HEADER_LEN);
    hal_serial_send(data, len);
    hal_serial_send((uint8_t*)&crc, sizeof(crc));
    tcdrain(gHal.fd);
  }
}

void hal_set_mode(int new_mode)
{
  DAS_LOG(DAS_INFO, "spine.set_mode", "Sending Mode Change %x", PAYLOAD_MODE_CHANGE);
  hal_send_frame(PAYLOAD_MODE_CHANGE, NULL, 0);
}

void hal_exit(void)
{
#if EXTENDED_SPINE_DEBUG
  fclose(gHal.logFd);
#endif
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
