/*****************************************************************************
* Copyright (c) 2012-2016 Qualcomm Technologies, Inc.
* All Rights Reserved.
* Confidential and Proprietary - Qualcomm Technologies, Inc.
*****************************************************************************/

#include "QExifComposer.h"

extern "C" {
#include "writer_utility.h"
#include "jpegerr.h"
#include "jpeglog.h"
#include "exif_private.h"
#include <stdlib.h>
}
/* -----------------------------------------------------------------------
** Type Declarations
** ----------------------------------------------------------------------- */
/* -----------------------------------------------------------------------
** Global Object Definitions
** ----------------------------------------------------------------------- */
/* -----------------------------------------------------------------------
** Local Object Definitions
** ----------------------------------------------------------------------- */
static const int jpeg_natural_order [64+16] = {
   0,  1,  8, 16,  9,  2,  3, 10,
  17, 24, 32, 25, 18, 11,  4,  5,
  12, 19, 26, 33, 40, 48, 41, 34,
  27, 20, 13,  6,  7, 14, 21, 28,
  35, 42, 49, 56, 57, 50, 43, 36,
  29, 22, 15, 23, 30, 37, 44, 51,
  58, 59, 52, 45, 38, 31, 39, 46,
  53, 60, 61, 54, 47, 55, 62, 63,
  63, 63, 63, 63, 63, 63, 63, 63, /* extra entries for safety */
  63, 63, 63, 63, 63, 63, 63, 63
};

static const char stats_debug_magic_string[] = "Qualcomm Camera Debug";
static const char mobicat_magic_string[] = "Qualcomm Camera Attributes v2";
static const char mi_magic_string[] = "Qualcomm Dual Camera Attributes";

extern exif_tag_entry_ex_t default_tag_interopindexstr;
extern exif_tag_entry_ex_t default_tag_r98_version;
/* -----------------------------------------------------------------------
** Forward Declarations
** ----------------------------------------------------------------------- */
/* =======================================================================
**                          Macro Definitions
** ======================================================================= */

// Maximum APP1 length
#define EXIF_MAX_APP1_LENGTH     0xFFFF

#define JWRITE_BYTE(b, ptr, off, size, overflow) \
  jpegw_emit_byte((uint8_t)(b), ptr, &off, size, &overflow)

#define JWRITE_SHORT(s, ptr, off, size, overflow) \
  jpegw_emit_short((uint16_t)(s), ptr, &off, size, &overflow)

#define JWRITE_LONG(l, ptr, off, size, overflow) \
  jpegw_emit_long((int)(l), ptr, &off, size, &overflow)

#define JWRITE_NBYTES(b, c, ptr, off, size, overflow) \
  jpegw_emit_nbytes((uint8_t *)(b), c, ptr, &off, size, &overflow)

#define JWRITE_LONG_LITTLE(l, ptr, off, size, overflow) \
  writer_emit_long_little((int)(l), ptr, &off, size, &overflow)

// 3A Stats debug SW version size in bytes
#define STATS_SW_VERSION_SZ (8)

/* =======================================================================
**                          Function Definitions
** ======================================================================= */

/*===========================================================================
 * Function: New
 *
 * Description: 2 phase contructor for QExifComposer
 *
 * Input parameters:
 *   aObserver - composer observer
 *
 * Return values:
 *   none
 *
 * Notes: none
 *==========================================================================*/
QExifComposer* QExifComposer::New(QImageWriterObserver &aObserver)
{
  int lrc = QI_SUCCESS;
  QExifComposer* lWriter = new QExifComposer(aObserver);
  if (NULL == lWriter) {
    return NULL;
  }
  lrc = lWriter->Init();
  if (QI_SUCCESS != lrc) {
    delete lWriter;
    return NULL;
  }
  return lWriter;
}

/*===========================================================================
 * Function: QExifComposer
 *
 * Description: QExifComposer constuctor
 *
 * Input parameters:
 *   aObserver - composer observer
 *
 * Return values:
 *   none
 *
 * Notes: none
 *==========================================================================*/
QExifComposer::QExifComposer(QImageWriterObserver &aObserver)
  : mObserver(aObserver)
{
  nTiffHeaderOffset = 0;
  nApp1LengthOffset = 0;
  nThumbnailOffset = 0;
  nThumbnailStreamOffset = 0;
  nGpsIfdPointerOffset = 0;
  nThumbnailIfdPointerOffset = 0;
  nFieldCount = 0;
  nFieldCountOffset = 0;
  nJpegInterchangeLOffset = 0;
  fHeaderWritten = 0;
  fApp1Present = false;
  app2_present = false;
  app2_start_offset = 0;
  overflow_flag = false;
  mThumbnail = NULL;
  mAheadBuffer = NULL;
  mCurrentOffset = 0;
  mBuffer = NULL;
  mAheadBufOffset = 0;
  p_exif_info = NULL;
  mWriterParams = NULL;
  is_exif_info_owned = false;
  mMPStartOffset = 0;
  mExitAppMarkerInc = 0;
}

/*===========================================================================
 * Function: ~QExifComposer
 *
 * Description: QExifComposer destructor
 *
 * Input parameters:
 *   none
 *
 * Return values:
 *   none
 *
 * Notes: none
 *==========================================================================*/
QExifComposer::~QExifComposer()
{
  if (mAheadBuffer) {
    delete mAheadBuffer;
    mAheadBuffer = NULL;
  }
  // Destroy exif info if owned by the writer
  if (is_exif_info_owned) {
    exif_destroy((exif_info_obj_t *)(&(p_exif_info)));
  }
}

/*===========================================================================
 * Function: Init
 *
 * Description: initializes the exif composer
 *
 * Input parameters:
 *   none
 *
 * Return values:
 *   none
 *
 * Notes: none
 *==========================================================================*/
int QExifComposer::Init()
{
  mAheadBuffer = QIHeapBuffer::New(EXIF_MAX_APP1_LENGTH);
  if (NULL == mAheadBuffer) {
    return QI_ERR_NO_MEMORY;
  }
  return QI_SUCCESS;
}

/*===========================================================================
 * Function: SetParams
 *
 * Description: set the exif composer parameters
 *
 * Input parameters:
 *   aParams -  reference to exif composer parameters
 *
 * Return values:
 *   none
 *
 * Notes: none
 *==========================================================================*/
int QExifComposer::SetParams(QExifComposerParams &aParams)
{
  mWriterParams = &aParams;
  // check if App2 header exist, if so set the length
  if (mWriterParams->App2HeaderLen()) {
    app2_present = true;

    // Set App2 header length
    app2_header_length = mWriterParams->App2HeaderLen();
  }

  // set exif info
  if (mWriterParams->Exif()) {
    if (p_exif_info && is_exif_info_owned) {
      exif_destroy((exif_info_obj_t *)(&(p_exif_info)));
    }
    p_exif_info = (exif_info_t *)(*mWriterParams->Exif());
  }

  //Set multi image info
  mi_info = mWriterParams->GetMultiImageInfo();

  return QI_SUCCESS;
}

/*===========================================================================
 * Function: FlushThumbnail
 *
 * Description: This function is used to flush the thumbnail data
 *
 * Input parameters:
 *   none
 *
 * Return values:
 *   none
 *
 * Notes: none
 *==========================================================================*/
int QExifComposer::FlushThumbnail()
{
  int lrc = QI_SUCCESS;
  if (mThumbnail) {
    memcpy(mBuffer->Addr() + mCurrentOffset,
      mThumbnail->BaseAddr(),
      mThumbnail->FilledLen());
    mCurrentOffset += mThumbnail->FilledLen();
  }
  return lrc;
}

/*===========================================================================
 * Function: FlushFileHeader
 *
 * Description: This function is used to flush the file header
 *
 * Input parameters:
 *   none
 *
 * Return values:
 *   QI_SUCCESS
 *   QI_ERR_GENERAL
 *
 * Notes: none
 *==========================================================================*/
int QExifComposer::FlushFileHeader()
{
  int lrc = QI_SUCCESS;
  int thumbnail_dropped = 0;
  char* lPayload = NULL;
  uint32_t lPayload_len = 0;

  if(mThumbnail) {
    // drop the thumbnail if it is larger than the allowable
    //  size or the allocated
    // buffer has overflown
    if((fApp1Present &&
      (mThumbnail->FilledLen() + mCurrentOffset - nApp1LengthOffset >
      EXIF_MAX_APP1_LENGTH))) {
        // Update App1 length field
      jpegw_overwrite_short ((uint16_t)(nThumbnailOffset - nApp1LengthOffset),
        mBuffer->Addr(), nApp1LengthOffset,
        mBuffer->Length(), &overflow_flag);

      // set thumbnail IFD pointer to NULL
      jpegw_overwrite_long (0, mBuffer->Addr(),
        nThumbnailIfdPointerOffset,
        mBuffer->Length(),
        &overflow_flag);

      // reset the scratch buffer offset to prior to writing the thumbnail IFD
      mCurrentOffset = nThumbnailOffset;

      // clean up the thumbnail buffer
      thumbnail_dropped = 1;
    } else {
      if (fApp1Present) {
        // Update Jpeg Interchange Format Length
        jpegw_overwrite_long ((mCurrentOffset - nThumbnailStreamOffset) +
          mThumbnail->FilledLen(),
          mBuffer->Addr(),
          nJpegInterchangeLOffset,
          mBuffer->Length(),
          &overflow_flag);
      }

      // Update App1 length field
      jpegw_overwrite_short (
        (uint16_t) ((mCurrentOffset - nApp1LengthOffset) +
        mThumbnail->FilledLen()),
        mBuffer->Addr(),
        nApp1LengthOffset,
        mBuffer->Length(),
        &overflow_flag);
    }
  }
  // Flush out whatever is in the scratch buffer for the header
  if (app2_present) {
    // Update APP2 start offset
    app2_start_offset += mCurrentOffset;
  }

  // Flush the thumbnail
  if (mThumbnail && !thumbnail_dropped) {
    QI_ERROR_RET(FlushThumbnail());
  }

  if (app2_present && mThumbnail) {
    // Update APP2 start offset
    app2_start_offset += mThumbnail->FilledLen();
  }

  if (app2_present) {
    EmitApp2();
  }

  // Emit Stats debug data
  if (mWriterParams->Get3AFlag()) {
    lPayload = mWriterParams->Get3A();
    lPayload_len = mWriterParams->Get3ASize();
    EmitAppByType(EXIF_STATS_DBG_DATA, lPayload, lPayload_len);
  } else {
    QIDBG_MED("%s:%d Not writing 3A debug data for Aux images", __func__, __LINE__);
  }

  // Emit Mobicat data
  //If multi image, write mobicat only for the primary
  if (mWriterParams->GetMobicatFlag()) {
    lPayload = mWriterParams->GetMobicat();
    lPayload_len = (uint32_t)strlen(lPayload);
    EmitAppByType(EXIF_MOBICAT_DATA, lPayload, lPayload_len);
  }

  // Emit Dual camera data
  if (mWriterParams->GetMIMetaFlag()) {
    lPayload = mWriterParams->GetMIMetadata();
    lPayload_len = mWriterParams->GetMIMetaSize();
    EmitAppByType(EXIF_DUAL_CAM_DATA, lPayload, lPayload_len);
  }

  EmitFrameHeader(mWriterParams->EncodeParams(), mWriterParams->Subsampling());

  EmitScanHeader(mWriterParams->EncodeParams());

  if (thumbnail_dropped) {
    mObserver.WriteError(QImageWriterObserver::ERR_THUMBNAIL_DROPPED);
  }

  return lrc;
}

/*===========================================================================
 * Function: StartIfd
 *
 * Description: This function is used to start writing the IFd
 *
 * Input parameters:
 *   none
 *
 * Return values:
 *   none
 *
 * Notes: none
 *==========================================================================*/
void QExifComposer::StartIfd()
{
  nFieldCount = 0;
  nFieldCountOffset = mCurrentOffset;
  mCurrentOffset += 2;
}

/*===========================================================================
 * Function: FinishIfd
 *
 * Description: This function is used to finish writing the IFd
 *
 * Input parameters:
 *   none
 *
 * Return values:
 *   none
 *
 * Notes: none
 *==========================================================================*/
void QExifComposer::FinishIfd()
{
  uint32_t i;
  // the offset in scratch buffer where ahead buffer should be copied to
  int nAheadBufDestination;

  // Emit Next Ifd pointer
  JWRITE_LONG(0, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  // Update the number of fields written and flush the ahead buffer
  // to scratch buffer
  jpegw_emit_short((int16_t)nFieldCount, mBuffer->Addr(),
    &nFieldCountOffset, mBuffer->Length(),
    &overflow_flag);

  // extra 4 bytes for NextIfdOffset
  nAheadBufDestination = nFieldCountOffset + nFieldCount * 12 + 4;
  for (i = 0; i < nFieldCount; i++) {
    uint32_t nAheadOffset;
    uint32_t nWriteOffset = nFieldCountOffset + i * 12 + 8;

    // update offset if dataLen is greater than 4
    const uint32_t type = jpegw_read_short(mBuffer->Addr(), nWriteOffset - 6);
    const uint32_t count = jpegw_read_long(mBuffer->Addr(), nWriteOffset - 4);

    // tag_type_sizes is with size 11
    // valid type is 0 to 10
    if (type >= 11) {
        QIDBG_ERROR("FinishIfd: invalid tag type %d in field %d\n", type, i);
    } else if (count * tag_type_sizes[type] > 4) {
      nAheadOffset = jpegw_read_long(mBuffer->Addr(), nWriteOffset);

      // adjust offset by adding
      jpegw_emit_long(nAheadOffset + nAheadBufDestination - nTiffHeaderOffset,
        mBuffer->Addr(), &nWriteOffset, mBuffer->Length(),
        &overflow_flag);
    }
  }
  // copy ahead buffer to scratch buffer
  memcpy(mBuffer->Addr() + nAheadBufDestination,
    mAheadBuffer->Addr(), mAheadBufOffset);
  mCurrentOffset = nAheadBufDestination + mAheadBufOffset;
  mAheadBufOffset = 0;
  nFieldCount = 0;
}

/*===========================================================================
 * Function: EmitDQT
 *
 * Description: This function is used to emit DQT header
 *
 * Input parameters:
 *   aQtbl - pointer to the Qtable
 *
 * Return values:
 *   none
 *
 * Notes: none
 *==========================================================================*/
void QExifComposer::EmitDQT(uint16_t *aQtbl)
{
  int i;
  unsigned int qval;

  for (i = 0; i < 64; i++) {
    /* The table entries must be emitted in zigzag order. */
    qval = aQtbl[jpeg_natural_order[i]];
    JWRITE_BYTE((uint8_t) (qval & 0xFF), mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);
  }
}

/*===========================================================================
 * Function: EmitDHT
 *
 * Description: This function is used to emit DHT header
 *
 * Input parameters:
 *   htbl - pointer to the huffman table
 *   index - index to the table
 *
 * Return values:
 *   none
 *
 * Notes: none
 *==========================================================================*/
void QExifComposer::EmitDHT(const QIHuffTable::HuffTable *htbl, int index)
{
  uint16_t length = 0, i;

  for (i = 1; i <= 16; i++)
    length = (uint16_t)(length + (uint16_t)(htbl->mBits[i]));

  JWRITE_BYTE((uint8_t)(index & 0xFF), mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  for (i = 1; i <= 16; i++)
    JWRITE_BYTE((uint8_t)(htbl->mBits[i]), mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);

  for (i = 0; i < length; i++)
    JWRITE_BYTE((uint8_t)(htbl->mValues[i]), mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);
}

/*===========================================================================
 * Function: EmitDRI
 *
 * Description: This function is used to emit DRI header
 *
 * Input parameters:
 *   aRestartInterval - restart interval
 *
 * Return values:
 *   none
 *
 * Notes: none
 *==========================================================================*/
void QExifComposer::EmitDRI(uint16_t aRestartInterval)
{
  JWRITE_SHORT(0xFF00 | M_DRI, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  JWRITE_SHORT(4, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag); /* fixed length */

  JWRITE_SHORT((int16_t) (aRestartInterval), mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);
}

/*===========================================================================
 * Function: EmitSOF
 *
 * Description: This function is used to emit SOF header
 *
 * Input parameters:
 *   code - jpeg marker code
 *   aParams - encoder parameter reference
 *   aSubSampling - image subsampling type
 *
 * Return values:
 *   none
 *
 * Notes: none
 *==========================================================================*/
void QExifComposer::EmitSOF(jpeg_marker_t code, QIEncodeParams &aParams,
  QISubsampling aSubSampling)
{
  const uint16_t nNumComponents = aParams.NumOfComponents();
  const uint8_t nPrecision = 8;
  uint8_t i;
  uint32_t output_width, output_height;


  JWRITE_SHORT(0xFF00 | code, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  JWRITE_SHORT(3 * (uint16_t) nNumComponents + 2 + 5 + 1,
    mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag); /* length */

  JWRITE_BYTE(nPrecision, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag); /* data precision */

  output_width  = aParams.OutputSize().Width();
  output_height = aParams.OutputSize().Height();

  if (aParams.Rotation() % 180) {
    JWRITE_SHORT((uint16_t)output_width, mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);
    JWRITE_SHORT((uint16_t)output_height, mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);
  } else {
    JWRITE_SHORT((uint16_t)output_height, mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);
    JWRITE_SHORT((uint16_t)output_width, mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);
  }

  JWRITE_BYTE(nNumComponents, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  for (i = 0; i < nNumComponents; i++) {
    // Write component ID
    JWRITE_BYTE(i + 1, mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);

    // Luma
    if (i == 0) {
      // Write sampling factors
      switch (aSubSampling) {
        case QI_H2V2:
          JWRITE_BYTE(0x22, mBuffer->Addr(), mCurrentOffset,
            mBuffer->Length(), overflow_flag);
          break;
        case QI_H2V1:
          if (aParams.Rotation() % 180) {
            JWRITE_BYTE(0x12, mBuffer->Addr(), mCurrentOffset,
               mBuffer->Length(), overflow_flag);
          } else {
            JWRITE_BYTE(0x21, mBuffer->Addr(), mCurrentOffset,
              mBuffer->Length(), overflow_flag);
          }
          break;
        case QI_H1V2:
          if (aParams.Rotation() % 180) {
            JWRITE_BYTE(0x21, mBuffer->Addr(), mCurrentOffset,
              mBuffer->Length(), overflow_flag);
          } else {
            JWRITE_BYTE(0x12, mBuffer->Addr(), mCurrentOffset,
              mBuffer->Length(), overflow_flag);
          }
          break;
        case QI_H1V1:
          JWRITE_BYTE(0x11, mBuffer->Addr(), mCurrentOffset,
            mBuffer->Length(), overflow_flag);
          break;
      }
      // Write quantization table selector
      JWRITE_BYTE(0, mBuffer->Addr(), mCurrentOffset,
        mBuffer->Length(), overflow_flag);
    } else { // Chroma
      // Write sampling factors
      JWRITE_BYTE(0x11, mBuffer->Addr(), mCurrentOffset,
        mBuffer->Length(), overflow_flag);

      // Write quantization table selector
      JWRITE_BYTE(1, mBuffer->Addr(), mCurrentOffset,
        mBuffer->Length(), overflow_flag);
    }
  }
}

/*===========================================================================
 * Function: EmitSOS
 *
 * Description: This function is used to emit SOS header
 *
 * Input parameters:
 *   none
 *
 * Return values:
 *   none
 *
 * Notes: none
 *==========================================================================*/
void QExifComposer::EmitSOS(QIEncodeParams &aParams)
{
  const uint16_t nComponents = aParams.NumOfComponents();

  JWRITE_SHORT(0xFF00 | M_SOS, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  JWRITE_SHORT(2 * nComponents + 2 + 1 + 3, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag); /* length */

  /* number of components in scan */
  JWRITE_BYTE( (uint8_t) nComponents, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  /* Y                 Comp ID   Table Indices  */
  /*                                   DC       AC  */
  JWRITE_SHORT( (1 << 8)   |  (0 << 4) | 0, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag); /* (0 << 4) + 0 DC and AC table index */

  if (nComponents > 1) {
    /* Cb */
    JWRITE_SHORT( (2 << 8)   |  (1 << 4) | 1, mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag); /* DC and AC table index */

    /* Cr */
    JWRITE_SHORT( (3 << 8)   |  (1 << 4) | 1, mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag); /* DC and AC table index */
  }

  /*                    Ss        Se  */
  JWRITE_SHORT( (0 << 8)  |  63, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);  /* Se */

  JWRITE_BYTE(0, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag); /* (0 << 4) + 0) Ah and Al */
}

/*===========================================================================
 * Function: EmitApp0
 *
 * Description: This function is used to emit App0 header
 *
 * Input parameters:
 *   none
 *
 * Return values:
 *   none
 *
 * Notes: none
 *==========================================================================*/
void QExifComposer::EmitApp0()
{
  /*
   * Length of APP0 block   (2 bytes)
   * Block ID           (4 bytes - ASCII "JFIF")
   * Zero byte          (1 byte to terminate the ID string)
   * Version Major, Minor   (2 bytes - major first)
   * Units          (1 byte - 0x00 = none, 0x01 = inch, 0x02 = cm)
   * Xdpu           (2 bytes - dots per unit horizontal)
   * Ydpu           (2 bytes - dots per unit vertical)
   * Thumbnail X size       (1 byte)
   * Thumbnail Y size       (1 byte)
   */
  JWRITE_SHORT(0xFF00 | M_APP0, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  // no thumbnail case
  if (NULL == mThumbnail) {

    JWRITE_SHORT(2 + 4 + 1 + 2 + 1 + 2 + 2 + 1 + 1,
      mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag); /* length */

    // Write "JFIF" app string
    JWRITE_LONG(0x4A464946,
      mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);

    /* 0 | major version | minor version | pixel size (density unit) */
    JWRITE_LONG((0<<24) | (1<<16) | (1<<8) | 0,
      mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);

    /* XDensity | YDensity */
    JWRITE_LONG((1<<16) | (1),
      mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);

    /* No thumbnail | No thumbnail */
    JWRITE_SHORT(0,
      mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);

  } else {

    // length will be determined later, by pass for now
    nApp1LengthOffset = mCurrentOffset;
    mCurrentOffset += 2;

    // Write "JFXX" app string
    JWRITE_LONG(0x4A465858, mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);
    JWRITE_BYTE(0, mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);

    // Extension code for Thumbnail coded using JPEG
    JWRITE_BYTE(0x10, mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);

    // Extension data
    // Save Thumbnail Stream Offset
    nThumbnailStreamOffset = mCurrentOffset;

    // Emit SOI
    JWRITE_SHORT(0xFF00 | M_SOI, mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);

    // Emit Frame Header
    EmitFrameHeader(mWriterParams->EncodeParams(),
      mWriterParams->Subsampling());

    // Emit Scan Header
    EmitScanHeader(mWriterParams->EncodeParams(true));
  }
}

/*===========================================================================
 * Function: EmitExif
 *
 * Description: This function is used to emit Exif header
 *
 * Input parameters:
 *   none
 *
 * Return values:
 *   none
 *
 * Notes: none
 *==========================================================================*/
void QExifComposer::EmitExif(exif_tag_entry_ex_t *p_entry)
{
  uint16_t tag_id;
  uint32_t to_write_len, i;
  uint32_t bytes_written = 0;

  if (!p_entry)
    return;

  tag_id = p_entry->tag_id & 0xFFFF;

  // Write Tag ID
  JWRITE_SHORT(tag_id,
    mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  // Write Tag Type
  JWRITE_SHORT((uint16_t)p_entry->entry.type,
    mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  // Write Tag count
  JWRITE_LONG(p_entry->entry.count,
    mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  // Compute the length that needs to be written
  to_write_len = tag_type_sizes[p_entry->entry.type] * p_entry->entry.count;
  QIDBG_LOW("%s:%d] to_write_len %d", __func__, __LINE__, to_write_len);

  /*
   * If to_write_len <= 4, tag value written along-side taglength and
   * other parameters in the Scratch Buffer. Else, a pointer to the tag value
   * is written along-side taglength etc in the Scratch Buffer, and the actual
   * value written in the ahead buffer. Refer to EXIF std for details.
   */
  if (to_write_len <= 4) {
    if (p_entry->entry.type == EXIF_ASCII ||
      p_entry->entry.type == EXIF_UNDEFINED) {
      JWRITE_NBYTES(p_entry->entry.data._ascii, to_write_len,
        mBuffer->Addr(), mCurrentOffset,
        mBuffer->Length(), overflow_flag);
      bytes_written = to_write_len;
    } else {
      if (p_entry->entry.count > 1) {
        for (i = 0; i < p_entry->entry.count; i++) {
          switch (p_entry->entry.type) {
          case EXIF_BYTE:
            JWRITE_BYTE(p_entry->entry.data._bytes[i],
              mBuffer->Addr(), mCurrentOffset,
              mBuffer->Length(), overflow_flag);
            bytes_written ++;
            break;
          case EXIF_SHORT:
            JWRITE_SHORT(p_entry->entry.data._shorts[i],
              mBuffer->Addr(), mCurrentOffset,
              mBuffer->Length(), overflow_flag);
            bytes_written += 2;
            break;
          default:
            QIDBG_ERROR("EmitExif: impossible case! p_entry->entry.type = %d\n",
              p_entry->entry.type);
            return;
          }
        }
      } else {
        switch (p_entry->entry.type) {
        case EXIF_BYTE:
          JWRITE_BYTE((uint32_t)p_entry->entry.data._byte,
            mBuffer->Addr(), mCurrentOffset,
            mBuffer->Length(), overflow_flag);
          bytes_written = 1;
          break;
        case EXIF_SHORT:
          JWRITE_SHORT((uint32_t)p_entry->entry.data._short,
            mBuffer->Addr(), mCurrentOffset,
            mBuffer->Length(), overflow_flag);
          bytes_written = 2;
          break;
        case EXIF_LONG:
          JWRITE_LONG((uint32_t)p_entry->entry.data._long,
            mBuffer->Addr(), mCurrentOffset,
            mBuffer->Length(), overflow_flag);
          bytes_written = 4;
          break;
        case EXIF_SLONG:
          JWRITE_LONG((uint32_t)p_entry->entry.data._slong,
            mBuffer->Addr(), mCurrentOffset,
            mBuffer->Length(), overflow_flag);
          bytes_written = 4;
          break;
        default:
          QIDBG_ERROR("EmitExif: impossible case! p_entry->entry.type = %d\n",
            p_entry->entry.type);
          return;
        }
      }
    }
    // Fill up 0's till there are totally 4 bytes written
    for (i = bytes_written; i < 4; i++)
      JWRITE_BYTE(0, mBuffer->Addr(), mCurrentOffset,
        mBuffer->Length(), overflow_flag);

  } else  { // if (to_write_len <= 4)
    if (mAheadBufOffset & 1)
      mAheadBufOffset++;

    // Write the temporary offset (to be updated later)
    JWRITE_LONG(mAheadBufOffset, mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);

    if (p_entry->entry.type == EXIF_ASCII ||
      p_entry->entry.type == EXIF_UNDEFINED) {
        JWRITE_NBYTES(p_entry->entry.data._ascii, p_entry->entry.count,
          mAheadBuffer->Addr(), mAheadBufOffset,
          mAheadBuffer->Length(), overflow_flag);
    } else if (p_entry->entry.count > 1) {
      // Multiple data to write
      for (i = 0; i < p_entry->entry.count; i++) {
        switch (p_entry->entry.type) {
        case EXIF_BYTE:
          JWRITE_BYTE(p_entry->entry.data._bytes[i],
            mAheadBuffer->Addr(), mAheadBufOffset,
            mAheadBuffer->Length(), overflow_flag);
          break;
        case EXIF_SHORT:
          JWRITE_SHORT(p_entry->entry.data._shorts[i],
            mAheadBuffer->Addr(), mAheadBufOffset,
            mAheadBuffer->Length(), overflow_flag);
          break;
        case EXIF_LONG:
          JWRITE_LONG(p_entry->entry.data._longs[i],
            mAheadBuffer->Addr(), mAheadBufOffset,
            mAheadBuffer->Length(), overflow_flag);
          break;
        case EXIF_SLONG:
          JWRITE_LONG(p_entry->entry.data._slongs[i],
            mAheadBuffer->Addr(), mAheadBufOffset,
            mAheadBuffer->Length(), overflow_flag);
          break;
        case EXIF_RATIONAL:
          JWRITE_LONG(p_entry->entry.data._rats[i].num,
            mAheadBuffer->Addr(), mAheadBufOffset,
            mAheadBuffer->Length(), overflow_flag);
          JWRITE_LONG(p_entry->entry.data._rats[i].denom,
            mAheadBuffer->Addr(), mAheadBufOffset,
            mAheadBuffer->Length(), overflow_flag);
          break;
        case EXIF_SRATIONAL:
          JWRITE_LONG(p_entry->entry.data._srats[i].num,
            mAheadBuffer->Addr(), mAheadBufOffset,
            mAheadBuffer->Length(), overflow_flag);
          JWRITE_LONG(p_entry->entry.data._srats[i].denom,
            mAheadBuffer->Addr(), mAheadBufOffset,
            mAheadBuffer->Length(), overflow_flag);
          break;
        default:
            break;
        }
      }
    } else {
      switch (p_entry->entry.type) {
      case EXIF_BYTE:
        JWRITE_BYTE(p_entry->entry.data._byte,
          mAheadBuffer->Addr(), mAheadBufOffset,
          mAheadBuffer->Length(), overflow_flag);
        break;
      case EXIF_SHORT:
        JWRITE_SHORT(p_entry->entry.data._short,
          mAheadBuffer->Addr(), mAheadBufOffset,
          mAheadBuffer->Length(), overflow_flag);
        break;
      case EXIF_LONG:
        JWRITE_LONG(p_entry->entry.data._long,
          mAheadBuffer->Addr(), mAheadBufOffset,
          mAheadBuffer->Length(), overflow_flag);
        break;
      case EXIF_SLONG:
        JWRITE_LONG(p_entry->entry.data._slong,
          mAheadBuffer->Addr(), mAheadBufOffset,
          mAheadBuffer->Length(), overflow_flag);
        break;
      case EXIF_RATIONAL:
        JWRITE_LONG(p_entry->entry.data._rat.num,
          mAheadBuffer->Addr(), mAheadBufOffset,
          mAheadBuffer->Length(), overflow_flag);
        JWRITE_LONG(p_entry->entry.data._rat.denom,
          mAheadBuffer->Addr(), mAheadBufOffset,
          mAheadBuffer->Length(), overflow_flag);
        break;
      case EXIF_SRATIONAL:
        JWRITE_LONG(p_entry->entry.data._srat.num,
          mAheadBuffer->Addr(), mAheadBufOffset,
          mAheadBuffer->Length(), overflow_flag);
        JWRITE_LONG(p_entry->entry.data._srat.denom,
          mAheadBuffer->Addr(), mAheadBufOffset,
          mAheadBuffer->Length(), overflow_flag);
        break;
      default:
          break;
      }
    }
  }
  nFieldCount++;
}

/*===========================================================================
 * Function: Emit0thIfd
 *
 * Description: This function is used to emit 0th Ifd header
 *
 * Input parameters:
 *   nExifIfdPointerOffset - ifd offset
 *   nGpsIfdPointerOffset - gps pointer offset
 *
 * Return values:
 *   none
 *
 * Notes: none
 *==========================================================================*/
void QExifComposer::Emit0thIfd(uint32_t *nExifIfdPointerOffset,
  uint32_t *nGpsIfdPointerOffset)
{
  int i;
  exif_tag_entry_ex_t **pp_entries = (exif_tag_entry_ex_t **)(p_exif_info);
  exif_tag_entry_ex_t dummy_entry;

  StartIfd();

  for (i = (int)NEW_SUBFILE_TYPE; i <= (int)GPS_IFD; i++) {
    EmitExif(pp_entries[i]);
  }

  // Save location for later update and emit Exif IFD pointer
  *nExifIfdPointerOffset = mCurrentOffset;

  // Craft Exif IFD pointer tag
  dummy_entry.entry.count = 1;
  dummy_entry.entry.type = (exif_tag_type_t)EXIFTAGTYPE_EXIF_IFD_PTR;
  dummy_entry.entry.data._long = 0;
  dummy_entry.tag_id = EXIFTAGID_EXIF_IFD_PTR;

  EmitExif(&dummy_entry);

  // Save location for later use and emit GPD Ifd pointer
  *nGpsIfdPointerOffset = mCurrentOffset;

  dummy_entry.tag_id = EXIFTAGID_GPS_IFD_PTR;
  EmitExif(&dummy_entry);

  // Save location for thumbnail pointer IFD
  nThumbnailIfdPointerOffset = mCurrentOffset;

  FinishIfd();
}

/*===========================================================================
 * Function: EmitExifIfd
 *
 * Description: This function is used to emit Exif Ifd header
 *
 * Input parameters:
 *   nInteropIfdPointerOffset - pointer to interop offset
 *
 * Return values:
 *   none
 *
 * Notes: none
 *==========================================================================*/
void QExifComposer::EmitExifIfd(uint32_t *nInteropIfdPointerOffset)
{
  int i;
  exif_tag_entry_ex_t **pp_entries = (exif_tag_entry_ex_t **)(p_exif_info);
  exif_tag_entry_ex_t dummy_entry;
  uint32_t output_width, output_height;

  StartIfd();

  // Emit EXIF IFD
  for (i = (int)EXPOSURE_TIME; i <= (int)EXIF_COLOR_SPACE; i++) {
    EmitExif(pp_entries[i]);
  }

  output_width  = mWriterParams->EncodeParams().OutputSize().Width();
  output_height = mWriterParams->EncodeParams().OutputSize().Height();

  // Emit Pixel X and Y Dimension
  dummy_entry.entry.count = 1;
  dummy_entry.entry.type = (exif_tag_type_t)EXIF_LONG;
  if (mWriterParams->EncodeParams().Rotation() % 180) {
    dummy_entry.entry.data._long = output_height;
  } else {
    dummy_entry.entry.data._long = output_width;
  }
  dummy_entry.tag_id = EXIFTAGID_EXIF_PIXEL_X_DIMENSION;
  EmitExif(&dummy_entry);

  if (mWriterParams->EncodeParams().Rotation() % 180) {
    dummy_entry.entry.data._long = output_width;
  } else {
    dummy_entry.entry.data._long = output_height;
  }
  dummy_entry.tag_id = EXIFTAGID_EXIF_PIXEL_Y_DIMENSION;
  EmitExif(&dummy_entry);

  // Save offset and Emit Interoperability Ifd Pointer
  *nInteropIfdPointerOffset = mCurrentOffset;
  dummy_entry.entry.data._long = 0;
  dummy_entry.tag_id = EXIFTAGID_INTEROP_IFD_PTR;

  EmitExif(&dummy_entry);

  // Continue with the rest of the IFD
  for (i = (int)RELATED_SOUND_FILE; i <= (int)PIM; i++) {
    EmitExif(pp_entries[i]);
  }

  FinishIfd();
}

/*===========================================================================
 * Function: EmitExifIfd
 *
 * Description: This function is used to emit interop Ifd
 *
 * Input parameters:
 *   none
 *
 * Return values:
 *   none
 *
 * Notes: none
 *==========================================================================*/
void QExifComposer::EmitInteropIfd()
{
  StartIfd();

  // Emit Interoperability Index
  EmitExif(&default_tag_interopindexstr);

  // Emit Exif R98 Version
  EmitExif(&default_tag_r98_version);

  FinishIfd();
}

/*===========================================================================
 * Function: EmitGpsIfd
 *
 * Description: This function is used to emit Gps Ifd
 *
 * Input parameters:
 *   none
 *
 * Return values:
 *   none
 *
 * Notes: none
 *==========================================================================*/
void QExifComposer::EmitGpsIfd()
{
  int i;
  exif_tag_entry_ex_t **pp_entries = (exif_tag_entry_ex_t **)(p_exif_info);
  StartIfd();

  // Emit tags in GPS IFD
  for (i = (int)GPS_VERSION_ID; i <= (int)GPS_DIFFERENTIAL; i++) {
    EmitExif(pp_entries[i]);
  }

  FinishIfd();
}

/*===========================================================================
 * Function: EmitThumbnailIfd
 *
 * Description: This function is used to emit Thumbnail Ifd
 *
 * Input parameters:
 *   none
 *
 * Return values:
 *   none
 *
 * Notes: none
 *==========================================================================*/
void QExifComposer::EmitThumbnailIfd()
{
  exif_tag_entry_ex_t dummy_entry;
  int i, nJpegInterchangeOffset = 0;
  exif_tag_entry_ex_t **pp_entries = (exif_tag_entry_ex_t **)(p_exif_info);

  // Update Thumbnail_IFD pointer
  jpegw_overwrite_long(mCurrentOffset - nTiffHeaderOffset,
    mBuffer->Addr(), nThumbnailIfdPointerOffset,
    mBuffer->Length(), &overflow_flag);

  StartIfd();

  // Emit Pixel X and Y Dimension
  dummy_entry.entry.count = 1;
  dummy_entry.entry.type = (exif_tag_type_t)EXIF_LONG;
  dummy_entry.entry.data._long = 0;

  // Emit tags in TIFF IFD for thumbnail
  for (i = (int)TN_IMAGE_WIDTH; i <= (int)TN_COPYRIGHT; i++) {
    // Save offset to 'Offset to JPEG SOI'
    if (i == (int)TN_JPEGINTERCHANGE_FORMAT) {
      nJpegInterchangeOffset = mCurrentOffset + 8;
      dummy_entry.tag_id = EXIFTAGID_TN_JPEGINTERCHANGE_FORMAT;
      EmitExif(&dummy_entry);
    } else if (i == (int)TN_JPEGINTERCHANGE_FORMAT_L) {
      nJpegInterchangeLOffset = mCurrentOffset + 8;
      dummy_entry.tag_id = EXIFTAGID_TN_JPEGINTERCHANGE_FORMAT_L;
      EmitExif(&dummy_entry);
    } else {
      EmitExif(pp_entries[i]);
    }
  }

  FinishIfd();

  // Save Thumbnail Stream Offset
  nThumbnailStreamOffset = mCurrentOffset;

  // Emit SOI
  JWRITE_SHORT(0xFF00 | M_SOI, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  // Emit Frame Header
  EmitFrameHeader(mWriterParams->EncodeParams(true),
    mWriterParams->Subsampling(true));

  // Emit Scan Header
  EmitScanHeader(mWriterParams->EncodeParams(true));

  // Update Jpeg Interchange Format
  jpegw_overwrite_long(nThumbnailStreamOffset - nTiffHeaderOffset,
    mBuffer->Addr(),
    nJpegInterchangeOffset,
    mBuffer->Length(),
    &overflow_flag);
}

/*===========================================================================
 * Function: EmitApp1
 *
 * Description: This function is used to emit App1 marker
 *
 * Input parameters:
 *   none
 *
 * Return values:
 *   none
 *
 * Notes: none
 *==========================================================================*/
void QExifComposer::EmitApp1()
{
  uint32_t nExifIfdPointerOffset;
  uint32_t nGpsIfdPointerOffset;
  uint32_t nInteropIfdPointerOffset;

  JWRITE_SHORT(0xFF00 | M_APP1, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  // length will be calculated after all operations. bypass for now
  nApp1LengthOffset = mCurrentOffset;
  mCurrentOffset += 2;

  // Write "Exif" app string
  JWRITE_LONG(0x45786966, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);
  JWRITE_SHORT(0, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  // TIFF header
  nTiffHeaderOffset = mCurrentOffset;
  JWRITE_SHORT(0x4D4D, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);
  JWRITE_SHORT(0x002A, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);
  JWRITE_LONG(0x00000008, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  // Write 0th Ifd
  Emit0thIfd(&nExifIfdPointerOffset, &nGpsIfdPointerOffset);

  // Make sure the starting offset is at 2-byte boundary.
  if (mCurrentOffset & 1)
    JWRITE_BYTE(0, mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);

  // Go back and update EXIF_IFD pointer
  nExifIfdPointerOffset += 8;
  jpegw_emit_long(mCurrentOffset - nTiffHeaderOffset, mBuffer->Addr(),
    &nExifIfdPointerOffset, mBuffer->Length(), &overflow_flag);

  // Write Exif Ifd
  EmitExifIfd(&nInteropIfdPointerOffset);

  // Go back and update INTEROP_IFD pointer
  nInteropIfdPointerOffset += 8;
  jpegw_emit_long(mCurrentOffset - nTiffHeaderOffset,
    mBuffer->Addr(),
    &nInteropIfdPointerOffset,
    mBuffer->Length(),
    &overflow_flag);

  // Write Interoperability Ifd
  EmitInteropIfd();

  // Make sure the starting offset is at 2-byte boundary.
  if (mCurrentOffset & 1)
    JWRITE_BYTE(0, mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);

    // Go back and update GPS_IFD pointer
    nGpsIfdPointerOffset += 8;
    jpegw_emit_long(mCurrentOffset - nTiffHeaderOffset,
      mBuffer->Addr(),
      &nGpsIfdPointerOffset,
      mBuffer->Length(),
      &overflow_flag);

    // Write GPS Ifd
    EmitGpsIfd();

  if (mThumbnail) {
    // Make sure the starting offset is at 2-byte boundary.
    if (mCurrentOffset & 1)
      JWRITE_BYTE(0, mBuffer->Addr(), mCurrentOffset,
        mBuffer->Length(), overflow_flag);

    // Save the thumbnail starting offset
    nThumbnailOffset = mCurrentOffset;

    EmitThumbnailIfd();
  }

  // Update App1 length field
  jpegw_overwrite_short ((uint16_t)(mCurrentOffset - nApp1LengthOffset),
    mBuffer->Addr(), nApp1LengthOffset, mBuffer->Length(),
    &overflow_flag);

  fApp1Present = true;
}

/*===========================================================================
 * Function: EmitMPIndexIFD
 *
 * Description: This function is used to emit the MP Index IFD for the primary
 *  image in a MPO image
 *
 * Input parameters:
 *   none
 *
 * Return values:
 *   none
 *
 * Notes: This IFD is added only for the primary image
 *==========================================================================*/
void QExifComposer::EmitMPIndexIFD()
{
  int indexIFDCountOffset = 0, indexIFDCount = 0, offsetToNextIFD = 0;
  int mpEntryOffset = 0;
  exif_tag_entry_ex_t dummy_entry;
  uint8_t  default_mpentry_val = 0, default_uid_val = 0;
  uint32_t image_attribute, i = 0;
  uint8_t mp_format_version [] = {0x30, 0x31, 0x30, 0x30};

  //Update Count later
  indexIFDCountOffset = mCurrentOffset;
  mCurrentOffset += MP_APP2_FIELD_LENGTH_BYTES;

  //Write MP Format Version: 12 bytes
  dummy_entry.entry.count = 4;
  dummy_entry.entry.type = (exif_tag_type_t)EXIF_UNDEFINED;
  dummy_entry.entry.data._undefined = mp_format_version;
  dummy_entry.tag_id = _ID_MP_F_VERSION;
  EmitExif(&dummy_entry);
  indexIFDCount++;

  //Write Number of Images in the sequence: 12 bytes
  dummy_entry.entry.count = 1;
  dummy_entry.entry.type = (exif_tag_type_t)EXIF_LONG;
  dummy_entry.entry.data._long = mi_info.num_of_images;
  dummy_entry.tag_id = _ID_NUMBER_OF_IMAGES;
  EmitExif(&dummy_entry);
  indexIFDCount++;

  //Write MP Entry. Todo: check if value needs to be overwritten
  mpEntryOffset = mCurrentOffset;
  dummy_entry.entry.count = 16 * mi_info.num_of_images;
  dummy_entry.entry.type = EXIF_UNDEFINED;
  dummy_entry.entry.data._undefined = &default_mpentry_val;
  dummy_entry.tag_id = _ID_MP_ENTRY;
  EmitExif(&dummy_entry);
  indexIFDCount++;

  //Write Individual Image Unique ID
  dummy_entry.entry.count = 33 * mi_info.num_of_images;
  dummy_entry.entry.type = EXIF_UNDEFINED;
  dummy_entry.entry.data._undefined = &default_uid_val;
  dummy_entry.tag_id = _ID_IMAGE_UID_LIST;
  EmitExif(&dummy_entry);
  indexIFDCount++;

  //Write Total Num of captured images
  dummy_entry.entry.count = 1;
  dummy_entry.entry.type = EXIF_LONG;
  dummy_entry.entry.data._long = mi_info.num_of_images;
  dummy_entry.tag_id = _ID_TOTAL_FRAMES;
  EmitExif(&dummy_entry);
  indexIFDCount++;

  //Update count value
  jpegw_overwrite_short(indexIFDCount, mBuffer->Addr(), indexIFDCountOffset,
    mBuffer->Length(), &overflow_flag);

  //Mark the Offset to Next IFD.
  //ToDo: This has to point to the MP Attributes when the MP attributes
  //header is added
  offsetToNextIFD = mCurrentOffset;
  JWRITE_LONG(0x000, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  //MPEntry Value section
  //Image attribute for Primary image
  image_attribute = NON_DEPENDENT_IMAGE | NOT_REPRESENTATIVE_IMAGE | JPEG |
    MULTI_VIEW_DISPARITY;
  JWRITE_LONG(image_attribute, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  //Individual Image Size (Filled later during Composition)
  JWRITE_LONG(0x000, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  //Individual Image data offset (NULL for first image)
  JWRITE_LONG(0x000, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  // Dependent Image 1 Entry Number:
  if (mi_info.large_thumbnail_present) {
    //ToDo: Set Entry number to the entry number of the dependant child image
  } else {
    JWRITE_SHORT(0x0000, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);
  }

  // Dependent Image 2 Entry Number:
  JWRITE_SHORT(0x0000, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  for (i = 1; i < mi_info.num_of_images; i++) {
    //Image attribute
    JWRITE_LONG(image_attribute, mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);

    //Individual Image Size (Filled later during Composition)
    JWRITE_LONG(0x000, mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);

    //Individual Image data offset (Filled later during composition)
    JWRITE_LONG(0x000, mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);

    // Dependent Image 1 Entry Number:
    JWRITE_SHORT(0x0000, mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);

    // Dependent Image 2 Entry Number:
    JWRITE_SHORT(0x0000, mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);
  }

}

/*===========================================================================
 * Function: EmitApp2
 *
 * Description: This function is used to emit App2 marker
 *
 * Input parameters:
 *   none
 *
 * Return values:
 *   none
 *
 * Notes: This app marker contains the MPO header
 *==========================================================================*/
void QExifComposer::EmitApp2()
{
  // Write APP2 Marker
  JWRITE_SHORT(0xFF00 | M_APP2, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  // Update APP2 start offset
  app2_start_offset = mCurrentOffset;

  //Will update the app2 header length at the end. proceed for now
  mCurrentOffset += 2;

  //Write the Mp Format Identifier. Should always be "MPF"
  JWRITE_LONG(0x4D504600, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  //Start MP Header
  //All offsts in the MP header are specified wrt to the MPStartOffset
  mMPStartOffset = mCurrentOffset;

  //Fill in the Mp Endian
  JWRITE_LONG(MPO_BIG_ENDIAN, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  //Write Offset to First IFD
  //Offset to first IFD points to either the MP Index IFD or the MP
  //attributes IFD. If either of them is specified immediately after,
  //the value shall be 8
  if (mi_info.is_primary) {
    JWRITE_LONG(0x08, mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);
    //Write MP index ifd
    EmitMPIndexIFD();
  } else {
    //ToDo: Update this to point to the MP attributes IFD when
    // Attributes IFD is supported
    JWRITE_LONG(0x00, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);
  }

  // Update App2 length field
  jpegw_overwrite_short((uint16_t)(mCurrentOffset - app2_start_offset),
    mBuffer->Addr(), app2_start_offset, mBuffer->Length(),
    &overflow_flag);
}
/*===========================================================================
 * Function: EmitEXIF
 *
 * Description: This function is used to emit EXIF header
 *
 * Input parameters:
 *   none
 *
 * Return values:
 *   QI_SUCCESS
 *   QI_ERR_GENERAL
 *
 * Notes: none
 *==========================================================================*/
int QExifComposer::EmitEXIF()
{
  int lrc;

  // Allocate internal exif info object if none is supplied
  if (!p_exif_info) {
    exif_info_obj_t exif_info_obj;
    QIDBG_MED("%s:%d]: External EXIF", __func__, __LINE__);
    lrc = exif_init(&exif_info_obj);
    if (JPEG_SUCCEEDED(lrc)) {
      p_exif_info = (exif_info_t *)exif_info_obj;
      is_exif_info_owned = true;
    } else {
      // Better to return it immediately, else, later point exif_info_obj
      // or equivalently  p_writer->p_exif_info may get de-refrenced.
      QIDBG_ERROR("%s:%d] : exif_init failed\n", __func__, __LINE__);
      return lrc;
    }
  }

  JWRITE_SHORT(0xFF00 | M_SOI, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  EmitApp1();

  lrc = FlushFileHeader();

  mBuffer->SetFilledLen(mCurrentOffset);
  if (lrc == QI_SUCCESS) {
    mObserver.WriteComplete(*mBuffer);
  } else {
    mObserver.WriteError(QImageWriterObserver::ERR_GENERAL);
  }

  fHeaderWritten = true;
  return lrc;
}

/*===========================================================================
 * Function: EmitAppByType
 *
 * Description: This function is used to emit App based on payload type
 *
 * Input parameters:
 *   @aPayload_type: Exif payload type to emit
 *   @pPayload: Pointer to the payload
 *   @aPayload_length: Length of the payload
 *   @aPayload_written: Length of payload written so far
 *   @aRec_cnt: Recurssion count
 *
 * Return values:
 *   none
 *
 * Notes:
 *==========================================================================*/
void QExifComposer::EmitAppByType(ExifPayloadType aPayload_type,
  char *pPayload, uint32_t aPayload_length, uint32_t aPayload_written,
  uint8_t aRec_cnt)
{
  uint32_t overflow_length = 0;
  uint32_t app_header_size = 0;
  const char *p_magic_str = NULL;
  uint8_t magic_str_len = 0;

  if (pPayload == NULL) {
    QIDBG_ERROR("%s:%d] Exif payload NULL for type %d",__func__, __LINE__,
      aPayload_type);
    return;
  }

  switch (aPayload_type) {
  case EXIF_STATS_DBG_DATA:
    p_magic_str = stats_debug_magic_string;
    magic_str_len = sizeof(stats_debug_magic_string);

    if (aRec_cnt == 0) {
      app_header_size += STATS_SW_VERSION_SZ + FMT_3A_VER_SIZE;
    }
    break;
  case EXIF_MOBICAT_DATA:
    p_magic_str = mobicat_magic_string;
    magic_str_len = sizeof(mobicat_magic_string) - 1;
    break;
  case EXIF_DUAL_CAM_DATA:
    p_magic_str = mi_magic_string;
    magic_str_len = sizeof(mi_magic_string) - 1;
    break;
  default:
    QIDBG_ERROR("%s:%d] Invalid payload type",__func__, __LINE__);
    return;
  }

  // 2 bytes(next app length) + magic_str_len
  app_header_size += 2 + magic_str_len;

  // Check overflow
  if ((aPayload_length + app_header_size) > 0xFFFF) {
    overflow_length = aPayload_length + app_header_size - 0xFFFF;
    aPayload_length = 0xFFFF - app_header_size;
    QIDBG_HIGH("%s:%d] large payload, setting payload to %d "
      "overflow %d",__func__, __LINE__, aPayload_length, overflow_length);
  }

  // add payload length
  app_header_size += aPayload_length;

  // Write APP marker incrementally starting with APP4, 2 bytes
  JWRITE_SHORT(0xFF00 | (0xE4 + mExitAppMarkerInc), mBuffer->Addr(),
    mCurrentOffset, mBuffer->Length(), overflow_flag);

  // Write Size of Data inside the App header, 2 bytes
  JWRITE_SHORT((uint16_t)app_header_size, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  // Write magic string
  JWRITE_NBYTES((uint8_t *)p_magic_str, magic_str_len,
    mBuffer->Addr(), mCurrentOffset, mBuffer->Length(), overflow_flag);

  // if type STATS_DATA, write the 3a stats debug versions
  if ((aPayload_type == EXIF_STATS_DBG_DATA) && (aRec_cnt  == 0)) {

    // Write fmt version number, 4 bytes
    // Major Revision|Minor Revision|Patch Revision
    // 5.2.3 = 05.02.0003
    JWRITE_LONG((5<<24) | (2<<16) | (3<<0), mBuffer->Addr(), mCurrentOffset,
      mBuffer->Length(), overflow_flag);

    // Write 3A SW version number, 8 bytes
    // Major Revision|Minor Revision|Patch Revision|New Feature Designator
    // 5.0.0.1 = 0005.0000.0000.0001
    uint8_t def_swversion_3a[STATS_SW_VERSION_SZ];
    uint16_t* swversion_3a = mWriterParams->Get3ASWVer();
    if (swversion_3a == NULL) {
      memset(def_swversion_3a, 0, STATS_SW_VERSION_SZ);
      swversion_3a = (uint16_t *)def_swversion_3a;
    }
    JWRITE_NBYTES(swversion_3a, STATS_SW_VERSION_SZ, mBuffer->Addr(),
      mCurrentOffset, mBuffer->Length(), overflow_flag);
  }

  // Write stats payload
  JWRITE_NBYTES((uint8_t *)(pPayload + aPayload_written), aPayload_length,
    mBuffer->Addr(), mCurrentOffset, mBuffer->Length(), overflow_flag);

  // If overflow data exists recurse till no overflow left
  if (overflow_length > 0) {
    EmitAppByType(aPayload_type, pPayload, overflow_length,
      (aPayload_written + aPayload_length), (aRec_cnt + 1));
  } else {
    QIDBG_HIGH("%s:%d] EXIF payload type %d write successful to App%d",
      __func__, __LINE__, aPayload_type, (4 + mExitAppMarkerInc));
  }

  // Incriment App marker
  if (aRec_cnt == 0) {
    mExitAppMarkerInc++;
  }
}

/*===========================================================================
 * Function: EmitApp4
 *
 * Description: This function is used to emit App4 marker
 *
 * Input parameters:
 *   none
 *
 * Return values:
 *   none
 *
 * Notes: Used for Stats tuning params
 *==========================================================================*/
void QExifComposer::EmitApp4()
{
  char magic_str[] = "Qualcomm Camera Debug";
  uint32_t overflow_length = 0;
  char* stats_payload = NULL;
  uint32_t payload_length;
  // app marker + magic_str + fmtversion + swversion
  uint32_t size_of_identifiers = 2 + sizeof(magic_str)
    + SW_3A_VER_SIZE + FMT_3A_VER_SIZE;
  if (mWriterParams->Get3AFlag()) {
    stats_payload = mWriterParams->Get3A();
    payload_length = mWriterParams->Get3ASize();
    QIDBG_HIGH("%s:%d] [MOBICAT_DBG] payload_length %d",
      __func__, __LINE__, payload_length);
  } else {
    payload_length = 0;
    stats_payload = NULL;
  }

  // Write APP4 marker, 2 bytes
  JWRITE_SHORT(0xFF00 | 0xE4, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  // Check overflow
  if ((payload_length + size_of_identifiers) > 0xFFFF) {
    overflow_length = payload_length + size_of_identifiers - 0xFFFF;
    payload_length = 0xFFFF - size_of_identifiers;
    QIDBG_HIGH("%s:%d] [MOBICAT_DBG] large payload, setting to %d "
      "overflow %d",__func__, __LINE__, payload_length, overflow_length);
  }

  // Write Size of Data inside the App header, 2 bytes
  JWRITE_SHORT((uint16_t)(payload_length + size_of_identifiers),
    mBuffer->Addr(), mCurrentOffset, mBuffer->Length(), overflow_flag);

  // Write unique identifier used to distinguish
  // APP4 marker. APP4 always begins with
  // "Qualcomm Camera Debug " = 22 bytes
  JWRITE_NBYTES((uint8_t *)magic_str, sizeof(magic_str),
    mBuffer->Addr(), mCurrentOffset, mBuffer->Length(), overflow_flag);

  // Write version number
  // Major Revision|Minor Revision|Patch Revision
  // 5.2.3 = 05.02.0003
  JWRITE_LONG((5<<24)|(2<<16)|(3<<0), mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  // Write 3A SW version number (8 bytes)
  // Major Revision|Minor Revision|Patch Revision|New Feature Designator
  // 5.0.0.1 = 0005.0000.0000.0001
  uint8_t def_swversion_3a[STATS_SW_VERSION_SZ];
  uint16_t* swversion_3a = mWriterParams->Get3ASWVer();
  if (swversion_3a == NULL) {
    memset(def_swversion_3a, 0, STATS_SW_VERSION_SZ);
    swversion_3a = (uint16_t *)def_swversion_3a;
  }
  JWRITE_NBYTES(swversion_3a, 8, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  // Write 3A Payload
  if (mWriterParams->Get3AFlag()) {
    JWRITE_NBYTES((uint8_t *)stats_payload, payload_length,
      mBuffer->Addr(), mCurrentOffset, mBuffer->Length(), overflow_flag);
    if (overflow_length > 0) {
      EmitApp5(overflow_length, payload_length);
    } else {
      QIDBG_HIGH("%s:%d] [MOBICAT_DBG] stats debug write successful",
        __func__, __LINE__);
    }
  }
}

/*===========================================================================
 * Function: EmitApp5
 *
 * Description: This function is used to emit App5 marker
 *
 * Input parameters:
 *   none
 *
 * Return values:
 *   none
 *
 * Notes: Used for Stats tuning params when overflow occurs from App4
 *==========================================================================*/
void QExifComposer::EmitApp5(uint32_t payload_length, uint32_t payload_written)
{
  char magic_str[] = "Qualcomm Camera Debug";
  uint32_t overflow_length = 0;
  uint32_t size_of_identifiers;
  char* stats_payload = mWriterParams->Get3A();
  QIDBG_HIGH("%s:%d] [MOBICAT_DBG] payload_length %d",
    __func__, __LINE__, payload_length);

  // app marker + magic_str
  size_of_identifiers = 2 + sizeof(magic_str);

  if ((payload_length + size_of_identifiers) > 0xFFFF) {
    overflow_length = payload_length + size_of_identifiers - 0xFFFF;
    payload_length = 0xFFFF - size_of_identifiers;
    QIDBG_HIGH("%s:%d] [MOBICAT_DBG] large payload, setting to %d "
      "overflow %d",__func__, __LINE__, payload_length, overflow_length);
  }

  // Write APP5 marker, 2 bytes
  JWRITE_SHORT(0xFF00 | 0xE5, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  // Write Size of Data inside the App header, 2 bytes
  JWRITE_SHORT((uint16_t)(payload_length + size_of_identifiers),
    mBuffer->Addr(), mCurrentOffset, mBuffer->Length(), overflow_flag);

  // Write unique identifier used to distinguish APP5 marker.
  // APP5 always begins with "Qualcomm Camera Debug " = 22 bytes
  JWRITE_NBYTES((uint8_t *)magic_str, sizeof(magic_str),
    mBuffer->Addr(), mCurrentOffset, mBuffer->Length(), overflow_flag);

  // Write 3A Overflow Payload
  JWRITE_NBYTES((uint8_t *)stats_payload + payload_written, payload_length,
    mBuffer->Addr(), mCurrentOffset, mBuffer->Length(), overflow_flag);

  if (overflow_length > 0) {
    EmitApp6(overflow_length, payload_written + payload_length);
  } else {
    QIDBG_HIGH("%s:%d] [MOBICAT_DBG] stats debug write successful",
      __func__, __LINE__);
  }
}


/*===========================================================================
 * Function: EmitApp6
 *
 * Description: This function is used to emit App6 marker
 *
 * Input parameters:
 *   none
 *
 * Return values:
 *   none
 *
 * Notes: Used for Stats tuning params when overflow occurs from App5
 *==========================================================================*/
void QExifComposer::EmitApp6(uint32_t payload_length, uint32_t payload_written)
{
  char magic_str[] = "Qualcomm Camera Debug";
  uint32_t overflow_length = 0;
  uint32_t size_of_identifiers;
  char* stats_payload = mWriterParams->Get3A();
  QIDBG_HIGH("%s:%d] [MOBICAT_DBG] payload_length %d",
    __func__, __LINE__, payload_length);

  // app marker + magic_str
  size_of_identifiers = 2 + sizeof(magic_str);

  if ((payload_length + size_of_identifiers) > 0xFFFF) {
    overflow_length = payload_length + size_of_identifiers - 0xFFFF;
    payload_length = 0xFFFF - size_of_identifiers;
    QIDBG_HIGH("%s:%d] [MOBICAT_DBG] large payload, setting to %d "
      "overflow %d",__func__, __LINE__, payload_length, overflow_length);
  }

  // Write APP6 marker, 2 bytes
  JWRITE_SHORT(0xFF00 | 0xE6, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  // Write Size of Data inside the App header, 2 bytes
  JWRITE_SHORT((uint16_t)(payload_length + size_of_identifiers),
    mBuffer->Addr(), mCurrentOffset, mBuffer->Length(), overflow_flag);

  // Write unique identifier used to distinguish APP6 marker.
  // APP6 always begins with "Qualcomm Camera Debug " = 22 bytes
  JWRITE_NBYTES((uint8_t *)magic_str, sizeof(magic_str),
    mBuffer->Addr(), mCurrentOffset, mBuffer->Length(), overflow_flag);

  // Write 3A Overflow Payload
  JWRITE_NBYTES((uint8_t *)stats_payload + payload_written, payload_length,
    mBuffer->Addr(), mCurrentOffset, mBuffer->Length(), overflow_flag);

  if (overflow_length > 0) {
    QIDBG_ERROR("%s:%d] [MOBICAT_DBG] Stats payload too large, "
      "discarding overflow of size %d",__func__, __LINE__, overflow_length);
  } else {
    QIDBG_HIGH("%s:%d] [MOBICAT_DBG] stats debug write successful",
      __func__, __LINE__);
  }
}

/*===========================================================================
 * Function: EmitApp7
 *
 * Description: This function is used to emit App7 marker
 *
 * Input parameters:
 *   none
 *
 * Return values:
 *   none
 *
 * Notes: Used for mobicat tuning params
 *==========================================================================*/
void QExifComposer::EmitApp7()
{
  char* mobicatStr = mWriterParams->GetMobicat();
  uint32_t mobicat_total_length = (uint32_t)strlen(mobicatStr);

  // Write APP7 marker
  JWRITE_SHORT(0xFF00 | 0xE7, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  // Write Length of APP7 header
  // + 3 to accomidate 2 bytes for mobicate payload length and
  // 1 byte for magic string length
  JWRITE_SHORT((uint16_t)(mobicat_total_length + 3), mBuffer->Addr(),
    mCurrentOffset, mBuffer->Length(), overflow_flag);

  // Write length of unique identifier used to
  // distinguish APP7 marker. APP5 always begins
  // with "Qualcomm Camera Attributes v2"
  JWRITE_BYTE(0x1D, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  // Write APP7 data
  JWRITE_NBYTES((uint8_t *)mobicatStr, mobicat_total_length,
    mBuffer->Addr(), mCurrentOffset, mBuffer->Length(), overflow_flag);
}

/*===========================================================================
 * Function: EmitApp8
 *
 * Description: This function is used to emit App8 marker
 *
 * Input parameters:
 *   none
 *
 * Return values:
 *   none
 *
 * Notes: Used for Dual camera params
 *==========================================================================*/
void QExifComposer::EmitApp8()
{
  char *metadataStr = mWriterParams->GetMIMetadata();
  int metadata_len = mWriterParams->GetMIMetaSize();

  // Write APP8 marker
  JWRITE_SHORT(0xFF00 | 0xE8, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  // Write Length of APP8 header
  //metadata_len + 2 bytes (app length) + 1 byte(magic str len)
  JWRITE_SHORT((uint16_t)( metadata_len + 2 + 1), mBuffer->Addr(),
    mCurrentOffset, mBuffer->Length(), overflow_flag);

  //Write length of unique identifier used to distinguish APP8
  //APP8 always begins with string "Qualcomm Dual Camera Attributes"
  JWRITE_BYTE(0x1F, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  // Write APP8 data
  JWRITE_NBYTES((uint8_t *)metadataStr, metadata_len,
    mBuffer->Addr(), mCurrentOffset, mBuffer->Length(), overflow_flag);
}

/*===========================================================================
 * Function: EmitJFIF
 *
 * Description: This function is used to emit JFIF header
 *
 * Input parameters:
 *   none
 *
 * Return values:
 *   QI_SUCCESS
 *   QI_ERR_GENERAL
 *
 * Notes: none
 *==========================================================================*/
int QExifComposer::EmitJFIF()
{
  int lrc = QI_SUCCESS;

  JWRITE_SHORT(0xFF00 | M_SOI, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  EmitApp0();

  lrc = FlushFileHeader();

  mBuffer->SetFilledLen(mCurrentOffset);
  if (lrc == QI_SUCCESS) {
    mObserver.WriteComplete(*mBuffer);
  } else {
    mObserver.WriteError(QImageWriterObserver::ERR_GENERAL);
  }
  fHeaderWritten = true;
  return QI_SUCCESS;
}

/*===========================================================================
 * Function: addBuffer
 *
 * Description: This function is used to enqueue the output buffer
 *
 * Input parameters:
 *   aBuffer - pointer to the buffer object
 *
 * Return values:
 *   none
 *
 * Notes: none
 *==========================================================================*/
int QExifComposer::addBuffer(QIBuffer *aBuffer)
{
  mBuffer = aBuffer;
  return QI_SUCCESS;
}

/*===========================================================================
 * Function: Start
 *
 * Description: This function is used to start the exif composer
 *
 * Input parameters:
 *   aThumbnail - pointer to the thumbnail object
 *   aType - type of jpeg image
 *   aSync - start the composer synchronous or asynchronouse
 *
 * Return values:
 *   none
 *
 * Notes: Only sync mode is supported as of now
 *==========================================================================*/
int QExifComposer::Start(QImage *aThumbnail, JpegHeaderType aType,
  int aSync __unused)
{
  mThumbnail = aThumbnail;
  mCurrentOffset = mBuffer->FilledLen();

  return (aType != EXIF) ? EmitJFIF() : EmitEXIF();
}

/*===========================================================================
 * Function: EmitFrameHeader
 *
 * Description: This function is used to emit frame header
 *
 * Input parameters:
 *   aParams - pointer to the encoder parameter object
 *   aSS - subsampling for the image
 *
 * Return values:
 *   none
 *
 * Notes: none
 *==========================================================================*/
void QExifComposer::EmitFrameHeader(QIEncodeParams &aParams, QISubsampling aSS)
{
  /* Emit DQT for each quantization table.
   * Note that emit_dqt() suppresses any duplicate tables.
   */
  JWRITE_SHORT(0xFF00 | M_DQT, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  JWRITE_SHORT(64*2 + 2 + 2, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  JWRITE_BYTE(0, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  EmitDQT(aParams.QuantTable(QIQuantTable::QTABLE_LUMA)->Table());

  JWRITE_BYTE(1, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  EmitDQT(aParams.QuantTable(QIQuantTable::QTABLE_CHROMA)->Table());

  EmitSOF(M_SOF0, aParams, aSS);  // SOF code for baseline implementation
}

/*===========================================================================
 * Function: EmitScanHeader
 *
 * Description: This function is used to emit scan header
 *
 * Input parameters:
 *   aParams - pointer to the encoder parameter object
 *
 * Return values:
 *   none
 *
 * Notes: none
 *==========================================================================*/
void QExifComposer::EmitScanHeader(QIEncodeParams &aParams)
{
  int i;
  uint16_t length;

  // Emit Huffman tables.
  JWRITE_SHORT(0xFF00 | M_DHT, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag);

  // Compute length
  length = 0;

  for (i = 1; i <= 16; i++) {
    length = (uint16_t)(length + (uint16_t)
      (aParams.HuffTable(QIHuffTable::HTABLE_LUMA_DC)->Table()->mBits[i]));
    length = (uint16_t)(length + (uint16_t)
      (aParams.HuffTable(QIHuffTable::HTABLE_LUMA_AC)->Table()->mBits[i]));
    length = (uint16_t)(length + (uint16_t)
      (aParams.HuffTable(QIHuffTable::HTABLE_CHROMA_DC)->Table()->mBits[i]));
    length = (uint16_t)(length + (uint16_t)
      (aParams.HuffTable(QIHuffTable::HTABLE_CHROMA_AC)->Table()->mBits[i]));
  }

  //lint -e734 length + 17 * 4 + 2 will never overflow 16 bits
  JWRITE_SHORT(2 + length + 17*4, mBuffer->Addr(), mCurrentOffset,
    mBuffer->Length(), overflow_flag); // 4 Huffman tables

  EmitDHT(aParams.HuffTable(QIHuffTable::HTABLE_LUMA_DC)->Table(), 0);
  EmitDHT(aParams.HuffTable(QIHuffTable::HTABLE_LUMA_AC)->Table(), 0 | 0x10);
  EmitDHT(aParams.HuffTable(QIHuffTable::HTABLE_CHROMA_DC)->Table(), 1);
  EmitDHT(aParams.HuffTable(QIHuffTable::HTABLE_CHROMA_AC)->Table(), 1 | 0x10);

  /*
   * Emit DRI
   */
  if (aParams.RestartInterval() > 0) {
    EmitDRI((uint16_t) aParams.RestartInterval());
  }

  EmitSOS(aParams);
}
