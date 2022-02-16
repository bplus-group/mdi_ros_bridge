#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "DAQProtTypes_V2_1_extract.h"
#pragma GCC diagnostic push

using namespace AVETO::DAQProt_v201;


#pragma pack(1)

struct SUniqueID_t
{
  uint8_t   MDI_reserved[3];
  uint8_t   Index;
  uint16_t  DataType;
  uint8_t   Instance;
  uint8_t   Channel;
};

struct AvetoHeaderV2x1_Proto
{
  SFrameHdr_t               frame;
  SCycleInfoHdrExt_t        cycle;
  STimeInfoHdrExt_t         time[3];
};

/* one step further down the road: now with payload */

struct AvetoHeaderV2x1_CSI2Raw
{
  struct AvetoHeaderV2x1_Proto        Base;
  SCSI2HdrExt_t                       BusProp;
  SCSI2RawLineCollection_t            CSI2RawLines;
  /* PSEUDOCODE *//*
  union csi2_lines
  {
    struct long_package
    {
      SCSI2RawLineHeader_t  header;
      uint8_t               payload[ANYSIZE_ARRAY];
      SCSI2RawLineFooter_t  footer;
    };
    struct short_package
    {
      SCSI2RawLineHeader_t    header;
    };
  } [ANYSIZE_ARRAY];
  */
};

struct AvetoHeaderV2x1_JSONString
{
  struct AvetoHeaderV2x1_Proto      Base;
  SJSONStatusString_t               String;
};
