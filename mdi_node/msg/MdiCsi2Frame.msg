std_msgs/Header header
MdiAvetoProfile mdi_info
# this is the offset IN data to the actual csi2 lines
uint32      offset_to_payload
# csi2 lines are stored one after another, see description at the end
uint8[]     data


# /* assume the following C types */
# typedef union SCSI2PacketHdr
# {
# 	struct SCSI2ShortHdr
# 	{
# 		uint32_t uiDataType : 6;			/*!< Data type between 0x00 and 0x0f for short packet. */
# 		uint32_t uiVirtualChannel : 2;
# 		uint32_t uiData : 16;
# 		uint32_t uiECC : 8;
# 	} sShortHeader;
# 
# 	struct SCSILongHdr
# 	{
# 		uint32_t uiDataType : 6;			/*!< Data type 0x10 or larger for long packet. */
# 		uint32_t uiVirtualChannel : 2;
# 		uint32_t uiWordCount : 16;
# 		uint32_t uiECC : 8;
# 	} sLongHeader;
# } SCSI2PacketHdr_t;
# typedef struct SCSI2FpgaPreLineHdr
# {
# 	uint64_t uiTimestamp;				/*!< Synchronized/unsynchronized hardware timestamp. */
# 	uint32_t uiFrameCounter : 8;		/*!< Counter incrementing with every frame start packet. */
# 	uint32_t uiLineCounter : 24;		/*!< Counter incremented with every line. */
# } SCSI2FpgaPreLineHdr_t;
# typedef struct SCSI2RawLineFooter
# {
# 	uint16_t		uiCSI2CRC;		/*!< CSI2 specification 1.1 chapter 9.6 (Checksum Generation). */
# 	uint16_t		uiFpgaFlags;	/*!< FPGA generated footer - reserved for future use. */
# } SCSI2RawLineFooter_t;
#
# /* now lines are stored as following:
#    1. if it is a "short" packackage (data type < 0x10) only
#       SCSI2PacketHdr_t::SCSI2ShortHdr applies - after this structure, the next header starts
#    2. if it is a "long" package (data type >= 0x10) SCSI2PacketHdr_t::SCSILongHdr, SCSI2FpgaPreLineHdr
#       and SCSI2RawLineFooter applies.
#
#                  .----------------------------CSI2 data line-----------------------------------.
#  +---------------+-------------+---------------------+--------------------+--------------------+----------.....
#  | SCSI2ShortHdr | SCSILongHdr | SCSI2FpgaPreLineHdr | ---line content--- | SCSI2RawLineFooter | SCSILongHdr .....
#  +---------------+-------------+---------------------+--------------------+--------------------+----------.....
#                         |                             `----Word Count----´
#                          `--------------------------------´