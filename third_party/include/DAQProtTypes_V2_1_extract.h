/********************************************************************************************************************************/
/*! \file
*
* \verbatim
**********************************************************************************************************************************
*																																 *
*  Copyright (c) 2015-2021, b-plus GmbH. All rights reserved!																	 *
*																																 *
*    All rights are exclusively reserved by b-plus GmbH,																		 *
*    unless explicitly agreed otherwise.																						 *
*																																 *
*    Redistribution in source or any other form,																				 *
*    with or without modification, is not permitted.																			 *
*																																 *
*    You may use this code according to the license terms of b-plus.															 *
*    Please contact b-plus at services@b-plus.com to get the actual																 *
*    terms and conditions.																										 *
*																																 *
**********************************************************************************************************************************
\endverbatim
*
* \copyright (C)2015-2021 b-plus GmbH
* \date 17.07.2019
* \version 2.1e
*
* \attention This header file represents the structures described in the AVETO 2 DAQ Protocol specification. The header file
* is to be considered as additional information. No guarantee is providing on the correctness of the structures in this file.
* If any irregularities between this header file and the specification exist, the specification has the correct information
* and thus overrides the information in this header file.
*
* \brief Definition of the AVETO 2 DAQ Protocol structures.
* \details The b-plus AVETO 2 DAQ Protocol is used to transmit data of various sources in a generic way allowing the abstract 
* processing of the data without having to interpret the content.
*
*********************************************************************************************************************************/
#ifndef AVETO_HEADER_DEFINITION_V201
#define AVETO_HEADER_DEFINITION_V201

#include <stdint.h>

#ifdef __cplusplus

/**
* \brief Current version of the DAQProt interface
*/
#define DAQProt_v201 DAQProt

/**
* \brief Current version
*/
#define AVETO_DAQPROT_VERSION 201

namespace AVETO
{
	namespace DAQProt_v201
	{
		extern "C" {
#endif /* __cplusplus */

#if  __GNUC__
# pragma pack(push)
# pragma pack(1)
#elif _MSC_VER && !__INTEL_COMPILER
# pragma pack(push)
# pragma pack(1)
#else 
# error "Your compiler is not integrated. Please add the correct pack pragmas!"
#endif

/***************************************************************************
** Generic base header                                                    **
***************************************************************************/

/**
* \brief Mask type (big or little).
* \{
*/
#define DAQPROT_HDR_FLAG_BIG_ENDIAN		0x80	/*!< The header and payload is stored in big endian format unless explicitly	*/
												/*!< stated otherwise. */
#define DAQPROT_HDR_FLAG_LITTLE_ENDIAN	0x00	/*!< The header and payload is stored in little endian format unless explicitly	*/
												/*!< stated otherwise. */
#define DAQPROT_HDR_FLAG_COMPACT_HDR	0x40	/*!< The header represents a compact header. This only applies to packets. */
#define DAQPROT_HDR_FLAG_STD_HDR		0x00	/*!< The header represents a standard header (default). */
#define DAQPROT_HDR_FLAG_CTRL_HDR		0x10	/*!< The header refers to a control packet header. This only applies to			*/
												/*!< packets. */
#define DAQPROT_HDR_FLAG_VERSION_MASK	0x0f	/*!< The header version (only valid for standard header).						*/
/**
* \}
*/

/**
* \{
* \brief Protocol version number.
*/
#define DAQPROT_HDR_VERSION_MAJOR	0x02		/*!< The major version. */
#define DAQPROT_HDR_VERSION_MINOR	0x01		/*!< The minor version. */
/**
* \}
*/

/**
* \brief Base header.
*/
typedef struct SDAQHdr
{
	uint8_t		uiMaskVersionMajor;		/*!< MSB bit 7 is endianess (big = 1; little = 0); bit 6 is compact header				*/
										/*!< (compact = 1) and 4 bits are major version (currently version 2). */
	uint8_t		uiVersionMinor;			/*!< Minor version. */
	uint16_t	uiHdrType;				/*!< The type of header interface source. */
	uint16_t	uiHdrLen;				/*!< Length of the header. */
	uint16_t	uiHdrChecksum;			/*!< Optional checksum of the header; ones' complement of the ones' complement sum of	*/
										/*!< the header's 16-bit words with the checksum value set to zero during calculation.	*/
										/*!< Or zero when not used. */
} SDAQHdr_t;

/***************************************************************************
** Header extensions                                                      **
***************************************************************************/

/**
* \{
* \brief Header extension types.
*/
#define DAQPROT_HDREXT_TYPE_FIRST				0x1000		/*!< First header type associated with extensions. */
#define DAQPROT_HDREXT_TYPE_LAST				0x1FFF		/*!< Last header type associated with extensions. */
#define DAQPROT_HDREXT_TYPE_TIMEBASE			0x1000		/*!< Timebase header (STimeInfoHdrExt_t used). */
#define DAQPROT_HDREXT_TYPE_INTEGRITY			0x1001		/*!< Integrity header (SIntegrityInfoHdrExt_t used). */
#define DAQPROT_HDREXT_TYPE_CYCLE			    0x1002		/*!< Cycle info header (SCycleInfoHdrExt_t used). */
#define DAQPROT_HDREXT_TYPE_IMAGE				0x1100		/*!< Image information header (SImageInfoHdrExt_t used). */
#define DAQPROT_HDREXT_TYPE_MDI_CSI2			0x1108		/*!< MDI data received over CSI-2 (SCSI2HdrExt_t used). */
#define DAQPROT_HDREXT_TYPE_I2C					0x1109		/*!< I�C frame (SI2CHdrExt_t used).*/
#define DAQPROT_HDREXT_TYPE_AURORA				0x1110		/*!< Aurora (XILINX) frame (SAuroraHdrExt_t used). */
#define DAQPROT_HDREXT_TYPE_CAN					0x1140		/*!< CAN frame (SCANHdrExt_t used). */
#define DAQPROT_HDREXT_TYPE_FLEXRAY				0x1141		/*!< FlexRay frame (SFlexRayHdrExt_t used). */
/**
* \}
*/

/***************************************************************************
** Timebase header extension (header type DAQPROT_HDREXT_TYPE_TIMEBASE)   **
***************************************************************************/

/**
* \brief Timebase types
* \{
*/
#define DAQPROT_TIMEBASE_TAI		        0x0000	/*!< Temp atomic international used (XTSS synchronization in nano-seconds). */
#define DAQPROT_TIMEBASE_WCD		        0x0001	/*!< Working clock domain (XTSS synchronization in nano-seconds). */
#define DAQPROT_TIMEBASE_UTC		        0x0002	/*!< UTC derived from TAI (XTSS synchronization in nano-seconds). */
#define DAQPROT_TIMEBASE_LOCAL_MICROSEC		0x1000	/*!< Local timebase (in micro-seconds). */
#define DAQPROT_TIMEBASE_LOCAL_NANOSEC	    0x1001	/*!< Local timebase (in nano-seconds). */
#define DAQPROT_TIMEBASE_RELATIVE_MICROSEC	0x2000	/*!< Time is relative (in micro-seconds) to start of frame - only for data	*/
													/*!< packets. */
#define DAQPROT_TIMEBASE_RELATIVE_NANOSEC	0x2001	/*!< Time is relative (in nano-seconds) to start of frame - only for data	*/
													/*!< packets. */
#define DAQPROT_TIMEBASE_CUSTOM				0xfffe	/*!< Custom timebase (source not known). */
#define DAQPROT_TIMEBASE_UNKNOWN	        0xffff	/*!< Unknown timebase. */
/**
* \}
*/

/**
* \brief Timestamp Flag types
* \{
*/
#define DAQPROT_TIMEFLAG_UNSYNCED		        0x0000		/*!< The specific timebase is not in a synchronized state and might	*/
															/*!< be inaccurate. */
#define DAQPROT_TIMEFLAG_SYNCED			        0x0001		/*!< The specific timebase is in a synchronized state */
#define DAQPROT_TIMEFLAG_64BIT					0x0000		/*!< The timebase uses timestamps of 64 bits. */
#define DAQPROT_TIMEFLAG_32BIT					0x0010		/*!< The timebase uses timestamps of 32 bits. */
/**
* \}
*/

/**
* \brief Timebase structure header extension
*/
typedef struct STimeInfoHdrExt
{
	SDAQHdr_t		sHdr;				/*!< Base header identifying the type, length and version of the header as well as a	*/
										/*!< header checksum. */
	uint16_t		uiTimebaseType;		/*!< Timebase type. One of the DAQPROT_TIMEBASE_ types. */
	uint16_t		uiReserved1;		/*!< Reserved (used for backwards compatibility). */
	uint16_t		uiTimebaseFlags;	/*!< Timestamp flags indicating the quality of the timestamp. */
	uint16_t		uiReserved2;		/*!< Reserved (used for backwards compatibility). */
	uint64_t		uiTimestamp;		/*!< Timestamp. */
} STimeInfoHdrExt_t;

/***************************************************************************
** Integrity header extension (header type DAQPROT_HDREXT_TYPE_INTEGRITY) **
***************************************************************************/

/**
* \brief Integrity hash types
* \remarks Exception is the value 0.
* \{
*/
#define DAQPROT_HASH_TYPE_NONE				0x0000	/*!< No integrity */
#define DAQPROT_HASH_TYPE_CRC8				0x0080	/*!< 8-bits CRC calculation using supplied optional startvalue and          */
													/*!< polynomial, input/output reflection, initial value and final XOR		*/
													/*!< value are to be supplied. SIntegrityInfoHdrExtCRC8Gen structure		*/
													/*!< is used. */
#define DAQPROT_HASH_TYPE_CRC16				0x0081	/*!< 16-bits CRC calculation using supplied optional startvalue and	        */
													/*!< polynomial, input/output reflection, initial value and final XOR		*/
													/*!< value are to be supplied. SIntegrityInfoHdrExtCRC16Gen structure		*/
													/*!< is used. */
#define DAQPROT_HASH_TYPE_CRC32				0x0082	/*!< 32-bits CRC calculation using supplied optional startvalue and         */
													/*!< polynomial, input/output reflection, initial value and final XOR		*/
													/*!< value are to be supplied. SIntegrityInfoHdrExtCRC32Gen structure		*/
													/*!< is used. */
#define DAQPROT_HASH_TYPE_CRC64				0x0083	/*!< 64-bits CRC calculation using supplied optional startvalue and	        */
													/*!< polynomial, input/output reflection, initial value and final XOR		*/
													/*!< value are to be supplied. SIntegrityInfoHdrExtCRC64Gen structure		*/
													/*!< is used. */
#define DAQPROT_HASH_TYPE_CRC8_8H2F			0x0010	/*!< CRC8-8H2F (AUTOSAR) calculation with 8-bits. Optional start value can	*/
													/*!< be supplied. Polynomial is 0x2F. Both input an output are reflected.	*/
													/*!< The initial value is 0xFF and the final XOR value also is 0xFF.		*/
													/*!< SIntegrityInfoHdrExt8 structure is used. */
#define DAQPROT_HASH_TYPE_CRC8_SAE_J1850	0x0011	/*!< CRC8-SAE J1850 (AUTOSAR) calculation with 8-bits. Optional start value	*/
													/*!< can be supplied. Polynomial is 0x1D. Both input an output are not		*/
													/*!< reflected. The initial value is 0xFF and the final XOR value also is	*/
													/*!< 0xFF. SIntegrityInfoHdrExt8 structure is used. */
#define DAQPROT_HASH_TYPE_CRC16_CCIT_ZERO	0x0030	/*!< CRC16-CCITT ZERO calculation with 16-bits. Optional start value can be	*/
													/*!< supplied. Polynomial is 0x1021. Both input an output are not			*/
													/*!< reflected. The initial value is 0x0000 and the final XOR value also is	*/
													/*!< 0x0000. SIntegrityInfoHdrExt16 structure is used. */
#define DAQPROT_HASH_TYPE_CRC16_CCIT_FALSE	0x0031	/*!< CRC16-CCITT FALSE calculation with 16-bits. Optional start value can 	*/
													/*!< be supplied. Polynomial is 0x1021. Both input an output are reflected.	*/
													/*!< The initial value is 0xFFFF and the final XOR value is 0x0000.			*/
													/*!< SIntegrityInfoHdrExt16 structure is used. */
#define DAQPROT_HASH_TYPE_CRC32C			0x0020	/*!< Castagnioli CRC32 calculation with 32-bits. Optional start value can	*/
													/*!< be supplied. Polynomial is 0x1EDC6F41. Both input an output are		*/
													/*!< reflected. The initial value is 0xFFFFFFFF and the final XOR value 	*/
													/*!< also is 0xFFFFFFFF. SIntegrityInfoHdrExt32 structure is used. */
#define DAQPROT_HASH_TYPE_CRC32_IEEE		0x0021	/*!< IEEE 802.3 CRC32 calculation with 32-bits. Optional start value can be	*/
													/*!< supplied. Polynomial is 0x04C11DB7. Both input an output are			*/
													/*!< reflected. The initial value is 0xFFFFFFFF and the final XOR value 	*/
													/*!< also is 0xFFFFFFFF. SIntegrityInfoHdrExt32 structure is used. */
#define DAQPROT_HASH_TYPE_CRC32_AUTOSAR		0x0022	/*!< AUTOSAR CRC32 calculation with 32-bits. Optional start value can be	*/
													/*!< supplied. Polynomial is 0xF4ACFB13. Both input an output are			*/
													/*!< reflected. The initial value is 0xFFFFFFFF and the final XOR value 	*/
													/*!< also is 0xFFFFFFFF. SIntegrityInfoHdrExt32 structure is used. */
#define DAQPROT_HASH_TYPE_CRC64_AUTOSAR		0x0022	/*!< AUTOSAR CRC64 calculation with 64-bits. Optional start value can be	*/
													/*!< supplied. Polynomial is 0x42F0E1EBA9EA3693. Both input an output are	*/
													/*!< reflected. The initial value is 0xFFFFFFFFFFFFFFFF and the final XOR	*/
													/*!< value also is 0xFFFFFFFFFFFFFFFF. SIntegrityInfoHdrExt64 structure is	*/
													/*!< used. */
#define DAQPROT_HASH_TYPE_CHECKSUM16_IPV4	0x1000	/*!< IPv4 Checksum 16 bits, ones' complement of the ones' complement sum of	*/
													/*!< the header's 16-bit words. Optional start value can be supplied.		*/
													/*!< SIntegrityInfoHdrExt16 structure is used. */

#define DAQPROT_HASH_TYPE_XXHASH32			0x0100
#define DAQPROT_HASH_TYPE_XXHASH64			0x0101

/**
* \}
*/

/**
* \{
* \brief Integrity definition flags.
*/
#define DAQPROT_HASH_FLAG_USE_START_VALUE	0x0001	/*!< When set, the calculation uses the result of a previous calculation to	*/
													/*!< generate an hash value over multiple chunks of data. By default this	*/
													/*!< bit is disabled causing the calculation to take the value zero.		*/
													/*!< ATTENTION: The start value is not to be mistaken by the initial value	*/
													/*!< used for CRC calculations. */
#define DAQPROT_HASH_FLAG_FINAL_XOR_NEG		0x0002	/*!< When set, the final value will XOR with -1. Only valid for generic CRC	*/
													/*!< calculations. */
#define DAQPROT_HASH_FLAG_INITIAL_NEG		0x0004	/*!< When set, the initial value will be -1. Otherwise the initial value	*/
													/*!< will be 0. Only valid for generic CRC calculations. */
#define DAQPROT_HASH_FLAG_REFLECT_OUTPUT	0x0008	/*!< When set, the output is reflected (bit order is reversed). Only valid	*/
													/*!< for generic CRC calculations. */
#define DAQPROT_HASH_FLAG_REFLECT_INPUT		0x0010	/*!< When set, the input is reflected (bit order is reversed). Only valid	*/
													/*!< for generic CRC calculations. */
/**
* \}
*/

/**
* \brief Base integrity information structure .
* \remarks Dependable on the hash type, the integrity info structure is extended with hash calculation information.
*/
typedef struct SIntegrityInfoHdrExtBase
{
	SDAQHdr_t			sHdr;				/*!< Base header identifying the type, length and version of the header as well as	*/
											/*!< a header checksum. */
	uint32_t			uiSeqCnt;			/*!< Optional sequence counter used to detect lost frames. Zero when not used. */
	uint16_t			uiHashType;			/*!< Hash type. */
	uint16_t			uiFlags;			/*!< Integrity definition flags. */
	/* The hash value follows here. */
	/* The start value follows here. */
	/* For generic CRC calculations, the polynomial value follows here. */
} SIntegrityInfoHdrExtBase_t;

/**
* \{
* \brief Structures for dedicated 8, 16, 32, and 64-bits integrity information.
* \attention For the generic CRC calculations, other structures are defined.
* \remarks The structures might be added by padding bytes to enforce 64-bit alignment.
*/
typedef struct SIntegrityInfoHdrExt8
{
	SDAQHdr_t			sHdr;				/*!< Base header identifying the type, length and version of the header as well as	*/
											/*!< a header checksum. */
	uint32_t			uiSeqCnt;			/*!< Optional sequence counter used to detect lost frames. Zero when not used. */
	uint16_t			uiHashType;			/*!< Hash type (one of the DAQPROT_HASH_TYPE_xxx8 types). */
	uint16_t			uiFlags;			/*!< Integrity definition flags. */
	uint8_t				uiHashValue;		/*!< Hash value calculated. */
	uint8_t				uiStartValue;		/*!< The start value used for the hash value calculation using a previously			*/
											/*!< calculated value. The default value is 0. Only used when						*/
											/*!< DAQPROT_HASH_FLAG_USE_START_VALUE is set. */
	uint8_t				rguiReserved[6];	/*!< Reserved for alignment purposes. */
}SIntegrityInfoHdrExt8_t;
typedef struct SIntegrityInfoHdrExt16
{
	SDAQHdr_t			sHdr;				/*!< Base header identifying the type, length and version of the header as well as	*/
											/*!< a header checksum. */
	uint32_t			uiSeqCnt;			/*!< Optional sequence counter used to detect lost frames. Zero when not used. */
	uint16_t			uiHashType;			/*!< Hash type (one of the DAQPROT_HASH_TYPE_xxx16 types). */
	uint16_t			uiFlags;			/*!< Integrity definition flags. */
	uint16_t			uiHashValue;		/*!< Hash value calculated. */
	uint16_t			uiStartValue;		/*!< The start value used for the hash value calculation using a previously			*/
											/*!< calculated value. The default value is 0. Only used when						*/
											/*!< DAQPROT_HASH_FLAG_USE_START_VALUE is set. */
	uint16_t			rguiReserved[2];	/*!< Reserved for alignment purposes. */
}SIntegrityInfoHdrExt16_t;
typedef struct SIntegrityInfoHdrExt32
{
	SDAQHdr_t			sHdr;				/*!< Base header identifying the type, length and version of the header as well as	*/
											/*!< a header checksum. */
	uint32_t			uiSeqCnt;			/*!< Optional sequence counter used to detect lost frames. Zero when not used. */
	uint16_t			uiHashType;			/*!< Hash type (one of the DAQPROT_HASH_TYPE_xxx32 types). */
	uint16_t			uiFlags;			/*!< Integrity definition flags. */
	uint32_t			uiHashValue;		/*!< Hash value calculated. */
	uint32_t			uiStartValue;		/*!< The start value used for the hash value calculation using a previously			*/
											/*!< calculated value. The default value is 0. Only used when						*/
											/*!< DAQPROT_HASH_FLAG_USE_START_VALUE is set. */
}SIntegrityInfoHdrExt32_t;
typedef struct SIntegrityInfoHdrExt64
{
	SDAQHdr_t			sHdr;				/*!< Base header identifying the type, length and version of the header as well as	*/
											/*!< a header checksum. */
	uint32_t			uiSeqCnt;			/*!< Optional sequence counter used to detect lost frames. Zero when not used. */
	uint16_t			uiHashType;			/*!< Hash type (one of the DAQPROT_HASH_TYPE_xxx64 types). */
	uint16_t			uiFlags;			/*!< Integrity definition flags. */
	uint64_t			uiHashValue;		/*!< Hash value calculated. */
	uint64_t			uiStartValue;		/*!< The start value used for the hash value calculation using a previously			*/
											/*!< calculated value. The default value is 0. Only used when						*/
											/*!< DAQPROT_HASH_FLAG_USE_START_VALUE is set. */
}SIntegrityInfoHdrExt64_t;
/**
* \}
*/

/**
* \{
* \brief Structures for generic 8, 16, 32, and 64-bits CRC based integrity information.
* \attention For the dedicated CRC and checksum calculations, other structures are defined.
* \remarks The structures might be added by padding bytes to enforce 64-bit alignment.
*/
typedef struct SIntegrityInfoHdrExtCRC8Gen
{
	SDAQHdr_t			sHdr;				/*!< Base header identifying the type, length and version of the header as well as	*/
											/*!< a header checksum. */
	uint32_t			uiSeqCnt;			/*!< Optional sequence counter used to detect lost frames. Zero when not used. */
	uint16_t			uiHashType;			/*!< Hash type (DAQPROT_HASH_TYPE_CRC8). */
	uint16_t			uiFlags;			/*!< Integrity definition flags. */
	uint8_t				uiHashValue;		/*!< Hash value calculated. */
	uint8_t				uiStartValue;		/*!< The start value used for the hash value calculation using a previously			*/
											/*!< calculated value. The default value is 0. Only used when						*/
											/*!< DAQPROT_HASH_FLAG_USE_START_VALUE is set. */
	uint8_t				uiPolynomial;		/*!< The polynomial representation. */
	uint8_t				rguiReserved[5];	/*!< Reserved for alignment purposes. */
} SIntegrityInfoHdrExtCRC8Gen_t;
typedef struct SIntegrityInfoHdrExtCRC16Gen
{
	SDAQHdr_t			sHdr;				/*!< Base header identifying the type, length and version of the header as well as	*/
											/*!< a header checksum. */
	uint32_t			uiSeqCnt;			/*!< Optional sequence counter used to detect lost frames. Zero when not used. */
	uint16_t			uiHashType;			/*!< Hash type (DAQPROT_HASH_TYPE_CRC16). */
	uint16_t			uiFlags;			/*!< Integrity definition flags. */
	uint16_t			uiHashValue;		/*!< Hash value calculated. */
	uint16_t			uiStartValue;		/*!< The start value used for the hash value calculation using a previously			*/
											/*!< calculated value. The default value is 0. Only used when						*/
											/*!< DAQPROT_HASH_FLAG_USE_START_VALUE is set. */
	uint16_t			uiPolynomial;		/*!< The polynomial representation. */
	uint16_t			uiReserved;			/*!< Reserved for alignment purposes. */
} SIntegrityInfoHdrExtCRC16Gen_t;
typedef struct SIntegrityInfoHdrExtCRC32Gen
{
	SDAQHdr_t			sHdr;				/*!< Base header identifying the type, length and version of the header as well as	*/
											/*!< a header checksum. */
	uint32_t			uiSeqCnt;			/*!< Optional sequence counter used to detect lost frames. Zero when not used. */
	uint16_t			uiHashType;			/*!< Hash type (DAQPROT_HASH_TYPE_CRC32). */
	uint16_t			uiFlags;			/*!< Integrity definition flags. */
	uint32_t			uiHashValue;		/*!< Hash value calculated. */
	uint32_t			uiStartValue;		/*!< The start value used for the hash value calculation using a previously			*/
											/*!< calculated value. The default value is 0. Only used when						*/
											/*!< DAQPROT_HASH_FLAG_USE_START_VALUE is set. */
	uint32_t			uiPolynomial;		/*!< The polynomial representation. */
	uint32_t			uiReserved;			/*!< Reserved for alignment purposes. */
} SIntegrityInfoHdrExtCRC32Gen_t;
typedef struct SIntegrityInfoHdrExtCRC64Gen
{
	SDAQHdr_t			sHdr;				/*!< Base header identifying the type, length and version of the header as well as	*/
											/*!< a header checksum. */
	uint32_t			uiSeqCnt;			/*!< Optional sequence counter used to detect lost frames. Zero when not used. */
	uint16_t			uiHashType;			/*!< Hash type (DAQPROT_HASH_TYPE_CRC64). */
	uint16_t			uiFlags;			/*!< Integrity definition flags. */
	uint64_t			uiHashValue;		/*!< Hash value calculated. */
	uint64_t			uiStartValue;		/*!< The start value used for the hash value calculation using a previously			*/
											/*!< calculated value. The default value is 0. Only used when						*/
											/*!< DAQPROT_HASH_FLAG_USE_START_VALUE is set. */
	uint64_t			uiPolynomial;		/*!< The polynomial representation. */
}SIntegrityInfoHdrExtCRC64Gen_t;
/**
* \}
*/

/*******************************************************************************
** Cycle information header extension (header type DAQPROT_HDREXT_TYPE_CYCLE) **
*******************************************************************************/

#define DAQPROT_CYCLE_START			0x01	/*!< The start of a cycle. A new start might automatically indicate a cycle stop	*/
											/*!< before. */
#define DAQPROT_CYCLE_STOP			0x02	/*!< The stop of a cycle. This flag is optionally and automatically assumed when	*/
											/*!< the next cycle start is received. */

/**
* \brief Cycle info structure to inform about state changes of cycles. If this extension is used with data packets, the
* header extension refers to the cycle with the ID as stated in the data packet header allowing multiple cycle to be used
* asynchronously of each other.
*/
typedef struct SCycleInfoHdrExt
{
	SDAQHdr_t		sHdr;				/*!< Base header identifying the type, length and version of the header as well as a header	*/
										/*!< checksum. */
	uint32_t		uiCycleCount;		/*!< The count is incremented with every cycle start. */
	uint8_t			uiState;			/*!< Flags determining the state of the cycle. One of the DAQPROT_CYCLE_ types. */
	uint8_t			rguiReserved[3];	/*!< Reserved for alignment purposes (must be 0). */
} SCycleInfoHdrExt_t;

/***************************************************************************
** Image header extension (header type DAQPROT_HDREXT_TYPE_IMAGE)         **
***************************************************************************/

/**
* \brief Image flags.
* \{
*/
#define DAQPROT_IMAGE_FLAGS_REPRESENTS_ROI	0x00000001	/*!< The image data represents a ROI. ROI information fields are		*/ 
														/*!< filled. */
#define DAQPROT_IMAGE_FLAGS_FLIP_HORIZONTAL 0x00000010	/*!< Image was flipped horizontally. */
#define DAQPROT_IMAGE_FLAGS_FLIP_VERTICAL	0x00000020	/*!< Image was flipped vertically. */
#define DAQPROT_IMAGE_FLAGS_ROTATE_90		0x00000040	/*!< Image was rotated 90 deg. right. */
#define DAQPROT_IMAGE_FLAGS_ROTATE_180		0x00000080	/*!< Image was rotated 180 deg. right. */
#define DAQPROT_IMAGE_FLAGS_ROTATE_270		0x000000C0	/*!< Image was rotated 270 deg. right. */
#define DAQPROT_IMAGE_FLAGS_LEFT_IMAGE		0x00010000	/*!< The image represents the left image of a stereo camera. */
#define DAQPROT_IMAGE_FLAGS_RIGHT_IMAGE		0x00020000	/*!< The image represents the right image of a stereo camera. */
#define DAQPROT_IMAGE_FLAGS_STEREO_IMAGE	0x00030000	/*!< The image represents the stereo image of a stereo camera. */
/**
* \}
*/

/**
* brief Image payload composition order.
* \{
*/
#define DAQPROT_IMAGE_COMP_DEFAULT 0	/*!< Default. The data composition is not relevant for the data interpretation. */
#define DAQPROT_IMAGE_COMP_RAW7    7	/*!< Data is stored in 7-bits per 8 bits. MSB is not used. */
#define DAQPROT_IMAGE_COMP_RAW8    8	/*!< Default. Data is stored in the way it was received by the MDI. */
#define DAQPROT_IMAGE_COMP_RAW10   10	/*!< Data is stored 3 values per 4 bytes. Upper 2 MSB are not used. */
#define DAQPROT_IMAGE_COMP_RAW12   12	/*!< Data is stored 2 values per 3 bytes. */
#define DAQPROT_IMAGE_COMP_RAW14   14	/*!< Data is stored 1 value per 2 bytes. Upper 2 MSB are not used. */
#define DAQPROT_IMAGE_COMP_RAW16   16	/*!< Data is stored 1 value per 2 bytes. Used when values are ordered word wise and are	*/
										/*!< written as little endian. */
#define DAQPROT_IMAGE_COMP_RAW24   24	/*!< Data is stored 1 value per 3 bytes. */
#define DAQPROT_IMAGE_COMP_RAW32   32	/*!< Data is stored 1 value per 4 bytes. */
/**
* \}
*/

/**
* \brief Image information packet header.
* \remarks FourCC code for JPEG images is "JPEG" and for Kinect 3D images is "RGBD".
*/
typedef struct SImageInfoHdrExt
{
	SDAQHdr_t		sHdr;				/*!< Base header identifying the type, length and version of the header as well as a	*/
										/*!< header checksum. */
	uint8_t		    rgucFourCC[4];		/*!< FourCC code of the image type (or one of the types defined above). */
	uint32_t		uiFlags;			/*!< Image flags. */
	uint32_t		uiWidth;			/*!< Width of the (original) image. */
	uint32_t		uiHeight;			/*!< Height of the (original) image. */

	struct
	{
		uint32_t		uiPosX;			/*!< ROI position (before transition). */
		uint32_t		uiPosY;			/*!< ROI position (before transition). */
		uint32_t	    uiWidth;		/*!< ROI width. */
		uint32_t	    uiHeight;		/*!< ROI height. */
	} sROI;

	uint32_t		uiDPIHor;			/*< Horizontal Dots per Inch */
	uint32_t		uiDPIVer;			/*< Vertical Dots per Inch */
	uint32_t		uiFrameCounter;		/*!< Frame counter allowing to match frames and data together. */
	uint32_t		uiComposition;		/*!< Composition of the data. */
} SImageInfoHdrExt_t;

/***************************************************************************
** CSI-2 header extension (header type DAQPROT_HDREXT_TYPE_MDI_CSI2)	  **
***************************************************************************/

/**
* \brief CSI2 header extension.
*/
typedef struct SCSI2HdrExt
{
	SDAQHdr_t		sHdr;				/*!< Base header identifying the type, length and version of the header as well as a	*/
										/*!< header checksum. */
	uint32_t		uiState;			/*!< CSI2 state flags. */
	uint32_t		uiReserved;			/*!< Not used; must be 0. */
	uint64_t		uiBitrate;			/*!< CSI2 bitrate. */
} SCSI2HdrExt_t;

/***************************************************************************
** I�C header extension	(header type DAQPROT_HDREXT_TYPE_I2C)	          **
***************************************************************************/

/**
* \brief I�C header extension.
*/
typedef struct SI2CHdrExt
{
	SDAQHdr_t		sHdr;				/*!< Base header identifying the type, length and version of the header as well as a	*/
										/*!< header checksum. */
	uint32_t		uiBaudrate;			/*!< I�C Baudrate. */
	uint32_t		uiReserved;			/*!< Reserved; must be 0. */
} SI2CHdrExt_t;

/******************************************************************************
** Aurora (XILINX) header extension (header type DAQPROT_HDREXT_TYPE_AURORA) **
******************************************************************************/

#define DAQPROT_AURORA_ERROR_NONE			0x00000000							/*!< No error. */
#define DAQPROT_AURORA_ERROR_CODE			0x00000001							/*!< 8-bit-10-bit Code error. */
#define DAQPROT_AURORA_ERROR_DISPARITY		0x00000002							/*!< 8-bit-10-bit Disparity error. */
#define DAQPROT_AURORA_ERROR_TIMEOUT_ECP	0x00000100							/*!< Timeout ECP (End Frame) missing. */
#define DAQPROT_AURORA_ERROR_TIMEOUT_DATA	0x00000200							/*!< Timeout data missing. */

/**
* \brief Aurora (XILINX) header extension.
*/
typedef struct SAuroraHdrExt
{
	SDAQHdr_t		sHdr;				/*!< Base header identifying the type, length and version of the header as well as a	*/
										/*!< header checksum. */
	uint32_t		uiErrorFlags;		/*!< Error flags. Zeror or more DAQPROT_AURORA_ERROR_ flags. */
	uint32_t		uiReserved;			/*!< Reserved; must be 0. */
} SAuroraHdrExt_t;

/***************************************************************************
** CAN header extension (header type DAQPROT_HDREXT_TYPE_CAN)			  **
***************************************************************************/

/**
* \brief CAN header extension.
*/
typedef struct SCANHdrExt
{
	SDAQHdr_t		sHdr;				/*!< Base header identifying the type, length and version of the header as well as a	*/
										/*!< header checksum. */
	uint32_t		uiBaudrate;			/*!< CAN Baudrate (CAN-LS: 40000..125000, CAN-HS: 125000..1000000). */
	uint32_t		uiMultiplicator;	/*!< CAN-FD multiplicator (CAN-LS/HS: 1, CAN-FD: 1..8). */
} SCANHdrExt_t;

/***************************************************************************
** FlexRay header extension (header type DAQPROT_HDREXT_TYPE_FLEXRAY)	  **
***************************************************************************/

/**
* \brief FlexRay header extension.
*/
typedef struct SFlexRayHdrExt
{
	SDAQHdr_t		sHdr;				/*!< Base header identifying the type, length and version of the header as well as a	*/
										/*!< header checksum. */
	uint32_t		uiBaudrate;			/*!< FlexRay Baudrate (typical: 2500000, 5000000, 10000000). */
	uint32_t		uiReserved;			/*!< Reserved; must be 0. */
} SFlexRayHdrExt_t;

/***************************************************************************
** Frame layer                                                            **
***************************************************************************/

/**
* \brief Frame layer header type
* \{
*/
#define DAQPROT_FRAME_TYPE_FIRST			0x0010	/*!< First header type associated with frames. */
#define DAQPROT_FRAME_TYPE_LAST				0x0FFF	/*!< Last header type associated with frames. */
#define DAQPROT_FRAME_TYPE_GENERIC			0x0010	/*!< Generic frame header; SFameHdr is used. */	
/**
* \}
*/

/**
* \brief Generic frame header.
* \remarks The header is identified with a signature. The signature is "DAQFRAME".
*/
typedef struct SFrameHdr
{
	SDAQHdr_t		sHdr;				/*!< Base header identifying the type, length and version of the header as well as a	*/
										/*!< header checksum. */
	char			szSignature[8];		/*!< Header signature. */
	uint32_t		uiPayloadOffs;		/*!< Start of payload. */
	uint32_t		uiPayloadLen;		/*!< Length of the payload following the frame header. */
	uint64_t		uiStreamID;			/*!< Stream ID identifying the stream that provides the data. */
} SFrameHdr_t;

/***************************************************************************
** Packet layer                                                           **
***************************************************************************/

/**
* \brief Packet layer header type.
* \{
*/
#define DAQPROT_PACKET_TYPE_FIRST					0x4000		/*!< First header type associated with packets. */
#define DAQPROT_PACKET_TYPE_LAST					0x4FFF		/*!< Last header type associated with packets. */
#define DAQPROT_PACKET_TYPE_IMAGE					0x4000		/*!< The packet contains an image (fragment); SImageFrameHdr is	*/
																/*!< used. */
#define DAQPROT_PACKET_TYPE_AUDIO					0x4001		/*!< The packet contains an audio fragment; \todo Not used yet.	*/
#define DAQPROT_PACKET_TYPE_CAN						0x4010		/*!< The packet contains a CAN message; SCptCANMsgHdr is used.	*/
#define DAQPROT_PACKET_TYPE_FLEXRAY					0x4011		/*!< The packet contains a FlexRay frame; SCptFlexRayFrmHdr is	*/
																/*!< used. */
#define DAQPROT_PACKET_TYPE_ETHERNET				0x4012		/*!< The packet contains an Ethernet frame; SCptEthFrmHdr is	*/
																/*!< used. */
#define DAQPROT_PACKET_TYPE_NMEA					0x4020		/*!< The packet contains a NMEA message; SCptNMEAMsgHdr is		*/
																/*!< used. */
/* cwo: missing in 2.1e header!? */
#define DAQPROT_PACKET_TYPE_MDS						 0x4080	/*!< The packet contains a MDS structured	*/
																					/*!< data packet; SCptMDSPacketHdr is used. */
#define DAQPROT_PACKET_TYPE_I2C						0x4021		/*!< The packet contains an I2C message; SCptI2CMsgHdr is used. */
#define DAQPROT_PACKET_TYPE_SPI						0x4030		/*!< The packet contains SPI data. \todo Not used yet. */
#define DAQPROT_PACKET_TYPE_STATE					0x40c0		/*!< The packet contains state information. \todo Not used yet. */
#define DAQPROT_PACKET_TYPE_CSI2_RAW_AGGREGATION	0x4800		/*!< The packet contains CSI-2 raw line collection;				*/
																/*!< SCSI2RawLineCollection is used. */
#define DAQPROT_PACKET_TYPE_JPEGLS					0x4820		/*!< JPEG-LS packet (SJpegLSPcktHdr_t used). */
#define DAQPROT_PACKET_TYPE_JSON_STATUS				0x4850		/*!< The packet contains a JSON status string (UTF-8);			*/
#define DAQPROT_PACKET_TYPE_EVENT					0x9999		/*!< The packet contains a JSON status string (UTF-8);			*/
																/*!< SCptJSONStatusHdr is used. */
#define DAQPROT_PACKET_TYPE_CUSTOMER				0x4C00		/*!< The packet contains a customer specific data.				*/
																/*!< SCustomPcktHdr	is used. */
/**
* \}
*/

/*********************************************************************************
** Generic packet header (part of every packet); packet hdr type 0x4000..0x40ff **
**********************************************************************************/

/**
* \brief Generic packet header.
* \details The generic packet header
*/
typedef struct SDataPacket
{
	SDAQHdr_t		sHdr;				/*!< Base header identifying the type, length and version of the header as well as a	*/
										/*!< header checksum. */
	uint16_t		uiPayloadOffs;		/*!< Start of payload. */
	uint8_t			uiCycleID;			/*!< The ID of the cycle this packet belongs to (or zero when no cycle is defined). */
	uint8_t			uiReserved;
	uint32_t		uiPayloadLen;		/*!< Length of the payload following the frame header. */
} SDataPacket_t;

/*********************************************************************************
** Generic packet header (part of every packet); packet hdr type 0x4100..0x41ff **
**********************************************************************************/

/**
* \{
* \brief Flags for extensions (part of the SCompactDataPacket mask).
* \description The SCompactDataPacket is the base header for compact data packets of any type. The base packet can be extended by 
* specialized packet header information. These extensions are activated by setting the flags in the mask member of the packet. The
* extension are part of the header when the bit in the mask is activate starting with MSB first.
*/
#define DAQPROT_COMPACT_PACKET_EXT_PADDING				0x01		/*!< Header and or payload is extended with padding. The    */
																	/*!< padding length is specified in the padding extension.  */
																	/*!< The padding is not included in the payload length. */
#define DAQPROT_COMPACT_PACKET_EXT_CUST_HDR				0x02		/*!< Customer header is preceeding the payload. Length is   */
																	/*!< specified in customer header extension. The customer */
																	/*!< header is included in the payload length. */
#define DAQPROT_COMPACT_PACKET_EXT_INTEGRITY_HDR		0x04		/*!< Header integrity is available for this packet; type	*/
																	/*!< (and size) was defined in the frame header.  */
#define DAQPROT_COMPACT_PACKET_EXT_INTEGRITY_PAYLOAD	0x08		/*!< Payload (and padding) integrity is available for this	*/
																	/*!< packet; type (and size) was defined in the frame		*/
																	/*!< header. */
#define DAQPROT_COMPACT_PACKET_EXT_TIMESTAMP			0x10		/*!< Timestamp follows the header; type (and size) was		*/
																	/*!< defined in the frame header. */
/**
* \}
*/

/**
* \brief Generic packet header for compact frames (marked by DAQPROT_HDR_FLAG_COMPACT_HDR flag).
* \attention The compact header is identified by containing the DAQPROT_HDR_FLAG_COMPACT_HDR flag. Since the compact packet 
* header misses the information that is available in the standard packet header, this information needs to be supplied by a
* control-packet (see SCompactCtrlPacket in control packets section).
* \details The generic packet header
*/
typedef struct SCptDataPacket
{
	uint8_t		uiMaskVersionMajor;		/*!< MSB bit 7 is endianess (big = 1; little = 0); bit 6 is compact header				*/
										/*!< (compact = 1) and 4 bits are major version (currently version 2). The Mask must    */
										/*!< contain at least the DAQPROT_HDR_FLAG_COMPACT_HDR flag. */
	uint8_t		uiVersionMinor;			/*!< Minor version. */
	uint8_t		uiExtMask;				/*!< Bitmask with extension flags. One or more of the DAQPROT_COMPACT_PACKET_EXT_       */
										/*!< flags. */
	uint8_t		uiCycleID;				/*!< Cycle ID this packet belongs to. */
	union 
	{
		struct 
		{
			uint32_t uiPayloadLen : 24;
			uint32_t uiPayloadOffs: 8;
		};
		uint32_t	uiPayloadLenOffs;
	};

	
	
	//uint32_t	uiPayloadLenOffs;		/*!< Payload length (LSB 0..23 bits) including padding and offset (MSB 24..31) to begin	*/
										/*!< of payload/padding. */
} SCptDataPacket_t;

/**
* \brief Compact data packet extension when the mask of the data packet contains DAQPROT_COMPACT_PACKET_FLAG_TIMESTAMP and
* the default timestamp flags have the flag DAQPROT_COMPACT_PACKET_FLAG_TIMESTAMP_32BITS set.
*/
typedef struct SCptExtTSAbs32
{
	uint32_t		uiTimestamp;
} SCptExtTSAbs32_t;

typedef struct SCptExtTSRel32
{
	int32_t			iTimestamp;
} SCptExtTSRel32_t;

typedef struct SCptExtTSAbs64
{
	uint64_t		uiTimestamp;
} SCptExtTSAbs64_t;

typedef struct SCptExtTSRel64
{
	int64_t			iTimestamp;
} SCptExtTSRel64_t;

typedef struct SCptExtIntegrity8
{
	uint8_t			uiVal;
} SCptExtIntegrity8_t;

typedef struct SCptExtIntegrity16
{
	uint16_t		uiVal;
} SCptExtIntegrity16_t;

typedef struct SCptExtIntegrity32
{
	uint32_t		uiVal;
} SCptExtIntegrity32_t;

typedef struct SCptExtCustHdr
{
	uint8_t			uiHdrLen;	/*!< The length of the header in bytes. The header is considered to belong to the payload. */
} SCptExtCustHdr_t;

typedef struct SCptExtPadding
{
	uint8_t			uiPadding;	/*!< LSB nibble (bits 0..3) defines payload padding. MSB nibble (bits 4..7) defines header      */
								/*!< padding. */
} SCptExtPadding_t;

/***************************************************************************
** Image packet header (header type DAQPROT_PACKET_TYPE_IMAGE)            **
***************************************************************************/

/**
* \brief Image information packet header.
*/
typedef struct SImageFrmHdr
{
	SDataPacket_t	sPcktHdr;			/*!< Base header identifying the type, length and version of the header as well as a	*/
										/*!< header checksum and the payload offset and length. */
} SImageFrmHdr_t;

/***************************************************************************
** CAN/CAN-FD packet header (header type DAQPROT_PACKET_TYPE_CAN)         **
***************************************************************************/

/**
* \{
* \brief CAN information packet flags.
*/
#define DAQPROT_CAN_FLAG_TYPE_MESSAGE			0x0000		/*!< CAN/CAN-FD message containing payload */
#define DAQPROT_CAN_FLAG_TYPE_ERROR_FRAME		0x8000		/*!< CAN/CAN-FD error frame */

/* CAN message flags */
#define DAQPROT_CAN_FLAG_EXTENDED_IDENTIFIER	0x0080		/*!< CAN message uses extended identifier (otherwise standard		*/
															/*!< identifier). */
#define DAQPROT_CAN_FLAG_DIRECTION_RECEIVED		0x0010		/*!< CAN message was received (otherwise acknowledge was			*/
															/*!< transmitted). */
#define DAQPROT_CAN_FLAG_REMOTE_FRAME			0x1000		/*!< CAN message is a remote message. */
#define DAQPROT_CAN_FLAG_CANFD_FORMAT			0x0020		/*!< Set when the message uses the CAN-FD format. */
#define DAQPROT_CAN_FLAG_CANFD_BITRATE_SWITCH	0x0040		/*!< Set when the message uses the CAN-FD bitrate switching. */
/**
* \}
*/

/**
* \{
* \brief CAN error frame type.
*/
#define DAQPROT_CAN_ERROR_NONE				0x0000		/*!< No error. */
#define DAQPROT_CAN_ERROR_BIT				0x0001		/*!< If the bit level actually read differs from the one transmitted, a	*/
														/*!< Bit Error is signaled. (No bit error is raised during the			*/
														/*!< arbitration process.). */
#define DAQPROT_CAN_ERROR_STUFF				0x0002		/*!< If more than five consecutive bits of the same level occurs on the	*/
														/*!< bus, a Stuff Error is signaled. */
#define DAQPROT_CAN_ERROR_FORM				0x0003		/*!< If a CAN controller detects an invalid value in one of the fixed	*/
														/*!< fields, a Form Error is signaled. */
#define DAQPROT_CAN_ERROR_ACKNOWLEDGE		0x0004		/*!< If the transmitter can't detect a dominant level in the ACK slot,	*/
														/*!< an Acknowledge Error is signaled. */
#define DAQPROT_CAN_ERROR_CRC				0x0005		/*!< Any node that detects a different CRC in the message than what it	*/
														/*!< has calculated itself will signal an CRC Error. */
#define DAQPROT_CAN_ERROR_GENERAL           0x0006      /*!< General CAN error that could not be narrowed down                  */
#define DAQPROT_CAN_ERROR_BUFFER_OVERFLOW	0x1000		/*!< The Rx buffer was full and new messages could not be added. */
/**
* \}
*/

/**
* \brief CAN-LS, CAN-HS and CAN-FD message packet header.
* \remarks The payload of a CAN frame typically is 8 bytes; of CAN-FD can be up to 64 bytes.
*/
typedef struct SCptCANMsgHdr
{
	SCptDataPacket_t	sPcktHdr;	/*!< Base compact header. */
	uint16_t			uiFlags;	/*!< CAN flags. One or more flags of DAQPROT_CAN_FLAG_ flags. */
	uint32_t			uiID;		/*!< CAN identifier when uiFlags doesn't have DAQPROT_CAN_TYPE_ERROR_FRAME set or error		*/
									/*!< when uiFlags has DAQPROT_CAN_TYPE_ERROR_FRAME set. */
} SCptCANMsgHdr_t;

/***************************************************************************
** FlexRay packet header (header type DAQPROT_PACKET_TYPE_FLEXRAY)        **
***************************************************************************/

/**
* \{
* \brief FlexRay information packet flags.
*/
#define DAQPROT_FLEXRAY_FLAG_TYPE_FRAME			0x0000		/*!< FlexRay frame containing payload (incl NMVector and ID) */
#define DAQPROT_FLEXRAY_FLAG_TYPE_ERROR			0x8000		/*!< FlexRay error */

/* FlexRay channel flags */
#define DAQPROT_FLEXRAY_FLAG_CHANNEL_NONE		0x0000		/*!< No channel was used. */
#define DAQPROT_FLEXRAY_FLAG_CHANNEL_A			0x0010		/*!< Data was sent on channel A. */
#define DAQPROT_FLEXRAY_FLAG_CHANNEL_B			0x0020		/*!< Data was sent on channel B. */
#define DAQPROT_FLEXRAY_FLAG_CHANNEL_BOTH		0x0030		/*!< Data was sent both on channel A and channel B. */
/**
* \}
*/

/**
* \{
* \brief FlexRay error frame type.
*/
#define DAQPROT_FLEXRAY_ERROR_NONE							0x0000	/*!< No error. */
#define DAQPROT_FLEXRAY_ERROR_IFC_OVERFLOW					0x0001	/*!< Interface overflow. */
#define DAQPROT_FLEXRAY_ERROR_SYNC_BLOW_MIN					0x0002	/*!< Sync frame blow minimum. */
#define DAQPROT_FLEXRAY_ERROR_SYNC_FRAME_OVERFLOW			0x0003	/*!< Sync frame overflow. */
#define DAQPROT_FLEXRAY_ERROR_CLOCK_CORRECTION				0x0004	/*!< Clock correction failure. */
#define DAQPROT_FLEXRAY_ERROR_SYNTAX_ERROR					0x0005	/*!< Syntax error received; frame is syntactically			*/
																	/*!< incorrect. */
#define DAQPROT_FLEXRAY_ERROR_CONTENT_ERROR					0x0006	/*!< Content error received; frame is symantically			*/
																	/*!< incorrect. */
#define DAQPROT_FLEXRAY_ERROR_SLOT_BOUNDARY_VIOLATION		0x0007	/*!< Slot boundary violation observed. */
#define DAQPROT_FLEXRAY_ERROR_TRANSMISSION_ACROSS_BOUNDARY	0x0008	/*!< Transmission across boundary. */
#define DAQPROT_FLEXRAY_ERROR_LATEST_TRANSMIT				0x0009	/*!< Latest transmit. */
#define DAQPROT_FLEXRAY_ERROR_SW_SYNTAX_ERROR				0x000a	/*!< Symbol window syntax error. */
#define DAQPROT_FLEXRAY_ERROR_SW_SLOT_BOUNDARY_VIOLATION	0x000b	/*!< Symbol window slot boundary violation. */
#define DAQPROT_FLEXRAY_ERROR_SW_TRANSMISSION_CONFLICT		0x000c	/*!< Symbol window transmission conflict. */
#define DAQPROT_FLEXRAY_ERROR_NIT_SYNTAX_ERROR				0x000d	/*!< Network idle time syntax error. */
#define DAQPROT_FLEXRAY_ERROR_NIT_SLOT_BOUNDARY_VIOLATION	0x000e	/*!< Network idle time slot boundary violation. */
/**
* \}
*/

/**
* \brief Flexray information packet header (format of the paylaod is defined by the ISO17458 standard).
* \remarks The FlexRay payload consists of: 5 bytes header, up to 256 bytes data and 3 bytes trailer.
* \remarks The FlexRay header and trailer (as part of the payload) are big endian.
* \remarks Dependable on the payload preamble indicator, the payload might contain the NMVector as well as the message ID.
*/
typedef struct SCptFlexRayFrmHdr
{
	SCptDataPacket_t	sPcktHdr;			/*!< Base compact header. */
	uint16_t			uiFlags;			/*!< FlexRay flags. One or more flags of DAQPROT_FLAGS_FLAG_ flags. */
	uint16_t			uiError;			/*!< FlexRay error (valid when uiFlags has DAQPROT_FLEXRAY_TYPE_ERROR set). */
											/*!< Otherwise 0. */
} SCptFlexRayFrm_t;

/******************************************************************************************
** Ethernet frame header (header type DAQPROT_PACKET_TYPE_ETHERNET)						 **
******************************************************************************************/

/**
* \brief Ethernet frame header followed by an Ethernet II frame (as specified by IEEE802.3).
* \remarks See Wikipedia: https://en.wikipedia.org/wiki/Ethernet_frame
* \remarks The Ethernet frame as specified by IEEE802.3 starts with the MAC-address. The Preamble and SFD are not part of the
* frame.
* \remarks The Ethernet frame is specified in big-endian.
*/
typedef struct SCptEthFrmHdr
{
	SCptDataPacket_t	sPcktHdr;			/*!< Base compact header. */
} SCptEthFrmHdr_t;

/******************************************************************************************
** NMEA message header (header type DAQPROT_PACKET_TYPE_NMEA)							 **
******************************************************************************************/

/**
* \brief NMEA message header followed by a NMEA message string (including carriage return and line-feed).
* \remarks See Wikipedia: https://en.wikipedia.org/wiki/NMEA_0183
*/
typedef struct SCptNMEAMsgHdr
{
	SCptDataPacket_t	sPcktHdr;			/*!< Base compact header. */
} SCptNMEAMsgHdr_t;

/******************************************************************************************
** I2C message header (header type DAQPROT_PACKET_TYPE_I2C)								 **
******************************************************************************************/

#define DAQPROT_I2C_FLAG_WRITE				0x0000	/*!< Write flag. */
#define DAQPROT_I2C_FLAG_READ					0x0001	/*!< Read flag. */
#define DAQPROT_I2C_FLAG_ACK					0x0000	/*!< Acknowledge flag. */
#define DAQPROT_I2C_FLAG_NACK					0x0010	/*!< No acknowledge flag. */
#define DAQPROT_I2C_FLAG_START_STOP		0x0000	/*!< Start and stop received. */
#define DAQPROT_I2C_FLAG_START				0x0020	/*!< Start only. */
#define DAQPROT_I2C_FLAG_7BIT_ADDR		0x0000	/*!< 7-bit address. */
#define DAQPROT_I2C_FLAG_10BIT_ADDR		0x1000	/*!< 10-bit address. */


#define DAQPROT_LL_I2C_FLAG_START_CONDITION 		0x0001
#define DAQPROT_LL_I2C_FLAG_STOP_CONDITION  		0x0002
#define DAQPROT_LL_I2C_FLAG_RESTART_CONDITION   0x0003

#define DAQPROT_LL_I2C_FLAG_ACKNOWLEDGED  			0x0004
#define DAQPROT_LL_I2C_FLAG_NOT_ACKNOWLEDGED		0x0005

#define DAQPROT_LL_I2C_FLAG_PAYLOAD							0x0006

/**
* \brief I2C message header followed by I2C payload.
* \remarks See Wikipedia: https://en.wikipedia.org/wiki/I%C2%B2C
*/
typedef struct SCptI2CMsgHdr
{
	SCptDataPacket_t	sPcktHdr;			/*!< Base compact header. */
	uint16_t			uiFlags;					/*!< Flags defining the message context. */
	uint16_t			uiAddress;			/*!< 8 or 10-bit address */
} SCptI2CMsgHdr_t;

typedef struct SCptI2CLLMsgHdr
{
	SCptDataPacket_t	sPcktHdr;			/*!< Base compact header. */
	uint16_t			uiFlags;					/*!< Flags defining the message context. */
} SCptI2CLLMsgHdr_t;

/******************************************************************************************
** CSI2 Raw line packet header (header type DAQPROT_PACKET_TYPE_CSI2_RAW_AGGREGATION)    **
******************************************************************************************/

/**
* \brief Unified header for CSI2 short and long packet headers. See CSI2 specification 1.1 
* chapter 9.1 (Low Level Protocol Packet Overview) for more information.
*/
typedef union SCSI2PacketHdr
{
	struct SCSI2ShortHdr
	{
		uint32_t uiDataType : 6;			/*!< Data type between 0x00 and 0x0f for short packet. */
		uint32_t uiVirtualChannel : 2;
		uint32_t uiData : 16;
		uint32_t uiECC : 8;
	} sShortHeader;

	struct SCSILongHdr
	{
		uint32_t uiDataType : 6;			/*!< Data type 0x10 or larger for long packet. */
		uint32_t uiVirtualChannel : 2;
		uint32_t uiWordCount : 16;
		uint32_t uiECC : 8;
	} sLongHeader;
} SCSI2PacketHdr_t;

/**
* \brief Header preceeding every CSI2 package (short and long packet).
*/
typedef struct SCSI2FpgaPreLineHdr
{
	uint64_t uiTimestamp;				/*!< Synchronized/unsynchronized hardware timestamp. */
	uint32_t uiFrameCounter : 8;		/*!< Counter incrementing with every frame start packet. */
	uint32_t uiLineCounter : 24;		/*!< Counter incremented with every line. */
} SCSI2FpgaPreLineHdr_t;

/**
* \brief Header for every CSI2 line, followed by the line payload and the footer (SCSI2RawLineFooter_t).
*/
typedef struct SCSI2RawLineHeader
{
	SCSI2FpgaPreLineHdr_t 	sFPGAHeader;	/*!< FPGA generated header associated with every CSI2 packet */
	SCSI2PacketHdr_t 		sCSI2Header;	/*!< CSI2 long/short header preceeding data lines */
} SCSI2RawLineHeader_t;

/**
* \brief Footer for every CSI2 line, directly after Header (SCSI2RawLineHeader_t) and line payload, only for long packets
*/
typedef struct SCSI2RawLineFooter
{
	uint16_t		uiCSI2CRC;		/*!< CSI2 specification 1.1 chapter 9.6 (Checksum Generation). */
	uint16_t		uiFpgaFlags;	/*!< FPGA generated footer - reserved for future use. */
} SCSI2RawLineFooter_t;

/**
* \brief Header with a collection of raw CSI2 lines.
* \remarks The header follows immediatly the "LineCount" specified number of raw CSI2 lines (SCSI2RawLineHeader)
* every array item is build the following way:
* <SCSI2RawLineHeader_t>[<line_payload[n]><SCSI2RawLineFooter_t>]
* -> short packages have no line_payload and footer for obvious reasons
* -> long packages always have the footer and probably line_payload
*/
typedef struct SCSI2RawLineCollection
{
	SDataPacket_t	sPcktHdr;			/*!< Base header identifying the type, length and version of the header as well as a */
										/*!< header checksum and the payload offset and length. */
	uint32_t		uiLineCount;		/*!< Number of CSI2 packages in this line */
	uint32_t		uiFlags;			/*!< Reserved flags for future use */
} SCSI2RawLineCollection_t;

/***************************************************************************
** JPEG-LS header extension (header type DAQPROT_PACKET_TYPE_JPEGLS)      **
***************************************************************************/

/**
* \brief Specific packet header containing JPEG-LS compression information.
*/
typedef struct SJpegLSPcktHdr
{
	SDataPacket_t	sPcktHdr;					/*!< Base header identifying the type, length and version of the header as well	*/
												/*!< as a header checksum and the payload offset and length. */
	uint8_t			uiSamplePrecision;			/*!< This argument defines the input image samples precision in bits. */
	uint8_t			uiColorComponentCount;		/*!< This argument defines the number of color components. */
	uint8_t			uiPointTransform;			/*!< This argument defines the point transform value used for JPEGLS encoding. */
	uint8_t			uiThreshold1;				/*!< This argument defines the T1 quantization threshold value used for JPEGLS	*/
												/*!< encoding. */
	uint32_t		uiThreshold2;				/*!< This argument defines the T2 quantization threshold value used for JPEGLS	*/
												/*!< encoding. */
	uint32_t		uiThreshold3;				/*!< This argument defines the T3 quantization threshold value used for JPEGLS	*/
												/*!< encoding. */
	uint32_t		uiFlags;			        /*!< Reserved; used for alignment purposes. */
} SJpegLSPcktHdr_t;

/********************************************************************************************
** JSON status packet header (header type DAQPROT_PACKET_TYPE_JSON_STATUS)                 **
********************************************************************************************/

/**
* \brief Generic header containing status information in JSON format.
* \remarks The content should only be valid JSON strings, but without structure specification. Both, fast and styled strings 
* are valid.
*/
typedef struct SJSONStatusString
{
	SDataPacket_t	sPcktHdr;			/*!< Base header identifying the type, length and version of the header as well as a */
										/*!< header checksum and the payload offset and length. */
} SJSONStatusString_t;


typedef struct SSpontanEvent
{
	SDataPacket_t	sPcktHdr;			/*!< Base header identifying the type, length and version of the header as well as a */
										/*!< header checksum and the payload offset and length. */
} SSpontanEvent_t;
/******************************************************************************************
** Customer packet header (header type DAQPROT_PACKET_TYPE_CUSTOMER)					 **
******************************************************************************************/

/**
* \brief Customer data packet containing customer specific data.
*/
typedef struct SCustomPcktHdr
{
	SDataPacket_t	sPcktHdr;			/*!< Base header identifying the type, length and version of the header as well as a */
										/*!< header checksum and the payload offset and length. */
	uint32_t		uiCustomerID;		/*!< ID used to identify the customer. */
	uint32_t		uiFlags;			/*!< Flags; free to be defined by the customer. */
} SCustomPcktHdr_t;

/***************************************************************************
** Control packets														  **
***************************************************************************/

/**
* \{
* \brief Flags for control packets (part of the SInfoCtrlDataPacket mask).
* \description The SInfoCtrlDataPacket is the base header for compact data control packets of any type. The base control packet
* can be extended by specialized packet header information. These control extensions are activated by setting the flags in the 
* control mask member of the packet. The extension are part of the header when the bit in the mask is activate starting with MSB 
* first. They are located before any generic packet extensions (identified by the DAQPROT_COMPACT_PACKET_FLAG_ flags).
*/
#define DAQPROT_CTRL_PACKET_TYPE_INFO		0x0001	/*!< Generic packet info; used to provide generic packet information; valid	*/
													/*!< for all packets when not specified otherwise (needed for compact		*/
													/*!< packets). */
#define DAQPROT_CTRL_PACKET_TYPE_SYNC		0x0002	/*!< Time synchronization packet; identifies the time at the start of the	*/
													/*!< frame. This packet is needed when using absolute timestamps for		*/
													/*!< packets; to synchronize the timebases. */
#define DAQPROT_CTRL_PACKET_TYPE_CYCLE		0x0010	/*!< Cycle start packet; identifies the start of a cycle. */
#define DAQPROT_CTRL_PACKET_TYPE_COUNT		0x0020	/*!< Packet count; informing about the amount of packets that follow. */
/*
* \}
*/

/**
* \brief Compact info control packet. Used to control the data flow.
* \remarks The control packet is extended dependable on the type flags (one or more flags of DAQPROT_CTRL_PACKET_TYPE_).
* \attention This packet is header only and doesn't have any payload.
*/
typedef struct SCptCtrlPacket
{
	SCptDataPacket_t	sPcktHdr;				/*!< Compact header. */
	uint16_t			uiCtrlTypeMask;			/*!< Type of control packet. For information packet type, must be */
												/*!< DAQPROT_CTRL_PACKET_TYPE_INFO. */
} SCptCtrlPacket_t;

/**
* \brief Default information of all compact data packets that follow. The use of at least one control packet with this
* information is compulsory and necessary to be able to interpret the packets that follow.
* \atention This information packet doesn't have calculation information for the default integrity of header and data. Use
* specialized headers instead.
*/
typedef struct SInfoCtrl
{
	uint16_t	uiDefPacketType;		/*!< Default packet type. One of the DAQPROT_PACKET_TYPE_ types. */
	uint16_t	uiDefTimebaseType;		/*!< Default timebase type. One of the DAQPROT_TIMEBASE_ types (relative types are		*/
										/*!< allowed). */
	uint16_t	uiDefTimebaseFlags;		/*!< Default timestamp flags indicating the quality of the timestamps. */
	uint16_t	uiDefHdrIntegrityType;	/*!< Default type of integrity for the header (DAQPROT_HASH_TYPE_). For generic CRC		*/
										/*!< calculation, the CRC polynomial follows this structure. */
	uint16_t  HeaderIntegrityFlags;
	uint16_t	uiDefDataIntegrityType;	/*!< Default type of integrity for the payload (DAQPROT_HASH_TYPE_). For generic CRC		*/
	uint16_t  DataIntegretyFlags;
										/*!< calculation, the CRC polynomial follows this structure. */
	/* For generic CRC integrity for the header, the polynomial is defined here (uintx_t type). */
	/* For generic CRC integrity for the payload, the polynomial is defined here (uintx_t type). */
} SInfoCtrl_t;

/**
* \brief Sync control packet. Used to inform about the value of the timebase at the beginning of the frame (when not 
* supplying an additional relative time through the compact packet extension).
*/
typedef struct SSyncCtrl
{
	uint16_t		uiTimebaseType;			/*!< Timebase type. One of the DAQPROT_TIMEBASE_ types. */
	uint16_t		uiTimebaseFlags;		/*!< Timestamp flags indicating the quality of the timestamp. */
	uint64_t		uiTimestamp;			/*!< Timestamp of the beginning of the cycle. Could be 32 or 64-bit dependable on */
											/*!< the timebase type and resolution. */
} SSyncCtrl_t;

/**
* \brief Cycle control packet. Used to inform about the start of a cycle.
* \remarks The cycle ID is contained in the data packet header.
*/
typedef struct SCycleCtrl
{
	uint8_t			uiCycleCount;		/*!< The count is incremented with every cycle start. */
	uint8_t			uiState;			/*!< Flags determining the state of the cycle. One of the DAQPROT_CYCLE_ types. */
} SCycleCtrl_t;

/**
* \brief Packet count control packet, containing the amount of compact packets following this control packet until the next
* packet count control packet is introduced.
*/
typedef struct SPacketCountCtrl
{
	uint16_t		uiCount;			/*!< The amount of compact packets that follow this control packet. */
} SPacketCountCtrl_t;


#if  __GNUC__
# pragma pack(pop)
#elif _MSC_VER && !__INTEL_COMPILER
# pragma pack(pop)
#else 
# error "Your compiler is not integrated. Please add the correct pack pragmas!"
#endif


#ifdef __cplusplus
		}	// extern "C"
	}	// namespace DAQProt_v201
}	// namespace AVETO
#endif /* __cplusplus */

#endif /* AVETO_HEADER_DEFINITION_V201 */
