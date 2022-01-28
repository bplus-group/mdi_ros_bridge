#ifndef __BPL_MEAS_BACKWARDS_COMPATIBILITY_HEADER__
#define __BPL_MEAS_BACKWARDS_COMPATIBILITY_HEADER__

typedef struct 
{
  MdiRx_StructInfo_t const info; /**< standard interface info */
  uint32_t (MDIRXAPI_DECL *GetApiVersion)( void );
  WaitHandle_t (MDIRXAPI_DECL *GetDataEventHandle)(void);
  uint32_t (MDIRXAPI_DECL *Init)(char const* szIP);
  uint32_t (MDIRXAPI_DECL *InitEx)(char const* szIP, uint16_t port, uint8_t cnt);
  uint32_t (MDIRXAPI_DECL *Start)(void);
  uint32_t (MDIRXAPI_DECL *Stop)(void);
  uint32_t (MDIRXAPI_DECL *Deinit)(void);
  uint32_t (MDIRXAPI_DECL *GetData)(BplMeasFrameInfo_t FrameArray[], uint32_t* ArraySize);
  uint32_t (MDIRXAPI_DECL *FreeData)(BplMeasFrameInfo_t* frame);
  uint32_t (MDIRXAPI_DECL *GetStatistics)(BplMeasMdiReceptionStatistics_t* statistics);
  uint32_t (MDIRXAPI_DECL *RegisterMemManager)(mm_malloc_t pfct_malloc, mm_realloc_t pfct_realloc, mm_free_t pfct_free, void* tag);
  bool (MDIRXAPI_DECL     *IsCrc32cHwSupported)();
  uint32_t (MDIRXAPI_DECL *CalcCrc32c)(void const* pData, uint32_t size);
  uint32_t (MDIRXAPI_DECL *SetTimeoutMS)(uint32_t timeout);
  uint32_t (MDIRXAPI_DECL *DeliverCorruptFrames)(bool bEnable);
  bool (MDIRXAPI_DECL     *JoinMulticastGroup)(char const* szMcastGroup);
  bool (MDIRXAPI_DECL     *JoinMulticastGroupEx)(char const* szMcastGroup, char const* szIfaceIp);
} MdiRx_Reception_interface_v1_t;



#endif