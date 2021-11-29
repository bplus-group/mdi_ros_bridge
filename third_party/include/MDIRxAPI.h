/** @file *************************************************************************************************************

@verbatim
***********************************************************************************************************************
*
*  Copyright (c) 2021, b-plus technologies GmbH. All rights reserved!
*
*    All rights exclusively reserved for b-plus technologies GmbH, unless expressly otherwise agreed.
*
*    Redistribution in source or any other form, with or without modification, is not permitted.
*
*    You may use this code under the according license terms of b-plus. Please contact b-plus at services@b-plus.com
*    to get the appropriate terms and conditions.
*
***********************************************************************************************************************
@endverbatim

Filename    mdirxapi.h

@brief      Export definition of MDI data reception API.

@author     cwolf

@version    1.7.0

**********************************************************************************************************************/
#ifndef __BPLUS_MEAS_SPECIALIZED_MDIRX_EXPORT_HEADER__
#define __BPLUS_MEAS_SPECIALIZED_MDIRX_EXPORT_HEADER__
#include <stdint.h>

#ifdef WIN32
  #include <winsock2.h>
  #include <windows.h>
  typedef HANDLE WaitHandle_t;
  

  #if defined(_LP64) || defined(__LP64__) || defined(__amd64__) || defined(__amd64) || defined(__x86_64__) || defined(_M_AMD64) || defined(_WIN64) || defined(__aarch64__) || defined(__ia64__) || defined(_IA64) || defined(__IA64__) || defined(_M_IA64)
    #define MDIRXAPI_DECL __fastcall
  #elif defined(_ILP32) || defined(__ILP32__) || defined(_WIN32) || defined(i386) || defined(__i386) || defined(__i386__) || defined(_M_IX86) || defined(__X86__) || defined(_X86_) || defined(__arm__) || defined(_M_ARM) || defined(_M_ARMT) || defined(__arm)
    #define MDIRXAPI_DECL __stdcall
  #else
    #warning "undefined bitness"
  #endif
  
  #ifdef MDIRXAPI_EXPORTS
    #define MDIRXAPI_API __declspec(dllexport) 
    #define MDIRXAPI_DEPRECATED_API __declspec(dllexport)
  #else
    #define MDIRXAPI_API __declspec(dllimport) 
    #define MDIRXAPI_DEPRECATED_API __declspec(dllimport)
  #endif
#else
  #define MDIRXAPI_DECL
  #define MDIRXAPI_API
  #define MDIRXAPI_DEPRECATED_API
  typedef int WaitHandle_t;
  #define NOERROR (0)
  #define E_UNEXPECTED (0x8000FFFF)
  #define E_INVALIDARG (0x80070057)
  #define E_FAIL       (0x80004005)
  #define INVALID_HANDLE_VALUE (-1)
#endif

#pragma pack(4)


#define BPLMEAS_FRAME_INFO_CORRUPT        0x80000000  /* frame is corrupt */
#define BPLMEAS_FRAME_INFO_AVAILABLE      0x40000000  /* extensive info about corruption is available */
#define BPLMEAS_CHUNK_INFO_AVAILABLE      0x20000000  /* chunk flag map is available */
#define BPLMEAS_FRAME_WAS_RECLAIMED       0x10000000  /* parts of the frame have been relaimed in a second reception, this indicated issues with the reception */

typedef struct BplMeasValidFramePartsInfo_t
{
  void* pStart;
  uint32_t Length;
} BplMeasValidFramePartsInfo_t;

typedef struct BplMeasValidFrameInfo_t
{
  uint32_t NoOfEntries;
  uint32_t ExpectedSize;
  BplMeasValidFramePartsInfo_t Parts[1]; /*! anysize array, the correct number is NoOfEntries */
} BplMeasValidFrameInfo_t;

typedef struct BplMeasChunkInfo_t
{
  uint32_t cnt;
  uint32_t bits;
  uint64_t chunks[1];
} BplMeasChunkInfo;

typedef struct BplMeasFrameInfo_t
{
  void* pFrame;                         /*! pointer to the frame content, the pointer is to be cleared by application! */
  void* pInstanceTag;                   /*! instance tag supplied during each call to a memory callback */
  uint32_t Size;                        /*! size of the frame */
  uint32_t SrcIp;                       /*! ip of the sender of the frame */
  uint32_t Flags;                       /*! Flags for the frame */
  BplMeasValidFrameInfo_t* pFrameInfo;  /*! variable sized list of valid entries of the frame, if corruption is detected */
  BplMeasChunkInfo_t*      pChunkInfo;  /*! variable sized flag list of valid chunks */
} BplMeasFrameInfo_t;

typedef struct BplMeasMdiReceptionStatistics_t
{
  uint64_t CurrentFrameCache;       /*! Amount of memory currently used for caching reception data */
  uint64_t CompletedFrames;         /*! Number of successfully received frames */
  uint64_t CorruptFrames;           /*! Number of failed frames due to reception of duplicated fragment data */
  uint64_t TimeoutFrames;           /*! Number of frames not received in time due to loss of at least one fragment */
  uint64_t DiscardedFrames;         /*! Number of frames needed to be discarded due to not being received by application in time */
  
  uint32_t FramesReady;             /*! Current number of completed frames ready for reception by application */
  uint32_t PendingFrames;           /*! Number of frames currently be received */

  uint32_t NewCompletedFrames;      /*! Number of completed frames since last status request */
  uint32_t NewCorruptFrames;        /*! Number of corrupt frames since last status request */
  uint32_t NewTimeoutFrames;        /*! Number of timed out frames since last status request */
  uint32_t NewDiscardedFrames;      /*! Number of discarded frames since last status request */
} BplMeasMdiReceptionStatistics_t;



#pragma pack()


typedef void*(*mm_malloc_t)(size_t size_to_alloc, void* pMemMgr, void** pInstanceTag);         /*! @brief          function pointer for malloc(size_to_alloc, tag) / new memory allocation
                                                                                                  @params[in]     size_to_alloc - is the number of bytes requested
                                                                                                  @params[in]     pMemMgr - the value initial passed to BplMeas_RegisterMemManager 
                                                                                                  @params[in]     pInstanceTag - custom transparent value supplied on reception 
                                                                                                  @retval         NULL if the memory cannot be allocated, else the valid pointer.
                                                                                              */
typedef void*(*mm_realloc_t)(void* old_pointer, size_t new_size, void* pMemMgr, void** pInstanceTag);  /*! @brief          function pointer for realloc(old_pointer, new_size, tag) / resize allocation
                                                                                                          @params[in]     old_pointer - the previously acquired pointer
                                                                                                          @params[in]     new_size - the new size of the memory, can be smaller or larger!
                                                                                                          @params[in]     pMemMgr - the value initial passed to BplMeas_RegisterMemManager
                                                                                                          @params[in]     pInstanceTag - custom transparent value supplied on reception
                                                                                                          @retval         NULL if the reallocation fails (and the passed pointer has to be
                                                                                                                          invalidated by this function), else the new valid pointer (the old one
                                                                                                                          is invalidated by this function)
                                                                                                      */

typedef void(*mm_free_t)(void* pointer_to_free, void* pMemMgr, void** pInstanceTag);             /*! @brief          function pointer for free(pointer_to_free, tag) / free allocation 
                                                                                                    @params[in]     pointer_to_free - the previously acquired pointer which is to be freed and
                                                                                                                    invalidated by this function.
                                                                                                    @params[in]     pMemMgr - the value initial passed to BplMeas_RegisterMemManager
                                                                                                    @params[in]     pInstanceTag - custom transparent value supplied on reception
                                                                                                */

#define BPLMEAS_DEFAULT_RX_BASE_PORT (3092)
#define BPLMEAS_LISTEN_ALL_IFC_ADDR "0.0.0.0"
#define BPLMEAS_DEFAULT_RX_PORT_COUNT (4)

extern "C"
{
  /*********************************************************************************************************************/
  /*! @brief        Retrives the current API version.
  *
  *   @return       version of the API
  *
  *   @retval       31 - debug (1), release (0)
  *                 23..16 major
  *                 15..8  minor
  *                 7..0   patch
  *
  *   @pre          BplMeas_Init has to be called first.
  *
  *   @author       b-plus/cwolf
  **********************************************************************************************************************/
  MDIRXAPI_DEPRECATED_API uint32_t MDIRXAPI_DECL BplMeas_GetApiVersion( void );

  /*********************************************************************************************************************/
  /*! @brief        Retrives a event handle for signaling if new data is ready.
  *
  *   @return       Event handle.
  *
  *   @retval       INVALID_HANDLE_VALUE - if API is not properly initialized.
  *   @retval       Otherwise a valid event handle is returned.
  *
  *   @pre          BplMeas_Init has to be called first.
  *
  *   @author       b-plus/cwolf
  **********************************************************************************************************************/
  MDIRXAPI_DEPRECATED_API WaitHandle_t MDIRXAPI_DECL BplMeas_GetDataEventHandle(void);
  
  /*********************************************************************************************************************/
  /*! @brief        Initializes the API and the internal data structures. Is to be called prior to any other functions.
  *
  *   @param[in]    szIP - Ansi string of the IP the data is received from. In doubt use "0.0.0.0".
  *
  *   @return       Result of the operation.
  *
  *   @retval       NOERROR -       API is properly initialized.
  *   @retval       E_UNEXPECTED -  API is already initialized.
  *
  *   @author       b-plus/cwolf
  **********************************************************************************************************************/
  MDIRXAPI_DEPRECATED_API uint32_t MDIRXAPI_DECL BplMeas_Init(char const* szIP);

  /*********************************************************************************************************************/
  /*! @brief        Initializes the API and the internal data structures. Is to be called prior to any other functions.
  *
  *   @param[in]    szIP - Ansi string of the IP the data is received from. In doubt use "0.0.0.0".
  *   @param[in]    port - First port number to listen to for data, default is 3092.
  *   @param[in]    cnt -  Number of consecutive port to use for reception, default is 1.
  *
  *   @return       Result of the operation.
  *
  *   @retval       NOERROR -       API is properly initialized.
  *   @retval       E_UNEXPECTED -  API is already initialized.
  *
  *   @author       b-plus/cwolf
  **********************************************************************************************************************/
  MDIRXAPI_DEPRECATED_API uint32_t MDIRXAPI_DECL BplMeas_InitEx(char const* szIP, uint16_t port, uint8_t cnt);

  /*********************************************************************************************************************/
  /*! @brief        Starts the data reception. The event from BplMeas_GetDataEventHandle will be signalled if new 
  *                 data is available.
  *
  *   @return       Result of the operation.
  *
  *   @retval       NOERROR -       Reception is started.
  *   @retval       E_UNEXPECTED -  API is not initialized.
  *
  *   @author       b-plus/cwolf
  **********************************************************************************************************************/
  MDIRXAPI_DEPRECATED_API uint32_t MDIRXAPI_DECL BplMeas_Start(void);


  /*********************************************************************************************************************/
  /*! @brief        Stops the data reception.
  *
  *   @return       Result of the operation.
  *
  *   @retval       NOERROR -       Reception is stopped.
  *   @retval       E_UNEXPECTED -  API is not initialized.
  *
  *   @author       b-plus/cwolf
  **********************************************************************************************************************/
  MDIRXAPI_DEPRECATED_API uint32_t MDIRXAPI_DECL BplMeas_Stop(void);

  /*********************************************************************************************************************/
  /*! @brief        Deinitializes the API and the internal data structures. Is to be called prior to unload of the library
  *                 or application shutdown.
  *
  *   @return       Result of the operation.
  *
  *   @retval       NOERROR -       API is properly deinitialized.
  *   @retval       E_UNEXPECTED -  API is already deinitialized.
  *
  *   @author       b-plus/cwolf
  **********************************************************************************************************************/
  MDIRXAPI_DEPRECATED_API uint32_t MDIRXAPI_DECL BplMeas_Deinit(void);

  /*********************************************************************************************************************/
  /*! @brief        Retrieves the received data frames from the API.
  *
  *   @important    The pointer (pFrame) contained in each frame structure is to be free'd by the application itself!
  *
  *   @param[in]     FrameArray - Array to be filled with complete frames. The caller is responsible for allocation!
  *   @param[in|out] ArraySize - The number of provided elements is passed in and will be replaces by the concrete
  *                              amount of filled items.
  *
  *   @return       Result of the operation.
  *
  *   @retval       NOERROR -       Data was retrieved.
  *   @retval       E_UNEXPECTED -  API is not initialized.
  *
  *   @author       b-plus/cwolf
  **********************************************************************************************************************/
  MDIRXAPI_DEPRECATED_API uint32_t MDIRXAPI_DECL BplMeas_GetData(BplMeasFrameInfo_t FrameArray[], uint32_t* ArraySize);
  
  /*********************************************************************************************************************/
  /*! @brief        Frees a single allocated frame. If a custom memory manager is used, this function is obsolete if 
  *                 the user keeps track of the allocated memory or frees frame->pFrame on its own.
  *
  *   @param[in]    frame - the frame to be freed.
  *
  *   @return       Result of the operation.
  *
  *   @retval       NOERROR -       Data was retrieved.
  *   @retval       E_UNEXPECTED -  API is not initialized.
  *
  *   @author       b-plus/cwolf
  **********************************************************************************************************************/
  MDIRXAPI_DEPRECATED_API uint32_t MDIRXAPI_DECL BplMeas_FreeData(BplMeasFrameInfo_t* frame);

  /*********************************************************************************************************************/
  /*! @brief        Retrives status and statistic information about the reception.
  *
  *   @param[out]   statistics - Is filled with a snapshot of the current statistics data.
  *
  *   @return       Result of the operation.
  *
  *   @retval       NOERROR -       Statistics were retrieved.
  *   @retval       E_UNEXPECTED -  API is not initialized.
  *
  *   @author       b-plus/cwolf
  **********************************************************************************************************************/
  MDIRXAPI_DEPRECATED_API uint32_t MDIRXAPI_DECL BplMeas_GetStatistics(BplMeasMdiReceptionStatistics_t* statistics);

  /*********************************************************************************************************************/
  /*! @brief        Provides custom memory functions, which replace default ones. This also disables internal
  *                 memory usage checking, so the supplier of the memory is responsible for this.
  *
  *   @param[in]    malloc - function for allocating memory of specified size, must return NULL on failure.
  *   @param[in]    realloc - function for reallocating memory of specified size, must return NULL on failure.
  *   @param[in]    free - function to free memory.
  *   @param[in]    tag - custom value which is passed on every custom method of mm_malloc, mm_realloc or mm_free.
  *
  *   @return       Result of the operation.
  *
  *   @retval       NOERROR -       functions acceped.
  *   @retval       E_UNEXPECTED -  one or more functions not accepted.
  *
  *   @pre          This function has to be called after BplMeas_Init or BplMeas_InitEx, but prior to BplMeas_Start!
  *
  *   @author       b-plus/cwolf
  **********************************************************************************************************************/
  MDIRXAPI_DEPRECATED_API uint32_t MDIRXAPI_DECL BplMeas_RegisterMemManager(mm_malloc_t pfct_malloc, mm_realloc_t pfct_realloc, mm_free_t pfct_free, void* tag);

  /*********************************************************************************************************************/
  /*! @brief        Checks if your CPU supports CRC32c in hardware. This is just for your convenience. The API
  *                 itself will fall back to a software solution if neccessary.
  *
  *   @return       The result if CRC32c in CPU hardware is supported.
  *
  *   @retval       true - hardware calculation is possible, this will be about 10 (or even more) times faster
  *   @retval       false - no hardware calculation possible, a software solution will be used.
  *
  *   @pre          none
  *
  *   @author       b-plus/cwolf
  **********************************************************************************************************************/
  MDIRXAPI_DEPRECATED_API bool MDIRXAPI_DECL BplMeas_IsCrc32cHwSupported();

  /*********************************************************************************************************************/
  /*! @brief        Calculates CRC32c for the given memory chunk.
  *
  *   @param[in]    pData - Pointer to the data for CRC calculation
  *   @param[in]    size - the number of byte the CRC should be calculated by.
  *
  *   @return       The checksum for the passed memory area.
  *
  *   @pre          none
  *
  *   @author       b-plus/cwolf
  **********************************************************************************************************************/
  MDIRXAPI_DEPRECATED_API uint32_t MDIRXAPI_DECL BplMeas_CalcCrc32c(void const* pData, uint32_t size);
  
  /*********************************************************************************************************************/
  /*! @brief        Sets a new timeout value for frames in miliseconds, minimum is 10ms, default is 50ms.
  *
  *   @param[in]    timeout - minimum duration (ms) to wait for more fragments if frame is still incomplete
  *
  *   @return       Result of the operation.
  *
  *   @retval       NOERROR -       timeout value accepted.
  *   @retval       E_UNEXPECTED -  not initialized or timeout smaller than 10ms.
  * 
  *   @pre          Start must not have been called or Stop needs to be called first.
  * 
  *   @author       b-plus/cwolf
  **********************************************************************************************************************/
  MDIRXAPI_DEPRECATED_API uint32_t MDIRXAPI_DECL BplMeas_SetTimeoutMS(uint32_t timeout);

  /*********************************************************************************************************************/
  /*! @brief        Corrupt frames will be delivered and not "silently" dropped
  *
  *   @param[in]    bEnable - true  - corrupt frames will be delivered via API
  *                           false - (default) corrupt frames are dropped
  *   @return       Result of the operation.
  *
  *   @retval       NOERROR -       setting accepted.
  *   @retval       E_UNEXPECTED -  probably not initialized.
  *
  *   @pre          This function has to be called after BplMeas_Init or BplMeas_InitEx, but prior to BplMeas_Start!
  *
  *   @author       b-plus/cwolf
  **********************************************************************************************************************/
  MDIRXAPI_DEPRECATED_API uint32_t MDIRXAPI_DECL BplMeas_DeliverCorruptFrames(bool bEnable);

  /*********************************************************************************************************************/
  /*! @brief        Provides the multicast groups to be joined on reception. This function might be called multiple
  *                 times for multiple groups. But the list cannot be modified after start of reception!
  *
  *   @param[in]    szMcastGroup - canonical IP string for the group to join (e.g. "224.0.0.2").
  *
  *   @return       Result of the operation.
  *
  *   @retval       NOERROR -       functions acceped.
  *   @retval       E_FAIL  -       multicast IP group was not accepted.
  *   @retval       E_UNEXPECTED -  probably not initialized.
  *
  *   @pre          This function has to be called after BplMeas_Init or BplMeas_InitEx, but prior to BplMeas_Start!
  *
  *   @author       b-plus/cwolf
  **********************************************************************************************************************/
  MDIRXAPI_DEPRECATED_API bool MDIRXAPI_DECL BplMeas_JoinMulticastGroup(char const* szMcastGroup);

  /*********************************************************************************************************************/
  /*! @brief        Provides the multicast groups to be joined on reception. This function might be called multiple
  *                 times for multiple groups. But the list cannot be modified after start of reception!
  *
  *   @param[in]    szMcastGroup - canonical IP string for the group to join (e.g. "224.0.0.2").
  *   @param[in]    szIfaceIp    - canonical IP string for the IP of the interface to join on.
  *
  *   @return       Result of the operation.
  *
  *   @retval       NOERROR -       functions acceped.
  *   @retval       E_FAIL  -       multicast IP group was not accepted.
  *   @retval       E_UNEXPECTED -  probably not initialized.
  *
  *   @pre          This function has to be called after BplMeas_Init or BplMeas_InitEx, but prior to BplMeas_Start!
  *
  *   @author       b-plus/cwolf
  **********************************************************************************************************************/
  MDIRXAPI_DEPRECATED_API bool MDIRXAPI_DECL BplMeas_JoinMulticastGroupEx(char const* szMcastGroup, char const* szIfaceIp);
  
}


/**
 * @brief Generic information about the returned interfaces.
 * 
 * For handling different structure versions and for debugging purposes, 
 * each structure contains a incrementing interface version and 
 * the expected size of the structure. 
 */
typedef struct
{
  uint32_t version  : 8;  /**< running number matching structure verstion */
  uint32_t size     : 24; /**< size of the structure */
} MdiRx_StructInfo_t;

/*  we have several "deprecated" interfaces, which have are extended by now. the current
    one can be found in this header. the older ones are stored in the backwards compatibility 
    header, just to clean things up
*/
#include "MDIRxAPI_Backward_Compatibility.h"

/**
 * @brief Generic API interface of the library to interact with the MDI
 * 
 * The V1 of this interface provides exactly the same functionality as the classic
 * ANSI-C interface. However for adding NEW functionality, the ABI interface is much
 * more flexible. Bugfixes (as long as they don't require an interface modification) 
 * will not affect the ABI itself, of course.
 * In other words: new features which require new interfaces will most likely only 
 * be added to the ABI interface and not to the ANSI-C interface.
 */
typedef struct 
{
  MdiRx_StructInfo_t const info; /**< standard interface info */

  /**
   * @brief Request information about the current timesync status
   * @return Version in 0xDDMAMIPA DD=Debug, MA=Major, MI=Minor, PA=Patch
   */
  uint32_t (MDIRXAPI_DECL *GetApiVersion)( void );

  /**
   * @brief Request a abstract event handle to be notified about new data.
   * @return A handle (int of a pipe for linux, autoreset event HANDLE for windows)
   */
  WaitHandle_t (MDIRXAPI_DECL *GetDataEventHandle)(void);

  /**
   * @brief Initializes the API and the internal data structures. Is to be called prior to any other functions.
   * @param szIP    Ansi string of the IP the data is received from. In doubt use "0.0.0.0".
   * @return Result of the operation. NOERROR - API is properly initialized, E_UNEXPECTED - API is already initialized
   */
  uint32_t (MDIRXAPI_DECL *Init)(char const* szIP);
  
  /**
   * @brief Initializes the API and the internal data structures. Is to be called prior to any other functions.
   * @param szIP    Ansi string of the IP the data is received from. In doubt use "0.0.0.0".
   * @param port    First port number to listen to for data, default is 3092.
   * @param cnt     Number of consecutive port to use for reception, default is 4.
   * @return Result of the operation. NOERROR - API is properly initialized, E_UNEXPECTED - API is already initialized
   */
  uint32_t (MDIRXAPI_DECL *InitEx)(char const* szIP, uint16_t port, uint8_t cnt);

  /**
   * @brief Starts the data reception. The event from BplMeas_GetDataEventHandle will be signalled if new data is available.
   * @return Result of the operation. NOERROR - Reception is started, E_UNEXPECTED - API is not initialized
   */
  uint32_t (MDIRXAPI_DECL *Start)(void);
  
  /**
   * @brief Stops the data reception.
   * @return Result of the operation. NOERROR - Reception is stopped, E_UNEXPECTED - API is not initialized
   */
  uint32_t (MDIRXAPI_DECL *Stop)(void);

  /**
   * @brief Deinitializes the API and the internal data structures. Is to be called prior to unload of the library or application shutdown.
   * @return Result of the operation. NOERROR - API is properly deinitialized, E_UNEXPECTED - API is already deinitialized.
   */
  uint32_t (MDIRXAPI_DECL *Deinit)(void);

  /**
   * @brief Retrieves the received data frames from the API.
   *        The pointer (pFrame) contained in each frame structure is to be free'd by the application itself!
   * @param FrameArray  Array to be filled with complete frames. The caller is responsible for allocation!
   * @param ArraySize   The number of provided elements is passed in and will be replaces by the concrete amount of filled items.
   * @return Result of the operation. NOERROR - Data was retrieved, E_UNEXPECTED - API is not initialized.
   */
  uint32_t (MDIRXAPI_DECL *GetData)(BplMeasFrameInfo_t FrameArray[], uint32_t* ArraySize);

  /**
   * @brief Frees a single allocated frame. If a custom memory manager is used, this function is obsolete if the user keeps track of the allocated memory or frees frame->pFrame on its own.
   * @param frame       the frame to be freed.
   * @return Result of the operation. NOERROR - Frame was freed, E_UNEXPECTED - API is not initialized. If it crashes here, probably not a valid frame was passed.
   */
  uint32_t (MDIRXAPI_DECL *FreeData)(BplMeasFrameInfo_t* frame);

  /**
   * @brief Retrives status and statistic information about the reception.
   * @param statistics    Is filled with a snapshot of the current statistics data.
   * @return Result of the operation. NOERROR - Statistics were retrieved, E_UNEXPECTED - API is not initialized. If it crashes here, probably not a valid frame was passed.
   */
  uint32_t (MDIRXAPI_DECL *GetStatistics)(BplMeasMdiReceptionStatistics_t* statistics);

  /**
   * @brief Provides custom memory functions, which replace default ones. This also disables internal memory usage checking, so the supplier of the memory is responsible for this.
   *        The internal memory manager will have an awful performance on purpose! Use your own handler!
   * @param malloc function for allocating memory of specified size, must return NULL on failure.
   * @param realloc function for reallocating memory of specified size, must return NULL on failure.
   * @param free function to free memory.
   * @param tag custom value which is passed on every custom method of mm_malloc, mm_realloc or mm_free. Can be used for class-boxing.
   * @return Result of the operation. NOERROR - functions acceped, E_UNEXPECTED - one or more functions not accepted.
   * @pre This function has to be called after BplMeas_Init or BplMeas_InitEx, but prior to BplMeas_Start!
   */
  uint32_t (MDIRXAPI_DECL *RegisterMemManager)(mm_malloc_t pfct_malloc, mm_realloc_t pfct_realloc, mm_free_t pfct_free, void* tag);

  /**
   * @brief Checks if your CPU supports CRC32c in hardware. This is just for your convenience. The API itself will fall back to a software solution if neccessary.
   * @return true if hardware supports CRC32c calculation, false if it does not support it.
   */
  bool (MDIRXAPI_DECL *IsCrc32cHwSupported)();

  /**
   * @brief Calculates CRC32c for the given memory chunk.
   * @param pData Pointer to the data for CRC calculation.
   * @param size  the number of byte the CRC should be calculated by.
   * @return The checksum for the passed memory area.
   */
  uint32_t (MDIRXAPI_DECL *CalcCrc32c)(void const* pData, uint32_t size);

  /**
   * @brief Sets a new timeout value for frames in miliseconds, minimum is 10ms, default is 50ms.
   * @param timeout minimum duration (ms) to wait for more fragments if frame is still incomplete. Note that a large timeout might exhaust your memory easily!
   * @return Result of the operation. NOERROR - timeout value accepted, E_UNEXPECTED - not initialized or timeout smaller than 10ms.
   * @pre This function has to be called after BplMeas_Init or BplMeas_InitEx, but prior to BplMeas_Start!
   */  
  uint32_t (MDIRXAPI_DECL *SetTimeoutMS)(uint32_t timeout);

  /**
   * @brief Corrupt frames will be delivered and not "silently" dropped
   * @param bEnable true - corrupt frames will be delivered via API, false - (default) corrupt frames are dropped.
   * @return Result of the operation. NOERROR - setting accepted, E_UNEXPECTED - probably not initialized.
   */
  uint32_t (MDIRXAPI_DECL *DeliverCorruptFrames)(bool bEnable);

  /**
   * @brief Provides the multicast groups to be joined on reception. This function might be called multiple times for multiple groups. But the list cannot be modified after start of reception!
   * @param szMcastGroup canonical IP string for the group to join (e.g. "224.0.0.2"). Make sure not to use any reserved/special addresses.
   * @return Result of the operation. True if join was successful, false if not.
   * @pre This function has to be called after BplMeas_Init or BplMeas_InitEx, but prior to BplMeas_Start!
   */
  bool (MDIRXAPI_DECL *JoinMulticastGroup)(char const* szMcastGroup);

  /**
   * @brief Provides the multicast groups to be joined on reception. This function might be called multiple times for multiple groups. But the list cannot be modified after start of reception!
   * @param szMcastGroup  canonical IP string for the group to join (e.g. "224.0.0.2"). Make sure not to use any reserved/special addresses.
   * @param szIfaceIp     canonical IP string for the IP of the interface to join on.
   * @return Result of the operation. True if join was successful, false if not.
   * @pre This function has to be called after BplMeas_Init or BplMeas_InitEx, but prior to BplMeas_Start!
   */
  bool (MDIRXAPI_DECL *JoinMulticastGroupEx)(char const* szMcastGroup, char const* szIfaceIp);

   /**
   * @brief Internal function for testing
   */
  void* (MDIRXAPI_DECL *ExperimentalFeatures)(uint32_t, void*);

  /**
   * @brief Enables the ZeroConf service anouncement.
   * @param szServiceNameAddon  the service name will be starting with the host name and the api name and
   *                            might be extended with a customer string.
   * @return Result of the operation. NOERROR - ZeroConf started, E_UNEXPECTED - probably not initialized.
   * @pre This function has to be called after BplMeas_Init or BplMeas_InitEx, but prior to BplMeas_Start!
   */
  uint32_t (MDIRXAPI_DECL *StartZeroConfAnnounce)( char const* szServiceNameAddon );
  
  /**
   * @brief Disables the ZeroConf service anouncement.
   * @return Result of the operation. NOERROR - ZeroConf stopped, E_UNEXPECTED - probably not initialized.
   * @pre This function has to be called after BplMeas_Init or BplMeas_InitEx, but prior to BplMeas_Start!
   */
  uint32_t (MDIRXAPI_DECL *StopZeroConfAnnounce)( void );

} MdiRx_Reception_interface_v2_t;

/**
 * @brief version type wrapper for clean code upgrade
 */
typedef MdiRx_Reception_interface_v2_t MdiRx_Reception_interface_t;

/**
 * @brief Ethernet frame header for testint/injection interface or low level recorded replay
 */
typedef struct
{
  uint8_t SrcMAC[6];
  uint8_t DstMAC[6];
  uint16_t Type;
}MdiRx_EthHeader_t;

/**
 * @brief IP frame header for testint/injection interface or low level recorded replay
 */
typedef struct 
{
  uint32_t _r0;
  uint32_t _r1;
  uint32_t _r2;
  uint32_t SrcIP;
  uint32_t DstIP;
} MdiRx_IpHeader_t;

/**
 * @brief UDP frame header for testint/injection interface or low level recorded replay
 */
typedef struct
{
  uint16_t SrcPort;
  uint16_t DstPort;
  uint16_t Length;
  uint16_t Checksum;
} MdiRx_UdpHeader_t;

/**
 * @brief Network frame header for testint/injection interface or low level recorded replay
 */
typedef struct
{
  MdiRx_EthHeader_t const* pEthHdr;  /* optional at the moment */
  MdiRx_IpHeader_t  const* pIpHdr;   /* src ip is mandatory */
  MdiRx_UdpHeader_t const* pUdpHdr;  /* mandatory - make sure to have correct length field! */
  uint8_t           const* pPayload; /* mandatory */
} MdiRx_NetworkHeader_t;


typedef struct 
{
  MdiRx_StructInfo_t const info; /**< standard interface info */

  /**
   * @brief Inject recorded or artificical data frame into reception API. The frames are injected and processed synchronuously with the call.
   *        After the call, the packet list can be freed. The data reception callback might still be asynchrounuosly as it depends on the content.
   * @param pPacketList is a list of low-level ethernet frames.
   * @param Number_packets is the numer of passed elements in pPacketList.
   * @return Result of the operation. NOERROR - setting accepted, E_UNEXPECTED - probably not initialized.
   */
  uint32_t (MDIRXAPI_DECL *Inject)(MdiRx_NetworkHeader_t const*const pPacketList, uint32_t Number_packets);
} MdiRx_Injection_interface_v1_t;

/**
 * @brief version type wrapper for clean code upgrade
 */
typedef MdiRx_Injection_interface_v1_t MdiRx_Injection_interface_t;

typedef struct 
{
  MdiRx_StructInfo_t const info; /**< standard interface info */
  MdiRx_Reception_interface_t const*const (MDIRXAPI_DECL *GetRxAPI)( void ); /**< Regular reception handling interface */
} MdiRx_API_interface_v1_t;

typedef struct 
{
  MdiRx_StructInfo_t const info; /**< standard interface info */
  MdiRx_Reception_interface_t const*const (MDIRXAPI_DECL *GetRxAPI)( void );  /**< Regular reception handling interface */
  MdiRx_Injection_interface_t const*const (MDIRXAPI_DECL *GetInjectionAPI)( void );  /**< Low level frame injection interface */
} MdiRx_API_interface_v2_t;

/**
 * @brief version type wrapper for clean code upgrade
 */
typedef MdiRx_API_interface_v2_t MdiRx_API_interface_t;

extern "C"
{
  /**
   * @brief Returns the inteface structure to this library
   * @return A interface structure for accessing the MDI library
   */
  MDIRXAPI_API MdiRx_API_interface_t const*const MDIRXAPI_DECL BplMeas_InvokeApi( void );
}
#endif