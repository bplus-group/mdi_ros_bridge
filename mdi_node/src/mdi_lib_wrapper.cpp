#include "mdi_publisher.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
MDIRXAPI_API MdiRx_API_interface_t const*const (MDIRXAPI_DECL *__BplMeas_DynInvokeApi)( void )=nullptr;
#pragma GCC diagnostic pop

MdiRx_API_interface_t const* BplMeas_DynInvokeApi(void) {
    return __BplMeas_DynInvokeApi?__BplMeas_DynInvokeApi():nullptr;
}

#ifdef WIN32
    #include <fileapi.h>
    inline int __eventWait(HANDLE h, uint32_t timeout) {
    uint32_t x = WaitForSingleObject(h, timeout);
    return x==WAIT_OBJECT_0;
    }

    uint64_t CreateTimestampUs() {
    uint64_t t, f;
    QueryPerformanceCounter((PLARGE_INTEGER)&t);
    QueryPerformanceFrequency((PLARGE_INTEGER)&f);
    return (((uint64_t)(t)) * ((uint64_t)1000000)) / ((uint64_t) f);
    }
#else 
    #include <dlfcn.h>
    #include <unistd.h>
    #include <limits.h>
    #include <sys/select.h>

    int __eventWait(int fd, uint32_t timeout) {
    int result = 3;
    fd_set set;
    FD_ZERO(&set);
    FD_SET(fd, &set);
    timeval tv;
    tv.tv_sec = (int)(timeout / 1000);
    tv.tv_usec = (timeout - (tv.tv_sec*1000)) * 1000;
    int __r = 1;
    if(timeout != 0xFFFFFFFF) {
        __r = select(fd + 1, &set, NULL, NULL, &tv );
    }
    if(__r > 0) {
        static uint8_t readdummy[PIPE_BUF];
        result = 1;
        read(fd, readdummy, PIPE_BUF);
    } else if(__r < 0) {
        result = 0;
    } else {
        result = 0;
    }
    return result;
    }

    uint64_t CreateTimestampUs() {
    timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (((uint64_t)ts.tv_sec) * 1000*1000) + (ts.tv_nsec/1000);
    }
#endif





#ifdef WIN32
  /* to do as soon as ros2-win setup is setup */
#else
void* libmdirxapi_handle=nullptr;
__attribute__((constructor))
static void mdi_receiver_impl_load() {
    /* be very careful with this function: the initialization order of global/static variables with respect to this function depends on the moon phase or so */
    std::string location="";
    Dl_info dl_info;
    dladdr((void *)mdi_receiver_impl_load, &dl_info);
    size_t src_s=strlen(dl_info.dli_fname);
    location.resize(src_s);
    memcpy((void*)location.c_str(), dl_info.dli_fname, src_s);
    /* we need to fiddle a little bit here to get the path to us */
    location=location.substr(0, location.rfind("/"))+"/libmdirxapi.so";
    libmdirxapi_handle=dlopen(location.c_str(), RTLD_NOW|RTLD_GLOBAL);
    if(!libmdirxapi_handle) {
      RCLCPP_ERROR(rclcpp::get_logger(MDI_NODE_NAME), "failed to load libmdirxapi at %s", location.c_str() );
    } else {
      RCLCPP_INFO(rclcpp::get_logger(MDI_NODE_NAME), "loaded libmdirxapi at %p", libmdirxapi_handle);
      *(void**)&__BplMeas_DynInvokeApi = dlsym(libmdirxapi_handle, "BplMeas_InvokeApi");
      RCLCPP_DEBUG(rclcpp::get_logger(MDI_NODE_NAME), "BplMeas_DynInvokeApi at %p", (void*)__BplMeas_DynInvokeApi);
    }
}

__attribute__((destructor)) 
static void mdi_receiver_impl_unload() {
  if(libmdirxapi_handle) {
    dlclose(libmdirxapi_handle);
  }
}
#endif