//**************************************************************************************************
// Copyright (c) 2017 Qualcomm Technologies, Inc.
// All Rights Reserved.
// Confidential and Proprietary - Qualcomm Technologies, Inc.
//**************************************************************************************************

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include <errno.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <getopt.h>
#include <time.h>
#include <drm/msm_drm.h>
#include <drm/drm.h>
#include <linux/msm_ion.h>
#include <gbm_priv.h>
#include <msmgbm.h>
#include <media/msm_media_info.h>
#ifdef BUILD_HAS_WAYLAND_SUPPORT
#include <wayland-server.h>
#endif
#define DRM_DEVICE_NAME "/dev/dri/card0"
#define DRM_MODULE_NAME "msm_drm"
#define ION_DEVICE_NAME "/dev/ion"
#define YUV_420_SP_BPP  1
#define MAX_YUV_PLANES  3
#define DUAL_PLANES     2
#define CHROMA_STEP     2
#define msmgbm_perform gbm_perform
#define msmgbm_get_priv gbm_get_priv
#define PAGE_SIZE (4096)
#define ROUND_UP_PAGESIZE(x) (x + (PAGE_SIZE-1)) & ~(PAGE_SIZE-1)
#define ALIGN(x, align) (((x) + ((align)-1)) & ~((align)-1))

//Global variables
int g_debug_level = LOG_INFO;

//Global Variables
static pthread_mutex_t mutex_obj = PTHREAD_MUTEX_INITIALIZER;
static inline void lock_init(void)
{
    if(pthread_mutex_init(&mutex_obj, NULL))
    {
        LOG(LOG_ERR,"Failed to init Mutex\n %s\n",strerror(errno));
        return NULL;
    }

}
static inline void lock(void)
{
    if(pthread_mutex_lock(&mutex_obj))
    {
        LOG(LOG_ERR,"Failed to lock Mutex\n %s\n",strerror(errno));
        return NULL;
    }

}
static inline void unlock(void)
{
    if(pthread_mutex_unlock(&mutex_obj))
    {
        LOG(LOG_ERR,"Failed to un lock Mutex\n %s\n",strerror(errno));
        return NULL;
    }

}

static inline void lock_destroy(void)
{
    if(pthread_mutex_destroy(&mutex_obj))
        LOG(LOG_ERR,"Failed to init Mutex\n %s\n",strerror(errno));

}

//ION Helper Functions
int ion_open(void)
{
    int fd = open("/dev/ion", O_RDWR);
    if (fd < 0)
        LOG(LOG_ERR, "open /dev/ion failed!\n %s\n",strerror(errno));
    return fd;
}

static inline
struct msmgbm_device * to_msmgbm_device(struct gbm_device *dev)
{
    return (struct msmgbm_device *)dev;
}

static inline
struct msmgbm_bo * to_msmgbm_bo(struct gbm_bo *bo)
{
    return (struct msmgbm_bo *)bo;
}

static inline
struct msmgbm_surface * to_msmgbm_surface(struct gbm_surface*surf)
{
    return (struct msmgbm_surface *)surf;
}

inline
void  msmgbm_dump_hashmap(void)
{
    dump_hashmap();
}

static int
msmgbm_bo_get_fd(struct gbm_bo *bo)
{

    if(bo!=NULL){
        return bo->ion_fd;
    }
    else {
        LOG(LOG_ERR, "NULL or Invalid bo pointer\n");
    return 0;
    }
}

static struct gbm_device*
msmgbm_bo_get_device(struct gbm_bo *bo)
{
    struct msmgbm_bo *msm_gbm_bo = to_msmgbm_bo(bo);
    if(msm_gbm_bo!=NULL){
        return &msm_gbm_bo->device->base;
    }
    else {
        LOG(LOG_ERR, "NULL or Invalid bo pointer\n");
        return NULL;
    }
}

static int
msmgbm_bo_write(struct gbm_bo *bo, const void *buf, size_t count)
{
    struct msmgbm_bo *msm_gbm_bo = to_msmgbm_bo(bo);
   int mappedNow =0;

    if((msm_gbm_bo!=NULL) && (buf != NULL)){
        if(bo->usage_flags & GBM_BO_USE_WRITE) {

            if(count <=0 || count > msm_gbm_bo->size){
                LOG(LOG_ERR, "Invalid count bytes (%d)\n",count);
                return -1;
            }

            if(msm_gbm_bo->cpuaddr == NULL)
            {
                if(msmgbm_bo_cpu_map(bo) == NULL){
                     LOG(LOG_ERR, "Unable to Map to CPU, cannot write to BO\n");
                     return -1;
                }
                mappedNow =1;
            }
            //Write to BO
            memcpy(msm_gbm_bo->cpuaddr, buf, count);

            if(mappedNow){ //Unmap BO, if we mapped it.
                msmgbm_bo_cpu_unmap(bo);
            }
            return 0;
        }
        else {
            LOG(LOG_ERR,"Operation not allowed\n");
        }
    }
    else {
        LOG(LOG_ERR,"NULL or Invalid bo or buffer pointer\n");
    }

    return -1;
}

static void
msmgbm_bo_destroy(struct gbm_bo *bo)
{
    struct msmgbm_bo *msm_gbm_bo = to_msmgbm_bo(bo);
    struct ion_handle_data handle_data;
    struct drm_gem_close gem_close;

    int ret = 0;

    if(NULL != msm_gbm_bo){

        LOG(LOG_DBG," \nmsm_gbm_bo->cpuaddr=0x%x\n msm_gbm_bo->mt_cpuaddr=0x%x\n",
                                msm_gbm_bo->cpuaddr, msm_gbm_bo->mt_cpuaddr);

        LOG(LOG_DBG,"Destroy called for fd=%d",bo->ion_fd);
        /*
         * Perform unmap of both the BO buffer and Metadata
         * We are only handling CPU mapping here
         */
        if((msm_gbm_bo->cpuaddr != NULL)||(msm_gbm_bo->mt_cpuaddr != NULL))
            ret = msmgbm_bo_cpu_unmap(bo);

         //Delete the Map entries if any
        if((decr_refcnt(bo->ion_fd)) && (!msm_gbm_bo->import_flg))
        {
            LOG(LOG_INFO,"Currently closing fd=%d\n",bo->ion_fd);

            /*
             * Close the fd's for both BO and Metadata
             */
            if(bo->ion_fd >= 0){
                if(close(bo->ion_fd))
                    LOG(LOG_ERR,"Failed to Close bo->ion_fd=%d\n%s\n",
                                             bo->ion_fd,strerror(errno));
            }

            if(bo->ion_metadata_fd >= 0){
                if(close(bo->ion_metadata_fd))
                    LOG(LOG_ERR,"Failed to Close bo->ion_metadata_fd=%d\n %s\n",
                                           bo->ion_metadata_fd,strerror(errno));
            }

            /*
             * Close the GEM handle for both the BO buffer and Metadata
             */
            memset(&gem_close, 0, sizeof(gem_close));
            if(bo->handle.u32){
                gem_close.handle=bo->handle.u32;
                if(ioctl(msm_gbm_bo->device->fd,DRM_IOCTL_GEM_CLOSE,&gem_close))
                    LOG(LOG_ERR,"Failed to Close GEM Handle for BO=%p\n%s\n",
                                             bo->handle.u32,strerror(errno));
            }

            memset(&gem_close, 0, sizeof(gem_close));
            if(bo->metadata_handle.u32){
                gem_close.handle=bo->metadata_handle.u32;
                if(ioctl(msm_gbm_bo->device->fd,DRM_IOCTL_GEM_CLOSE,&gem_close))
                    LOG(LOG_ERR,"Failed to Close GEM Handle for BO=%p\n%s\n",
                                     bo->metadata_handle.u32,strerror(errno));

            }
        }

        /*
         * Free the msm_gbo object
         */
        if(msm_gbm_bo != NULL){
            LOG(LOG_DBG,"msm_gbm_bo handle to be freed for BO=%p\n",msm_gbm_bo);
            free(msm_gbm_bo);
            msm_gbm_bo = NULL;
        }
    }
    else
        LOG(LOG_ERR,"NULL or Invalid bo pointer\n");

}

/*************************
 * GetFormatBpp(uint_32 format)
 *
 * returns number of bytes for a supported format
 * returns 0 for unsupported format
 *************************/
static int GetFormatBpp(uint32_t format)
{
   switch(format)
   {
        case GBM_FORMAT_RGB565:
        case GBM_FORMAT_BGR565:
            return 2;
        case GBM_FORMAT_RGB888:
            return 3;
        case GBM_FORMAT_RGBA8888:
        case GBM_FORMAT_RGBX8888:
        case GBM_FORMAT_XRGB8888:
        case GBM_FORMAT_XBGR8888:
        case GBM_FORMAT_ARGB8888:
        case GBM_FORMAT_ABGR8888:
        case GBM_FORMAT_ABGR2101010:
            return 4;
        case GBM_FORMAT_YCbCr_420_SP:
        case GBM_FORMAT_YCrCb_420_SP:
        case GBM_FORMAT_YCbCr_420_SP_VENUS:
        case GBM_FORMAT_NV12_ENCODEABLE:
        case GBM_FORMAT_NV12:
        case GBM_FORMAT_YCbCr_420_TP10_UBWC:
             LOG(LOG_DBG,"YUV format BPP\n");
            return 1;
        default:
            return 0;
   }
   return 0;
}

static int IsFormatSupported(uint32_t format)
{
    int is_supported;

    switch(format)
    {
        case GBM_FORMAT_RGB565:
        case GBM_FORMAT_BGR565:
        case GBM_FORMAT_RGB888:
        case GBM_FORMAT_RGBA8888:
        case GBM_FORMAT_RGBX8888:
        case GBM_FORMAT_XRGB8888:
        case GBM_FORMAT_XBGR8888:
        case GBM_FORMAT_ARGB8888:
        case GBM_FORMAT_ABGR8888:
        case GBM_FORMAT_YCbCr_420_SP:
        case GBM_FORMAT_YCrCb_420_SP:
        case GBM_FORMAT_YCbCr_420_SP_VENUS:
        case GBM_FORMAT_NV12_ENCODEABLE:
        case GBM_FORMAT_NV12:
        case GBM_FORMAT_ABGR2101010:
        case GBM_FORMAT_YCbCr_420_TP10_UBWC:
        case GBM_FORMAT_YCbCr_420_P010_UBWC:
            is_supported = 1;
            LOG(LOG_DBG,"Valid format\n");
            break;
        default:
            is_supported = 0;
    }

    return is_supported;
}

static int
is_format_rgb(uint32_t format)
{
    int result;

    switch(format)
    {
        case GBM_FORMAT_RGB565:
        case GBM_FORMAT_BGR565:
        case GBM_FORMAT_RGB888:
        case GBM_FORMAT_RGBA8888:
        case GBM_FORMAT_RGBX8888:
        case GBM_FORMAT_XRGB8888:
        case GBM_FORMAT_XBGR8888:
        case GBM_FORMAT_ARGB8888:
        case GBM_FORMAT_ABGR8888:
        case GBM_FORMAT_ABGR2101010:
            result = 1;
            break;
        default:
            result = 0;
            break;
    }

    return result;
}

static int init_metadata(uint32_t mt_size, int meta_fd)
{
    struct meta_data_t *data = NULL;

    data = (struct meta_data_t *)mmap(NULL, mt_size, PROT_READ|PROT_WRITE, MAP_SHARED, meta_fd, 0);
    if (data == MAP_FAILED) {
        LOG(LOG_ERR,"Map failed \n %s\n",strerror(errno));
        return GBM_ERROR_BAD_HANDLE;
    }

    memset(data, 0 , mt_size);

    LOG(LOG_DBG,"data->igc=%d\n",data->igc);
    LOG(LOG_DBG,"data->color_space=%d\n",data->color_space);
    LOG(LOG_DBG,"data->interlaced=%d\n",data->interlaced);
    LOG(LOG_DBG,"data->is_buffer_secure=%d\n",data->is_buffer_secure);
    LOG(LOG_DBG,"data->linear_format=%d\n",data->linear_format);
    LOG(LOG_DBG,"data->map_secure_buffer=%d\n",data->map_secure_buffer);
    LOG(LOG_DBG,"data->operation\n=%d\n",data->operation);
    LOG(LOG_DBG,"data->refresh_rate=%f\n",data->refresh_rate);
    LOG(LOG_DBG,"data->s3d_format=%d\n",data->s3d_format);

    if(munmap(data, mt_size)){
        LOG(LOG_ERR,"failed to unmap ptr %p\n%s\n",(void*)data, strerror(errno));
        return GBM_ERROR_BAD_VALUE;
    }

    return GBM_ERROR_NONE;
}


static inline uint32_t query_metadata_size(void)
{
    //currently metadata is just a structure
    //But we will enhance in future as metadata info does
    return (ROUND_UP_PAGESIZE(sizeof(struct meta_data_t)));
}


static struct gbm_bo *
msmgbm_bo_create(struct gbm_device *gbm,
              uint32_t width, uint32_t height,
              uint32_t format, uint32_t usage)
{
    int ret = 0;
    int drm_fd = -1;    // Master fd for DRM
    void *base = NULL;
    void *mt_base = NULL;
    uint32_t aligned_width;
    uint32_t aligned_height;
    uint32_t bo_handles[4] = {0};
    uint32_t pitches[4] = {0};
    uint32_t offsets[4] = {0};
    uint32_t flags = 0;
    uint32_t Bpp = 0;
    uint32_t size = 0;
    uint32_t mt_size = 0;
    uint32_t gem_handle = 0;
    uint32_t mt_gem_handle = 0;
    struct msmgbm_device *msm_dev = to_msmgbm_device(gbm);
    struct gbm_bo *gbmbo = NULL;
    struct msmgbm_bo *msm_gbmbo = NULL;
    struct ion_handle_data handle_data;
    struct ion_fd_data fd_data;
    struct ion_handle_data mt_handle_data;
    struct ion_fd_data mt_fd_data;
    struct ion_allocation_data ionAllocData;
    struct drm_prime_handle drm_args;
    struct gbm_bufdesc bufdesc={width,height,format,usage};

    if(msm_dev == NULL){
        LOG(LOG_ERR,"INVALID Device pointer\n");
        return NULL;
    }

    if(width  <= 0 || height <=0){
        LOG(LOG_ERR,"INVALID width or height\n");
        return NULL;
    }

    if(1 == IsFormatSupported(format))
        Bpp = GetFormatBpp(format);
    else
    {
        LOG(LOG_ERR,"Format (0x%x) not supported\n",format);
        return NULL;
    }

    /*Currently by default we query the aligned dimensions from
      adreno utils*/
    qry_aligned_wdth_hght(&bufdesc, &aligned_width, &aligned_height);

    size = qry_size(&bufdesc, aligned_width, aligned_height);

    LOG(LOG_DBG,"\n size=%d\n width=%d\n height=%d\n aligned_width=%d\n"
          " aligned_height=%d\n",size, width, height, aligned_width, aligned_height);

    /* First we will get ion_fd and gem handle for the frame buffer
     * Size of the ION buffer is in accordance to returned from the adreno helpers
     * Alignment of the buffer is fixed to Page size
     * ION Memory is from, the System heap
     * We get the gem handle from the ion fd using PRIME ioctls
     */
    memset(&ionAllocData, 0, sizeof(ionAllocData));
    memset(&fd_data, 0, sizeof(fd_data));
    memset(&handle_data, 0, sizeof(handle_data));

    /*
     * Depending on the usage flag settinggs we check for the heap from which the ION buffer
     * has to be allocated from.
     * Also cache/non cache buffer allocation
     */
    if (usage & GBM_BO_USAGE_PROTECTED_QTI) {
        ionAllocData.heap_id_mask = ION_HEAP(ION_SECURE_HEAP_ID); /* Secure Heap */
        ionAllocData.flags = ION_FLAG_SECURE | ION_FLAG_CP_PIXEL;
    }else {
        ionAllocData.heap_id_mask = ION_HEAP(ION_SYSTEM_HEAP_ID); /* System Heap */
        ionAllocData.flags = 0;
    }

    ionAllocData.len = size;
    ionAllocData.align = PAGE_SIZE; /*Page size */

    //This ioctl should have failed for a wrong fd, but it does not returns 0
    if(!(ioctl(msm_dev->iondev_fd, ION_IOC_ALLOC, &ionAllocData))){

        fd_data.handle = ionAllocData.handle;
        handle_data.handle = ionAllocData.handle;
        LOG(LOG_DBG,"fd_data.handle:= %p\n",fd_data.handle);
        LOG(LOG_DBG,"ionAllocData.handle:= %p\n",ionAllocData.handle);

        if(!(ioctl(msm_dev->iondev_fd, ION_IOC_MAP, &fd_data))){

            LOG(LOG_DBG,"fd_data.fd:= %d\n",fd_data.fd);

            //Do not mmap if it is secure operation.
            if(!(ionAllocData.flags & ION_FLAG_SECURE)) {
                base = mmap(NULL,size, PROT_READ|PROT_WRITE, MAP_SHARED,
                        fd_data.fd, 0);
                if(base == MAP_FAILED) {
                    LOG(LOG_ERR,"mmap failed memory on BO Err:\n%s\n",strerror(errno));
                    ioctl(msm_dev->iondev_fd, ION_IOC_FREE, &handle_data);
                    return NULL;
                }
                LOG(LOG_DBG,"BO Mapped Addr:= %p\n",base);
            }
        }else{
            LOG(LOG_ERR,"ION_IOC_MAP failed on BO Err:\n%s\n",strerror(errno));
            ioctl(msm_dev->iondev_fd, ION_IOC_FREE, &handle_data);
            return NULL;
        }

    }else{
        LOG(LOG_ERR,"Failed ION_IOC_ALLOC on BO Err:\n%s\n",strerror(errno));
        return NULL;
    }

    //Use PRIME ioctl to convert to GEM handle
    memset(&drm_args, 0, sizeof(drm_args));
    if(msm_dev->fd > 0)
    {

        if(fd_data.fd >0)
        {
            //Perform DRM IOCTL FD to Handle
            drm_args.fd = fd_data.fd;
            if(ioctl(msm_dev->fd,DRM_IOCTL_PRIME_FD_TO_HANDLE, &drm_args))
            {
                LOG(LOG_ERR,"DRM_IOCTL_PRIME_FD_TO_HANDLE failed =%d\n%s\n",
                                                          fd_data.fd,strerror(errno));
                return NULL;
            }
        }
        else
        {
            LOG(LOG_ERR,"ION_IOC_MAP failed on BO Err:\n%s\n",strerror(errno));
            return NULL;
        }

    }else
    {
        LOG(LOG_ERR,"DRM open failed error = %d\n%s\n",strerror(errno));
        return NULL;
    }

    gem_handle=drm_args.handle;
    LOG(LOG_DBG," Gem Handle for BO =:%p\n",gem_handle);

    //Free the ION Handle since we have the fd to deal with
    if(ioctl(msm_dev->iondev_fd, ION_IOC_FREE, &handle_data)){
        LOG(LOG_ERR," Failed to do ION_IOC_FREE  on BO Err:\n %s\n",
                                                   strerror(errno));
        return NULL;
    }

    /* To get ion_fd and gem handle for the metadata structure
     * Alignment of the buffer is fixed to Page size
     * ION Memory is from, the System heap
     * We get the gem handle from the ion fd using PRIME ioctls
     */

   //Reset the data objects to be used for ION IOCTL's
    memset(&ionAllocData, 0, sizeof(ionAllocData));
    memset(&mt_fd_data, 0, sizeof(mt_fd_data));
    memset(&handle_data, 0, sizeof(handle_data));

    ionAllocData.len = sizeof(struct meta_data_t);
    ionAllocData.align = 4096; /*Page size */
    ionAllocData.heap_id_mask= ION_HEAP(ION_SYSTEM_HEAP_ID); /* System Heap */
    ionAllocData.flags |= ION_FLAG_CACHED;

    mt_size = ionAllocData.len;

    if((ioctl(msm_dev->iondev_fd, ION_IOC_ALLOC, &ionAllocData)) == 0){

        mt_fd_data.handle = ionAllocData.handle;
        mt_handle_data.handle = ionAllocData.handle;


        if((ioctl(msm_dev->iondev_fd, ION_IOC_MAP, &mt_fd_data)) == 0){

            LOG(LOG_DBG,"mt_fd_data.fd:= %d\n",mt_fd_data.fd);

            mt_base = mmap(NULL, mt_size, PROT_READ|PROT_WRITE, MAP_SHARED, mt_fd_data.fd, 0);
            if(mt_base == MAP_FAILED) {
                LOG(LOG_ERR,"Failed to do  mapping on Metadata BO Err:\n%s\n",strerror(errno));
                ioctl(msm_dev->iondev_fd, ION_IOC_FREE, &mt_handle_data);
                return NULL;
            }
            LOG(LOG_DBG,"MT_BO Mapped Addr:= %p\n",mt_base);

             // Initiliaze the meta_data structure
             memset(mt_base, 0 , mt_size);
        }else
        {
            LOG(LOG_ERR,"ION_IOC_MAP failed on Metadata BO Err:\n%s\n",strerror(errno));
            ioctl(msm_dev->iondev_fd, ION_IOC_FREE, &mt_handle_data);
            return NULL;
        }

    }else
    {
        LOG(LOG_ERR,"Failed ION_IOC_ALLOC on Metadata BO Err:\n%s\n",strerror(errno));
        return NULL;
    }

    //Use PRIME ioctl to convert to GEM handle
    memset(&drm_args, 0, sizeof(drm_args));
    //Use drm fd returned from previous drmOpen API
    if(mt_fd_data.fd >0)
    {
        //Perform DRM IOCTL FD to Handle
        drm_args.fd = mt_fd_data.fd;
        if(ioctl(msm_dev->fd, DRM_IOCTL_PRIME_FD_TO_HANDLE, &drm_args))
        {
            LOG(LOG_ERR,"failed to import gem_handle for Metadata from prime_fd=%d\n%s\n",strerror(errno));
            return NULL;
        }
    }
    else
    {
        LOG(LOG_ERR,"ION_IOC_MAP failed for Metadata Err:\n%s\n",strerror(errno));
        return NULL;
    }

    mt_gem_handle=drm_args.handle;
    LOG(LOG_DBG,"Gem Handle for Metadata =:%p\n",mt_gem_handle);

    //Free the ION Handle since we have the fd to deal with
    if(ioctl(msm_dev->iondev_fd, ION_IOC_FREE, &mt_handle_data)){
        LOG(LOG_ERR," Failed to do ION_IOC_FREE  on Metadata BO Err:\n %s\n",
                                                            strerror(errno));
        return NULL;
    }

    //Update the secure buffer flag info
    if(usage & GBM_BO_USAGE_PROTECTED_QTI)
    {
        struct meta_data_t *data = (struct meta_data_t *)mt_base;
        data->is_buffer_secure = true;
        LOG(LOG_DBG,"Updating the Secure Buffer status =:%d\n",data->is_buffer_secure);
    }

    // Update UBWC buffer flag info
    if (is_ubwc_enabled(format, usage, usage)) {
        struct meta_data_t *data = (struct meta_data_t *)mt_base;
        data->is_buffer_ubwc = true;
        LOG(LOG_DBG,"Updating the UBWC buffer status =:%d\n",data->is_buffer_ubwc);
    }

    //Create a gbm_buf_info and to the map entry
    struct gbm_buf_info gbo_info;
    gbo_info.fd = fd_data.fd;
    gbo_info.metadata_fd = mt_fd_data.fd;
    gbo_info.format = format;
    gbo_info.height = height;
    gbo_info.width  = width;

    LOG(LOG_DBG," MAP registered bo info gbo_info =:%p\n",&gbo_info);

    //Let us lock and unlock mutex
    lock();
    register_to_hashmap(fd_data.fd,&gbo_info);
    incr_refcnt(fd_data.fd);
    unlock();

    /*
     * Initialize the gbm bo object with the handle's and fd's
     */
    msm_gbmbo = (struct msmgbm_bo *)calloc(1, sizeof(struct msmgbm_bo));

    if (msm_gbmbo == NULL) {
        LOG(LOG_ERR,"Unable to allocate BO\n");
        return NULL;
    }

    gbmbo = &msm_gbmbo->base;
    gbmbo->ion_fd = fd_data.fd;
    gbmbo->handle.u32 = gem_handle;
    gbmbo->ion_metadata_fd = mt_fd_data.fd;
    gbmbo->metadata_handle.u32 = mt_gem_handle;
    gbmbo->fbid = 0;                                     //$ drmModeAddFB2 ?
    gbmbo->format = format;
    gbmbo->width  = width;                               //BO width
    gbmbo->height = height;                              //BO height
    gbmbo->stride = aligned_width*Bpp;
    gbmbo->size = size;                                 // Calculated by qry_size
    gbmbo->usage_flags = usage;
    gbmbo->aligned_width = aligned_width;
    gbmbo->aligned_height = aligned_height;
    gbmbo->bo_destroy = msmgbm_bo_destroy;
    gbmbo->bo_get_fd = msmgbm_bo_get_fd;
    gbmbo->bo_get_device = msmgbm_bo_get_device;
    gbmbo->bo_write = msmgbm_bo_write;
    msm_gbmbo->device = msm_dev;
    msm_gbmbo->cpuaddr = base;
    msm_gbmbo->mt_cpuaddr = mt_base;
    msm_gbmbo->current_state =  GBM_BO_STATE_FREE;
    msm_gbmbo->size = size;
    msm_gbmbo->mt_size = mt_size;
    msm_gbmbo->magic = QCMAGIC;
    msm_gbmbo->ion_handle = handle_data.handle;
    msm_gbmbo->ion_mt_handle = mt_handle_data.handle;

    bo_handles[0] = gbmbo->handle.u32;
    pitches[0] = gbmbo->stride;
    return gbmbo;
}

struct gbm_bo *
msmgbm_bo_import_fd(struct msmgbm_device *msm_dev,
                                                      void *buffer, uint32_t usage)
{
    struct gbm_bo *gbmbo = NULL;
    struct msmgbm_bo *msm_gbmbo = NULL;
    struct drm_prime_handle gemimport_req;
    struct drm_prime_handle mtdadta_gemimport_req;
    struct gbm_import_fd_data *buffer_info = (struct gbm_import_fd_data *)buffer;
    struct gbm_device* gbm_dev = &(msm_dev->base);
    struct gbm_bufdesc bufdesc;
    int ret = 0;
    int Bpp=0;
    unsigned int size = 0;
    unsigned int aligned_width;
    unsigned int aligned_height;

    if (buffer_info == NULL){
        LOG(LOG_ERR,"INVALID buffer_info\n");
        return NULL;
    }

    if(msm_dev == NULL){
        LOG(LOG_ERR,"INVALID Device pointer\n");
        return NULL;
    }

    if(buffer_info->fd < 0)
    {
        LOG(LOG_ERR,"INVALID File descriptor=%d\n",buffer_info->fd);
        return NULL;
    }

    //Query Map
    struct gbm_buf_info gbo_info;

    if(search_hashmap(buffer_info->fd, &gbo_info) == GBM_ERROR_NONE)
    {
        LOG(LOG_DBG,"Map retrieved buf info\n gbm_buf_info.width=%d\n",
                                                        gbo_info.width);
        LOG(LOG_DBG,"gbm_buf_info.fd,gbm_buf_info.metadata_fd,"
                    "gbm_buf_info.height=%d\n gbm_buf_info.format = %d\n",
                    gbo_info.fd,gbo_info.metadata_fd,gbo_info.height,gbo_info.format);

        //If we have a valid entry within the map table then Increment ref count
        incr_refcnt(buffer_info->fd);
    }
    else
    {
        LOG(LOG_INFO,"Search failed so register_to_map\n",
                                                    __func__,__LINE__);
        //Copy the buffer info credentials
        gbo_info.fd=buffer_info->fd;
        gbo_info.metadata_fd = -1; //since we do not have meta fd info here
        gbo_info.format=buffer_info->format;
        gbo_info.width=buffer_info->width;
        gbo_info.height=buffer_info->height;

        lock();
        register_to_hashmap(buffer_info->fd, gbo_info);
        incr_refcnt(buffer_info->fd);
        unlock();

    }

    LOG(LOG_DBG," format: 0x%x width: %d height: %d \n",buffer_info->format, buffer_info->width, buffer_info->height);

    if(1 == IsFormatSupported(buffer_info->format))
        Bpp = GetFormatBpp(buffer_info->format);
    else
    {
        LOG(LOG_ERR,"Format (0x%x) not supported\n",buffer_info->format);
        return NULL;
    }


    /* Import the gem handle for image BO */
    memset(&gemimport_req, 0, sizeof(gemimport_req));
    gemimport_req.fd = buffer_info->fd;

    ret = ioctl(msm_dev->fd, DRM_IOCTL_PRIME_FD_TO_HANDLE, &gemimport_req);

    if (ret != 0){
        LOG(LOG_ERR,"PRIME FD to Handle failed on device(%x), error = %d\n",msm_dev,ret);
        return NULL;
    }

    memset(&mtdadta_gemimport_req, 0, sizeof(mtdadta_gemimport_req));

    //Initialize the helper structure
    bufdesc.Width  = buffer_info->width;
    bufdesc.Height = buffer_info->height;
    bufdesc.Format = buffer_info->format;
    bufdesc.Usage  = usage;

    if (gbo_info.metadata_fd != -1) {
        // Check whether imported gbm bo was UBWC allocated.
        struct meta_data_t *meta_data;
        meta_data = mmap(NULL, query_metadata_size(), PROT_READ|PROT_WRITE, MAP_SHARED,
                         gbo_info.metadata_fd, 0);
        if (meta_data == MAP_FAILED) {
            LOG(LOG_ERR," Map failed for gbo_info.metadata_fd: %d %s\n",
                                       gbo_info.metadata_fd, strerror(errno));
            return GBM_ERROR_BAD_HANDLE;
        }
        if (meta_data->is_buffer_ubwc) {
            bufdesc.Usage |= GBM_BO_USAGE_UBWC_ALIGNED_QTI | GBM_BO_USAGE_HW_RENDERING_QTI;
        }
        if (meta_data) {
          if(munmap(meta_data, query_metadata_size())){
              LOG(LOG_ERR," Map failed \n %s\n",strerror(errno));
              return GBM_ERROR_BAD_VALUE;
          }
        }
    }


    /*Query the size*/
    /*Currently by default we query the aligned dimensions from
      adreno utils*/
    qry_aligned_wdth_hght(&bufdesc, &aligned_width, &aligned_height);
    size = qry_size(&bufdesc, aligned_width, aligned_height);

    msm_gbmbo = (struct msmgbm_bo *)calloc(1, sizeof(struct msmgbm_bo));

    if (msm_gbmbo == NULL) {
        LOG(LOG_ERR,"Unable to allocate BO\n");
        return NULL;
    }

    gbmbo                = &msm_gbmbo->base;
    gbmbo->ion_fd        = buffer_info->fd;
    gbmbo->ion_metadata_fd = gbo_info.metadata_fd;
    gbmbo->handle.u32    = gemimport_req.handle;
    gbmbo->usage_flags   = usage;
    gbmbo->format        = buffer_info->format;
    gbmbo->width         = buffer_info->width;
    gbmbo->height        = buffer_info->height;
    gbmbo->stride        = Bpp*aligned_width;
    gbmbo->size          = size;
    gbmbo->aligned_width  = aligned_width;
    gbmbo->aligned_height = aligned_height;
    gbmbo->bo_destroy    = msmgbm_bo_destroy;
    gbmbo->bo_get_fd     = msmgbm_bo_get_fd;
    gbmbo->bo_get_device = msmgbm_bo_get_device;
    gbmbo->bo_write      = msmgbm_bo_write;
    msm_gbmbo->device    = msm_dev;
    msm_gbmbo->current_state   =  GBM_BO_STATE_FREE;
    gbmbo->metadata_handle.u32 = NULL;
    msm_gbmbo->size      = size;
    msm_gbmbo->magic     = QCMAGIC;
    msm_gbmbo->import_flg = 1;

    LOG(LOG_DBG,"Imported BO Info as below:\n");
    LOG(LOG_DBG,"gbmbo->ion_fd=%d,gbmbo->ion_metadata_fd=%d,"
        "gbmbo->width=%d,gbmbo->height=%d,gbmbo->format=0x%x\n",
        gbmbo->ion_fd,gbmbo->ion_metadata_fd,gbmbo->width,
        gbmbo->height,gbmbo->format);

    return gbmbo;

}

struct gbm_bo *
msmgbm_bo_import_wl_buffer(struct msmgbm_device *msm_dev,
                                                      void *buffer, uint32_t usage)
{
    struct gbm_bo *gbmbo = NULL;
    struct msmgbm_bo *msm_gbmbo = NULL;
    struct drm_prime_handle gemimport_req;
    struct drm_prime_handle mtdadta_gemimport_req;
    struct wl_resource* resource = NULL;
    struct gbm_buf_info *buffer_info = NULL;
    struct gbm_device* gbm_dev = &(msm_dev->base);
    struct gbm_bufdesc bufdesc;
    int ret = 0;
    int Bpp=0;
    unsigned int size = 0;
    unsigned int aligned_width;
    unsigned int aligned_height;


    resource = (struct wl_resource*)(buffer);
    if (resource == NULL){
        LOG(LOG_ERR,"INVALID buffer_info\n");
        return NULL;
    }

    if(msm_dev == NULL){
        LOG(LOG_ERR,"INVALID Device pointer\n");
        return NULL;
    }

    buffer_info = wl_resource_get_user_data(resource);
    if (buffer_info == NULL){
        LOG(LOG_ERR,"INVALID buffer\n");
        return NULL;
    }

    if(buffer_info->fd < 0)
    {
       LOG(LOG_ERR,"INVALID File descriptor(%d)\n",buffer_info->fd);
       return NULL;
    }

    LOG(LOG_DBG,"format: 0x%x width: %d height: %d\n",buffer_info->format, buffer_info->width, buffer_info->height);

    if(1 == IsFormatSupported(buffer_info->format))
        Bpp = GetFormatBpp(buffer_info->format);
    else
    {
        LOG(LOG_ERR," Format (0x%x) not supported\n",
                                                buffer_info->format);
        return NULL;
    }

    //Search Map for a valid entry
    struct gbm_buf_info gbo_info;

    if(search_hashmap(buffer_info->fd, &gbo_info) == GBM_ERROR_NONE)
    {
        LOG(LOG_DBG,"Map retrieved buf info\n gbm_buf_info.width=%d\n",
                                                          gbo_info.width);
        LOG(LOG_DBG,"gbm_buf_info.height=%d\n gbm_buf_info.format = %d\n",
                      gbo_info.height,gbo_info.format);

        //We will check if it has a valid metadata fd and update the same
        if((buffer_info->metadata_fd > 0) && (buffer_info->metadata_fd != gbo_info.metadata_fd))
        {
            lock();
           //Since we have already made sure entry exists
            update_hashmap(buffer_info->fd, buffer_info);
            //If we have a valid entry within the map table then Increment ref count
            incr_refcnt(buffer_info->fd);
            unlock();
        }
    }
    else
    {
        lock();
        register_to_hashmap(buffer_info->fd, buffer_info);
        incr_refcnt(buffer_info->fd);
        unlock();
    }

    /* Import the gem handle for image BO */
    memset(&gemimport_req, 0, sizeof(gemimport_req));
    gemimport_req.fd = buffer_info->fd;

    ret = ioctl(msm_dev->fd, DRM_IOCTL_PRIME_FD_TO_HANDLE, &gemimport_req);

    if (ret != 0){
        LOG(LOG_ERR,"PRIME FD to Handle failed on device(%x), error = %d\n",msm_dev,ret);
        return NULL;
    }

    memset(&mtdadta_gemimport_req, 0, sizeof(mtdadta_gemimport_req));

    if(buffer_info->metadata_fd < 0)
        LOG(LOG_DBG,"INVALID Metadata File descriptor provided(%d)\n",buffer_info->metadata_fd);
    else
    {
        /* Import the gem handle for metadata BO */
        mtdadta_gemimport_req.fd = buffer_info->metadata_fd;

        ret = ioctl(msm_dev->fd, DRM_IOCTL_PRIME_FD_TO_HANDLE, &mtdadta_gemimport_req);

        if (ret != 0){
            LOG(LOG_ERR,"PRIME FD to Handle failed on device(%x), error = %d\n",msm_dev,ret);
            return NULL;
        }
    }

    //Initialize the helper structure
    bufdesc.Width  = buffer_info->width;
    bufdesc.Height = buffer_info->height;
    bufdesc.Format = buffer_info->format;
    bufdesc.Usage  = usage;

    /*Query the size*/
    /*Currently by default we query the aligned dimensions from
      adreno utils*/
    qry_aligned_wdth_hght(&bufdesc, &aligned_width, &aligned_height);
    size = qry_size(&bufdesc, aligned_width, aligned_height);

    msm_gbmbo = (struct msmgbm_bo *)calloc(1, sizeof(struct msmgbm_bo));

    if (msm_gbmbo == NULL) {
        LOG(LOG_ERR,"Unable to allocate BO\n");
        return NULL;
    }

    gbmbo                = &msm_gbmbo->base;
    gbmbo->ion_fd        = buffer_info->fd;
    gbmbo->ion_metadata_fd = buffer_info->metadata_fd;
    gbmbo->handle.u32    = gemimport_req.handle;
    gbmbo->usage_flags   = usage;
    gbmbo->format        = buffer_info->format;
    gbmbo->width         = buffer_info->width;
    gbmbo->height        = buffer_info->height;
    gbmbo->stride        = Bpp*aligned_width;
    gbmbo->size          = size;
    gbmbo->aligned_width  = aligned_width;
    gbmbo->aligned_height = aligned_height;
    gbmbo->bo_destroy    = msmgbm_bo_destroy;
    gbmbo->bo_get_fd     = msmgbm_bo_get_fd;
    gbmbo->bo_get_device = msmgbm_bo_get_device;
    gbmbo->bo_write      = msmgbm_bo_write;
    msm_gbmbo->device    = msm_dev;
    msm_gbmbo->current_state   =  GBM_BO_STATE_FREE;
    gbmbo->metadata_handle.u32 = mtdadta_gemimport_req.handle;
    msm_gbmbo->size      = size;
    msm_gbmbo->magic     = QCMAGIC;
    msm_gbmbo->import_flg = 1;

    return gbmbo;

}

struct gbm_bo *
msmgbm_bo_import_egl_image(struct msmgbm_device *msm_dev,
                                                      void *buffer, uint32_t usage)
{
    //TODO: Need to know how to get either a name or FD for this egl image
    LOG(LOG_ERR,"GBM_BO_IMPORT_EGL_IMAGE not supported\n");
    return NULL;
}

struct gbm_bo *
msmgbm_bo_import_gbm_buf(struct msmgbm_device *msm_dev,
                                                      void *buffer, uint32_t usage)
{
    struct gbm_bo *gbmbo = NULL;
    struct msmgbm_bo *msm_gbmbo = NULL;
    struct drm_prime_handle gemimport_req;
    struct drm_prime_handle mtdadta_gemimport_req;
    struct gbm_buf_info *buffer_info = NULL;
    struct gbm_device* gbm_dev = &(msm_dev->base);
    struct gbm_bufdesc bufdesc;
    int ret = 0;
    int Bpp=0;
    unsigned int size = 0;
    unsigned int aligned_width;
    unsigned int aligned_height;


    buffer_info = (struct gbm_buf_info*)(buffer);
    if (buffer_info == NULL){
        LOG(LOG_ERR, "INVALID buffer_info\n");
        return NULL;
    }

    if(msm_dev == NULL){
        LOG(LOG_ERR,"INVALID Device pointer\n");
        return NULL;
    }

    if(buffer_info->fd < 0)
    {
        LOG(LOG_ERR,"INVALID File descriptor(%d)\n", buffer_info->fd);
        return NULL;
    }

    LOG(LOG_INFO," fd=%d format: 0x%x width: %d height: %d \n",buffer_info->fd, buffer_info->format,
                                                            buffer_info->width, buffer_info->height);

    if(1 == IsFormatSupported(buffer_info->format))
        Bpp = GetFormatBpp(buffer_info->format);
    else
    {
        LOG(LOG_ERR,"Format (0x%x) not supported\n",
                                                    buffer_info->format);
        return NULL;
    }

    //Search Map for a valid entry
    struct gbm_buf_info temp_buf_info;
    ret = search_hashmap(buffer_info->fd, &temp_buf_info);

    //If we have a valid entry within the map table then Increment ref count
    if(ret==GBM_ERROR_NONE)
    {
        incr_refcnt(buffer_info->fd);
        LOG(LOG_DBG,"MAP retrieved buf info\n");
        LOG(LOG_DBG,"temp_buf_info.width=%d\n",
                              temp_buf_info.width);
        LOG(LOG_DBG,"temp_buf_info.height=%d\n",
                             temp_buf_info.height);
        LOG(LOG_DBG,"temp_buf_info.format=%d\n",
                                    temp_buf_info.format);
        LOG(LOG_DBG,"temp_buf_info.meta_fd=%d\n",
                                    temp_buf_info.metadata_fd);

        //if the current metadata fd is invalid we need to override
        if(buffer_info->metadata_fd < 0)
            buffer_info->metadata_fd = temp_buf_info.metadata_fd;
    }
    else
    {
        LOG(LOG_INFO," MAP table is empty\n");

        lock();
        register_to_hashmap(buffer_info->fd, buffer_info);
        incr_refcnt(buffer_info->fd);
        LOG(LOG_INFO,"Registered fd=%d to table\n",buffer_info->fd);
        unlock();

    }

    /* Import the gem handle for image BO */
    memset(&gemimport_req, 0, sizeof(gemimport_req));
    gemimport_req.fd = buffer_info->fd;

    ret = ioctl(msm_dev->fd, DRM_IOCTL_PRIME_FD_TO_HANDLE, &gemimport_req);

    if (ret != 0){
        LOG(LOG_ERR,"PRIME FD to Handle failed on device(%x)\n %s\n",
                                               msm_dev,strerror(errno));
        return NULL;
    }

    memset(&mtdadta_gemimport_req, 0, sizeof(mtdadta_gemimport_req));

    if(buffer_info->metadata_fd < 0)
        LOG(LOG_DBG,"INVALID Metadata File descriptor provided(%d)\n",
                                             buffer_info->metadata_fd);
    else
    {

        /* Import the gem handle for metadata BO */
        mtdadta_gemimport_req.fd = buffer_info->metadata_fd;

        ret = ioctl(msm_dev->fd, DRM_IOCTL_PRIME_FD_TO_HANDLE, &mtdadta_gemimport_req);

        if (ret != 0){
            LOG(LOG_ERR,"PRIME FD to Handle failed on device(%x)\n %s\n",
                                                   msm_dev,strerror(errno));
            return NULL;
        }
    }

    //Initialize the helper structure
    bufdesc.Width  = buffer_info->width;
    bufdesc.Height = buffer_info->height;
    bufdesc.Format = buffer_info->format;
    bufdesc.Usage  = usage;

    if (buffer_info->metadata_fd != -1) {
        // Check whether imported gbm bo was UBWC allocated.
        struct meta_data_t *meta_data;

        meta_data = mmap(NULL, query_metadata_size(), PROT_READ|PROT_WRITE, MAP_SHARED,
                         buffer_info->metadata_fd, 0);
        if (meta_data == MAP_FAILED) {
            LOG(LOG_ERR," Map failed for gbo_info->metadata_fd: %d %s\n",
                                       buffer_info->metadata_fd, strerror(errno));
            return GBM_ERROR_BAD_HANDLE;
        }
        if (meta_data->is_buffer_ubwc) {
            bufdesc.Usage |= GBM_BO_USAGE_UBWC_ALIGNED_QTI | GBM_BO_USAGE_HW_RENDERING_QTI;
        }
        if (meta_data) {
          if(munmap(meta_data, query_metadata_size())){
              LOG(LOG_ERR," Map failed \n %s\n",strerror(errno));
              return GBM_ERROR_BAD_VALUE;
          }
        }
    }


    /*Query the size*/
    /*Currently by default we query the aligned dimensions from
      adreno utils*/
    qry_aligned_wdth_hght(&bufdesc, &aligned_width, &aligned_height);
    size = qry_size(&bufdesc, aligned_width, aligned_height);

    msm_gbmbo = (struct msmgbm_bo *)calloc(1, sizeof(struct msmgbm_bo));

    if (msm_gbmbo == NULL) {
        LOG(LOG_ERR," Unable to allocate BO OoM\n");
        return NULL;
    }

    gbmbo                  = &msm_gbmbo->base;
    gbmbo->ion_fd          = buffer_info->fd;
    gbmbo->ion_metadata_fd = buffer_info->metadata_fd;
    gbmbo->handle.u32      = gemimport_req.handle;
    gbmbo->usage_flags     = usage;
    gbmbo->format          = buffer_info->format;
    gbmbo->width           = buffer_info->width;
    gbmbo->height          = buffer_info->height;
    gbmbo->stride          = Bpp*aligned_width;
    gbmbo->aligned_width   = aligned_width;
    gbmbo->aligned_height  = aligned_height;
    gbmbo->size            = size;
    gbmbo->bo_destroy      = msmgbm_bo_destroy;
    gbmbo->bo_get_fd       = msmgbm_bo_get_fd;
    gbmbo->bo_get_device   = msmgbm_bo_get_device;
    gbmbo->bo_write        = msmgbm_bo_write;
    msm_gbmbo->device      = msm_dev;
    msm_gbmbo->current_state   =  GBM_BO_STATE_FREE;
    gbmbo->metadata_handle.u32 = mtdadta_gemimport_req.handle;
    msm_gbmbo->size            = size;
    msm_gbmbo->magic           = QCMAGIC;
    msm_gbmbo->import_flg      = 1;

    LOG(LOG_DBG,"Imported BO Info as below:\n");
    LOG(LOG_DBG,"gbmbo->ion_fd=%d,gbmbo->ion_metadata_fd=%d,"
        "gbmbo->width=%d,gbmbo->height=%d,gbmbo->format=0x%x\n",
        gbmbo->ion_fd,gbmbo->ion_metadata_fd,gbmbo->width,
        gbmbo->height,gbmbo->format);

    return gbmbo;

}

struct gbm_bo *
msmgbm_bo_import(struct gbm_device *gbm,
              uint32_t type, void *buffer, uint32_t usage)
{
     struct msmgbm_device *msm_dev = to_msmgbm_device(gbm);

    if(msm_dev == NULL){
        LOG(LOG_ERR," INVALID Device pointer\n");
        return NULL;
    }

    LOG(LOG_DBG,"msmgbm_bo_import invoked\n");

     switch(type){
     case GBM_BO_IMPORT_FD:
         LOG(LOG_DBG,"msmgbm_bo_import_fd invoked\n");
         return msmgbm_bo_import_fd(msm_dev,buffer,usage);
         break;
     case GBM_BO_IMPORT_WL_BUFFER:
         LOG(LOG_DBG,"msmgbm_bo_import_wl_buffer invoked\n");
         return msmgbm_bo_import_wl_buffer(msm_dev,buffer,usage);
         break;
     case GBM_BO_IMPORT_EGL_IMAGE:
        LOG(LOG_DBG,"msmgbm_bo_import_image invoked\n");
        return msmgbm_bo_import_egl_image(msm_dev,buffer,usage);
        break;
     case GBM_BO_IMPORT_GBM_BUF_TYPE:
        LOG(LOG_DBG,"msmgbm_bo_import_gbm_buf invoked\n");
        return msmgbm_bo_import_gbm_buf(msm_dev,buffer, usage);
        break;
     default:
         LOG(LOG_DBG," Invalid buffer type (%d), error = %d\n",type);
         return NULL;
     }
}

#ifdef ALLOCATE_SURFACE_BO_AT_CREATION
static void free_surface_bo(struct msmgbm_surface *surf, int num_bo_to_free)
{
    int index;
    for(index =0; index < num_bo_to_free; index++) {
        if(surf->bo[index] != NULL){
            gbm_bo_destroy(&surf->bo[index]->base);
            surf->bo[index] = NULL;
        }
    }
}
#endif

static void
msmgbm_surface_destroy(struct gbm_surface *surf)
{
    struct msmgbm_surface *msm_gbm_surf = to_msmgbm_surface(surf);

    if(msm_gbm_surf!=NULL){
#ifdef ALLOCATE_SURFACE_BO_AT_CREATION
        free_surface_bo(msm_gbm_surf, NUM_BACK_BUFFERS);
#endif
        free(msm_gbm_surf);
        msm_gbm_surf = NULL;
    }
    else {
         LOG(LOG_ERR," NULL or Invalid surface pointer\n");
    }

    return;
}

static struct gbm_bo *
msmgbm_surface_lock_front_buffer(struct gbm_surface *surf)
{
    struct msmgbm_surface *msm_gbm_surface = to_msmgbm_surface(surf);
    int index;

    if(msm_gbm_surface != NULL)
    {
#ifdef ALLOCATE_SURFACE_BO_AT_CREATION
        for(index =0; index < NUM_BACK_BUFFERS; index++)
        {
            if((msm_gbm_surface->bo[index]!= NULL) && \
                (msm_gbm_surface->bo[index]->current_state == GBM_BO_STATE_NEW_FRONT_BUFFER))
            {
                msm_gbm_surface->bo[index]->current_state = GBM_BO_STATE_INUSE_BY_COMPOSITOR;
                return &msm_gbm_surface->bo[index]->base;
            }
        }
        LOG(LOG_ERR,"No Front BO found\n");
#else
        for(index =0; index < NUM_BACK_BUFFERS; index++)
        {
            if((msm_gbm_surface->bo_slot[index] == SURFACE_BOSLOT_STATE_HAS_NEW_FRONT_BUFFER) && \
                (msm_gbm_surface->bo[index]->current_state == GBM_BO_STATE_NEW_FRONT_BUFFER))
            {
                msm_gbm_surface->bo_slot[index] =  SURFACE_BOSLOT_STATE_INUSE_BY_COMPOSITOR;
                msm_gbm_surface->bo[index]->current_state = GBM_BO_STATE_INUSE_BY_COMPOSITOR;
                return  &msm_gbm_surface->bo[index]->base;
            }
        }
        LOG(LOG_ERR,"No Front BO found\n");
#endif
    }
    else {
        LOG(LOG_ERR," NULL or Invalid surface pointer\n");
    }
    return NULL;
}

static void
msmgbm_surface_release_buffer(struct gbm_surface *surf, struct gbm_bo *bo)
{
    struct msmgbm_surface *msm_gbm_surf = to_msmgbm_surface(surf);
    struct msmgbm_bo *msm_gbm_bo = to_msmgbm_bo(bo);
    int index =0;

    if((msm_gbm_surf == NULL) || (msm_gbm_bo == NULL)) {
         LOG(LOG_ERR," Invalid surface or BO pointer\n");
         return;
    }

#ifdef ALLOCATE_SURFACE_BO_AT_CREATION
    for(index=0;index < NUM_BACK_BUFFERS;index++)
    {
        if((msm_gbm_surf->bo[index] != NULL) && \
                   (msm_gbm_surf->bo[index] == msm_gbm_bo) && \
                   (msm_gbm_surf->bo[index]->current_state == GBM_BO_STATE_INUSE_BY_COMPOSITOR)) //Not sure if this check is necessary
        {
           // BO will be destroyed when surface is destroyed, just set BO state to Free.
           msm_gbm_surf->bo[index]->current_state = GBM_BO_STATE_FREE;
           return;
        }
    }
    LOG(LOG_ERR,"Invalid Input BO, BO is not locked\n");
#else
    for(index=0;index < NUM_BACK_BUFFERS;index++)
    {
        if((msm_gbm_surf->bo_slot[index] == SURFACE_BOSLOT_STATE_INUSE_BY_COMPOSITOR) && \
            (msm_gbm_surf->bo[index]->current_state == GBM_BO_STATE_INUSE_BY_COMPOSITOR)) //Not sure if this check is necessary
        {
            msm_gbm_surf->bo_slot[index] = SURFACE_BOSLOT_STATE_FREE;
            msm_gbm_surf->bo[index] = NULL;
            return;
        }
    }
   LOG(LOG_ERR,"Invalid Input BO, BO is not locked\n");
#endif
}

static int
msmgbm_surface_has_free_buffers(struct gbm_surface *surf)
{
    struct msmgbm_surface *msm_gbm_surface = to_msmgbm_surface(surf);
    int index;

    if(msm_gbm_surface != NULL){
#ifdef ALLOCATE_SURFACE_BO_AT_CREATION
        for(index =0; index < NUM_BACK_BUFFERS; index++) {
            if((msm_gbm_surface->bo[index]!= NULL) &&(msm_gbm_surface->bo[index]->current_state == GBM_BO_STATE_FREE)){
                 return 1;
            }
        }
#else
        for(index =0; index < NUM_BACK_BUFFERS; index++) {
            if(msm_gbm_surface->bo_slot[index] == SURFACE_BOSLOT_STATE_FREE){
                return 1;
            }
        }
#endif
    }
    else {
         LOG(LOG_ERR," NULL or Invalid surface pointer\n");
    }
    return 0;
}

static struct gbm_surface *
msmgbm_surface_create(struct gbm_device *gbm,
                                                 uint32_t width, uint32_t height,
                                                 uint32_t format, uint32_t flags)
{
    struct msmgbm_device *msm_dev = to_msmgbm_device(gbm);
    struct gbm_surface *gsurf = NULL;
    struct msmgbm_surface*msm_gbmsurf = NULL;
#ifdef ALLOCATE_SURFACE_BO_AT_CREATION
    int index;
#endif

    if(msm_dev == NULL){
        LOG(LOG_ERR," INVALID device pointer\n");
        return NULL;
    }

    if(width  <= 0 || height <=0){
        LOG(LOG_ERR," INVALID width or height\n");
        return NULL;
    }

    msm_gbmsurf = (struct msmgbm_surface *)calloc(1, sizeof(struct msmgbm_surface));

    if (msm_gbmsurf == NULL) {
        LOG(LOG_ERR," Unable to allocate Surface OoM\n");
        return NULL;
    }

    gsurf = &msm_gbmsurf->base;
    gsurf->format = format;
    gsurf->height = height;
    gsurf->width = width;
    gsurf->flags = flags;
    gsurf->surface_destroy = msmgbm_surface_destroy;
    gsurf->surface_has_free_buffers =  msmgbm_surface_has_free_buffers;
    gsurf->surface_release_buffer = msmgbm_surface_release_buffer;
    gsurf->surface_lock_front_buffer = msmgbm_surface_lock_front_buffer;

    msm_gbmsurf->device = msm_dev;
    msm_gbmsurf->magic = QCMAGIC;

#ifdef ALLOCATE_SURFACE_BO_AT_CREATION
    for(index =0; index < NUM_BACK_BUFFERS; index++) {
       msm_gbmsurf->bo[index] = to_msmgbm_bo(msmgbm_bo_create(gbm, width, height, format, flags));
       if(msm_gbmsurf->bo[index] == NULL){
           LOG(LOG_ERR," Unable to create Surface BO %d\n", index);
           free_surface_bo(msm_gbmsurf, index);
           return NULL;
       }
    }
#endif

    return gsurf;
}

static int
msmgbm_device_is_format_supported(struct gbm_device *gbm,
                               uint32_t format, uint32_t usage)
{
    struct msmgbm_device *msm_dev = to_msmgbm_device(gbm);

    if(msm_dev != NULL){
        if(IsFormatSupported(format))
            return 1;
    }
    else {
         LOG(LOG_ERR,"NULL or Invalid device pointer\n");
    }
    return 0;
}

static void
msmgbm_device_destroy(struct gbm_device *gbm)
{
    struct msmgbm_device *msm_dev = to_msmgbm_device(gbm);

    //Destroy the platform wrapper cpp object
    platform_wrap_deinstnce();

    //Destroy the  mapper cpp object
    msmgbm_mapper_deinstnce();

    lock_destroy();

    //Close the ion device fd
    if(msm_dev->iondev_fd > 0)
        close(msm_dev->iondev_fd);

    if(msm_dev != NULL){
        free(msm_dev);
        msm_dev = NULL;
    }
    else {

         LOG(LOG_ERR,"NULL or Invalid device pointer\n");
    }
    return;
}

static struct gbm_device *
msmgbm_device_create(int fd)
{
    struct gbm_device *gbmdevice = NULL;
    struct msmgbm_device *msm_gbmdevice =  NULL;

    msm_gbmdevice = (struct msmgbm_device *)calloc(1,sizeof(struct msmgbm_device));

    if (msm_gbmdevice == NULL) {
        return NULL;
    }

    //Update the debug level here
    config_dbg_lvl();

   //Instantiate the platform wrapper cpp object
   if(platform_wrap_instnce())
     return NULL;

    //Instantiate the mapper cpp object
    if(msmgbm_mapper_instnce())
      return NULL;

    lock_init();

    //open the ion device
    msm_gbmdevice->iondev_fd=ion_open();
    if (msm_gbmdevice->iondev_fd < 0){

        LOG(LOG_ERR,"Failed to open ION device\n");
        return NULL;
    }

    gbmdevice =  &msm_gbmdevice->base;
    gbmdevice->fd = fd;
    gbmdevice->destroy = msmgbm_device_destroy;
    gbmdevice->is_format_supported = msmgbm_device_is_format_supported;
    gbmdevice->bo_create = msmgbm_bo_create;
    gbmdevice->bo_import = msmgbm_bo_import;
    gbmdevice->surface_create = msmgbm_surface_create;
    msm_gbmdevice->fd = fd;
    msm_gbmdevice->magic = QCMAGIC;

    LOG(LOG_DBG,"gbm device fd= %d\n",gbmdevice->fd);

    return gbmdevice;
}

struct gbm_backendpriv g_msm_priv = {
   .backend_name = "msm_drm", //As this will be using MSM DRM
   .create_device = msmgbm_device_create,
};

struct gbm_backendpriv *msmgbm_get_priv(void)
{
    return &g_msm_priv;
}

//$this API Vs QCMAGIC
unsigned int msmgbm_device_get_magic(struct gbm_device *dev)
{
    struct msmgbm_device *msm_dev = to_msmgbm_device( dev);

    if(msm_dev == NULL){
        LOG(LOG_ERR,"NULL or Invalid device pointer\n");
        return 0;
    }
    else
    {
        drm_auth_t auth;
        int ret  =0;
        memset(&auth, 0, sizeof(drm_auth_t));

        ret = ioctl(msm_dev->fd, DRM_IOCTL_GET_MAGIC, &auth);
        if (ret)
        {
            LOG(LOG_ERR,"GET_MAGIC failed for device (%x)\n %s\n",
                                    msm_dev,strerror(errno));
            return 0;
        }
        return auth.magic;
    }
}

int msmgbm_surface_set_front_bo(struct gbm_surface *surf, struct gbm_bo *bo)
{
    struct msmgbm_surface*msm_gbm_surface = to_msmgbm_surface(surf);
    struct msmgbm_bo *msm_gbm_bo = to_msmgbm_bo(bo);
    int index;

    if(msm_gbm_bo!=NULL ||msm_gbm_surface !=NULL )
    {
#ifdef ALLOCATE_SURFACE_BO_AT_CREATION
        for(index =0; index < NUM_BACK_BUFFERS; index++)
        {
            if((msm_gbm_surface->bo[index]!= NULL) && \
                 (msm_gbm_surface->bo[index] == msm_gbm_bo)  && \
                 (msm_gbm_surface->bo[index]->current_state == GBM_BO_STATE_INUSE_BY_GPU))
            {
                     msm_gbm_surface->bo[index]->current_state = GBM_BO_STATE_NEW_FRONT_BUFFER;
                     return GBM_ERROR_NONE;
            }
        }
        LOG(LOG_ERR," INVALID BO, Passed BO was not obtained using \
                                msmgbm_surface_get_free_bo\n");
        return GBM_ERROR_NO_RESOURCES;
#else
        for(index =0; index < NUM_BACK_BUFFERS; index++)
        {
            if(msm_gbm_surface->bo_slot[index] == SURFACE_BOSLOT_STATE_FREE)
            {
                msm_gbm_surface->bo_slot[index] = SURFACE_BOSLOT_STATE_HAS_NEW_FRONT_BUFFER;
                msm_gbm_surface->bo[index] = msm_gbm_bo;
                msm_gbm_surface->bo[index]->current_state = GBM_BO_STATE_NEW_FRONT_BUFFER;
                return GBM_ERROR_NONE;
            }
        }
        LOG(LOG_ERR," NO Free BO slot found!!\n");
       return GBM_ERROR_NO_RESOURCES;
#endif
    }
    else
    {
         LOG(LOG_ERR," INVALID BO or Surface pointer\n");
         return GBM_ERROR_BAD_HANDLE;
    }
}

#ifdef ALLOCATE_SURFACE_BO_AT_CREATION
struct gbm_bo* msmgbm_surface_get_free_bo(struct gbm_surface *surf)
{
    struct msmgbm_surface *msm_gbm_surface = to_msmgbm_surface(surf);
    int index;

    if(msm_gbm_surface != NULL)
    {
            for(index =0; index < NUM_BACK_BUFFERS; index++)
            {
                if((msm_gbm_surface->bo[index]!= NULL) && \
                    (msm_gbm_surface->bo[index]->current_state == GBM_BO_STATE_FREE))
                {
                    msm_gbm_surface->bo[index]->current_state = GBM_BO_STATE_INUSE_BY_GPU;
                    return &msm_gbm_surface->bo[index]->base;
                }
            }
            LOG(LOG_ERR," NO Free BO found!!\n");
    }
    else
    {
        LOG(LOG_ERR," NULL or Invalid surface pointer\n");
    }
    return NULL;
}
#else
struct gbm_bo* msmgbm_surface_get_free_bo(struct gbm_surface *surf)
{
     LOG(LOG_ERR," This API is not supported.\n");
     return NULL;
}
#endif

void* msmgbm_bo_cpu_map(struct gbm_bo *bo)
{
    struct msmgbm_bo *msm_gbm_bo = to_msmgbm_bo(bo);
    struct drm_msm_gem_info gem_info_req;
    int ret = 0;


    if(msm_gbm_bo!=NULL)
    {
       if(msm_gbm_bo->cpuaddr)
       {
            return msm_gbm_bo->cpuaddr;
       }

       msm_gbm_bo->cpuaddr = mmap(0, bo->size, PROT_READ | PROT_WRITE, MAP_SHARED,
                                  bo->ion_fd, 0);

        if(msm_gbm_bo->cpuaddr == ((void *)-1))
        {
              msm_gbm_bo->cpuaddr = NULL;
              LOG(LOG_ERR," CPU MAP FAILED for BO(%x)\n",bo);
        }

    }
    else
    {
        LOG(LOG_ERR," NULL or Invalid bo pointer\n");
        msm_gbm_bo->cpuaddr = NULL;
    }

    return msm_gbm_bo->cpuaddr;
}

int msmgbm_bo_cpu_unmap(struct gbm_bo *bo)
{
    struct msmgbm_bo *msm_gbm_bo = to_msmgbm_bo(bo);
    if(msm_gbm_bo!=NULL)
    {
        //BO buffer
        if (msm_gbm_bo->cpuaddr != NULL)
        {

            LOG(LOG_DBG," unmapping msm_gbm_bo->cpuaddr=0x%x\n",
                                           msm_gbm_bo->cpuaddr);
            if(munmap((void *)msm_gbm_bo->cpuaddr, bo->size))
                LOG(LOG_ERR," munmap failed for msm_gbm_bo->cpuaddr=0x%x\n",
                                                msm_gbm_bo->cpuaddr);
        }
        msm_gbm_bo->cpuaddr = NULL;

        //Metadata buffer
        if (msm_gbm_bo->mt_cpuaddr != NULL)
        {
            LOG(LOG_DBG," unmapping msm_gbm_bo->mt_cpuaddr=0x%x\n",
                                           msm_gbm_bo->mt_cpuaddr);
            if(munmap((void *)msm_gbm_bo->mt_cpuaddr, msm_gbm_bo->mt_size))
                LOG(LOG_ERR," munmap failed for msm_gbm_bo->mt_cpuaddr=0x%x\n",
                                                msm_gbm_bo->mt_cpuaddr);
        }
        msm_gbm_bo->mt_cpuaddr = NULL;

        return GBM_ERROR_NONE;
    }
    else
    {
        LOG(LOG_ERR," NULL or Invalid bo pointer\n");
        return GBM_ERROR_BAD_HANDLE;
    }
}

//$ how to go about the same
void* msmgbm_bo_gpu_map(struct gbm_bo *bo)
{
    /* John --  This piece of the code needs to go through UHAB to get GPU address */
    struct msmgbm_bo *msm_gbm_bo = to_msmgbm_bo(bo);
    return GBM_ERROR_UNSUPPORTED;
}

int msmgbm_bo_gpu_unmap(struct gbm_bo *bo)
{
    // BO will be unmapped from GPU MMU after GEM CLOSE. Silent return
    return GBM_ERROR_UNSUPPORTED;
}

static inline
size_t msmgbm_bo_get_size(struct gbm_bo *bo)
{
   struct msmgbm_bo *msm_gbm_bo = to_msmgbm_bo(bo);
    if(msm_gbm_bo!=NULL)
    {
        return msm_gbm_bo->size;
    }
    else
    {
        LOG(LOG_ERR," NULL or Invalid bo pointer\n");
        return GBM_ERROR_BAD_HANDLE;
    }
}

static inline
int msmgbm_validate_device(struct gbm_device *dev){
    struct msmgbm_device*msm_dev = to_msmgbm_device(dev);

    if((msm_dev != NULL) && (msm_dev->magic == QCMAGIC) ) {
        return GBM_ERROR_NONE;
    }
    else {
        return GBM_ERROR_BAD_HANDLE;
    }
}

static inline
int  msmgbm_validate_surface(struct gbm_surface *surf){
    struct msmgbm_surface*msmgbm_surface = to_msmgbm_surface(surf);

    if((msmgbm_surface != NULL) && (msmgbm_surface->magic == QCMAGIC) ) {
        return GBM_ERROR_NONE;
    }
    else {
        return GBM_ERROR_BAD_HANDLE;
    }
}

static inline
const char*  msmgbm_get_drm_device_name(void){
    return DRM_DEVICE_NAME;
}

int  msmgbm_device_authenticate_magic(struct gbm_device *dev, drm_magic_t magic){
    struct msmgbm_device *msm_dev = to_msmgbm_device(dev);

    if(msm_dev == NULL){
        LOG(LOG_ERR," NULL or Invalid device pointer\n");
        return GBM_ERROR_BAD_HANDLE;
    }
    else
    {
        drm_auth_t auth;
        int ret  =0;
        memset(&auth, 0, sizeof(drm_auth_t));
        auth.magic = magic;

        ret = ioctl(msm_dev->fd, DRM_IOCTL_AUTH_MAGIC, &auth);
        if (ret)
        {
            LOG(LOG_ERR," AUTH_MAGIC failed for device (%x)\n %s\n",
                                    msm_dev,strerror(errno));
            return GBM_ERROR_BAD_VALUE;
        }
    }
    return GBM_ERROR_NONE;

}

struct gbm_bo*  msmgbm_bo_import_from_name(struct gbm_device *dev, unsigned int name)
{
    struct msmgbm_device *msm_dev = to_msmgbm_device(dev);
    struct drm_prime_handle gemimport_req;
    struct gbm_bo *gbmbo = NULL;
    struct msmgbm_bo *msm_gbmbo = NULL;
    int fd = (int)name;
    int ret = 0;


    if(NULL == msm_dev){
        LOG(LOG_ERR," INVALID Device pointer\n");
        return NULL;
    }

    if(0 > fd)
    {
        LOG(LOG_ERR," INVALID File descriptor(%d)\n", name);
        return NULL;
    }

    memset(&gemimport_req, 0, sizeof(gemimport_req));
    gemimport_req.fd = fd;

    ret = ioctl(msm_dev->fd, DRM_IOCTL_PRIME_FD_TO_HANDLE, &gemimport_req);

    if (ret != 0){
        LOG(LOG_ERR," PRIME FD to Handle failed on device(%x), error = %d\n",
                                                        msm_dev,ret);
        return NULL;
    }

    msm_gbmbo = (struct msmgbm_bo *)calloc(1, sizeof(struct msmgbm_bo));

    if (msm_gbmbo == NULL) {
        LOG(LOG_ERR," Unable to allocate BO OoM\n");
        return NULL;
    }

    gbmbo =  &msm_gbmbo->base;
    gbmbo->ion_fd = fd;
    gbmbo->handle.u32 = gemimport_req.handle;
    gbmbo->bo_destroy = msmgbm_bo_destroy;
    gbmbo->bo_get_fd= msmgbm_bo_get_fd;
    gbmbo->bo_get_device = msmgbm_bo_get_device;
    gbmbo->bo_write = msmgbm_bo_write;
    msm_gbmbo->device = msm_dev;
    msm_gbmbo->current_state =  GBM_BO_STATE_FREE;
    msm_gbmbo->magic = QCMAGIC;
    msm_gbmbo->name = name;
    //msm_gbmbo->size = gem_open.size;

    return gbmbo;
}

int msmgbm_bo_get_name(struct gbm_bo* bo)
{
    struct msmgbm_bo *msm_gbmbo = to_msmgbm_bo(bo);
    struct ion_fd_data fd_data;
    struct drm_prime_handle drm_args;
    int ret;


    if(NULL == msm_gbmbo){
        LOG(LOG_ERR," INVALID BO pointer\n");
        return -1;
    }

    if(0 == msm_gbmbo->name)
    {
        memset(&drm_args, 0, sizeof(drm_args));
        drm_args.handle = msm_gbmbo->base.handle.u32;
        msm_gbmbo->name = bo->ion_fd;
    }
    return msm_gbmbo->name;
}

//$ How are we planning to expose to the clients the var args usage
int msmgbm_perform(int operation, ... )
{
    int res = GBM_ERROR_UNSUPPORTED;
    va_list args;


    va_start(args, operation);

    switch (operation){
        case GBM_PERFORM_GET_SURFACE_WIDTH:
            {
                struct gbm_surface *gbm_surf = va_arg(args, struct gbm_surface *);
                uint32_t *width  = va_arg(args, uint32_t *);

                struct msmgbm_surface* msmgbm_surf = to_msmgbm_surface(gbm_surf);

                if(msmgbm_surf != NULL && msmgbm_surf->magic == QCMAGIC){
                    *width = gbm_surf->width;
                     res = GBM_ERROR_NONE;
                }
                else
                    res = GBM_ERROR_BAD_HANDLE;
            }
            break;

        case GBM_PERFORM_GET_SURFACE_HEIGHT:
            {
                struct gbm_surface *gbm_surf = va_arg(args, struct gbm_surface *);
                uint32_t *height  = va_arg(args, uint32_t *);

                struct msmgbm_surface* msmgbm_surf = to_msmgbm_surface(gbm_surf);

                if(msmgbm_surf != NULL && msmgbm_surf->magic == QCMAGIC){
                    *height = gbm_surf->height;
                     res = GBM_ERROR_NONE;
                }
                else
                    res = GBM_ERROR_BAD_HANDLE;
            }
            break;

        case GBM_PERFORM_GET_SURFACE_FORMAT:
            {
                struct gbm_surface *gbm_surf = va_arg(args, struct gbm_surface *);
                uint32_t *format  = va_arg(args, uint32_t *);

                struct msmgbm_surface* msmgbm_surf = to_msmgbm_surface(gbm_surf);

                if(msmgbm_surf != NULL && msmgbm_surf->magic == QCMAGIC){
                    *format = gbm_surf->format;
                     res = GBM_ERROR_NONE;
                }
                else
                    res = GBM_ERROR_BAD_HANDLE;
            }
            break;

        case GBM_PERFORM_SET_SURFACE_FRONT_BO:
            {
                struct gbm_surface *gbm_surf = va_arg(args, struct gbm_surface *);
                struct gbm_bo *gbo = va_arg(args,struct gbm_bo *);

                res = msmgbm_surface_set_front_bo(gbm_surf, gbo);
            }
            break;

        case GBM_PERFORM_GET_SURFACE_FREE_BO:
            {
                struct gbm_surface *gbm_surf = va_arg(args, struct gbm_surface *);
                struct gbm_bo **gbo = va_arg(args,struct gbm_bo **);

                *gbo = msmgbm_surface_get_free_bo(gbm_surf);
                if(*gbo)
                    res = GBM_ERROR_NONE;
                else
                    res = GBM_ERROR_BAD_VALUE;
            }
            break;
        case GBM_PERFORM_VALIDATE_SURFACE:
            {
                struct gbm_surface *gbm_surf = va_arg(args, struct gbm_surface *);
                res = msmgbm_validate_surface(gbm_surf);
            }
            break;
        case GBM_PERFORM_CPU_MAP_FOR_BO:
            {
                struct gbm_bo *gbo = va_arg(args, struct gbm_bo *);
                void **map_addr_handle = va_arg(args,void **);

                *map_addr_handle=msmgbm_bo_cpu_map(gbo);
                if(*map_addr_handle)
                    res = GBM_ERROR_NONE;
                else
                    res = GBM_ERROR_BAD_VALUE;
            }
            break;
        case GBM_PERFORM_CPU_UNMAP_FOR_BO:
            {
                struct gbm_bo *gbo = va_arg(args, struct gbm_bo *);

                res = msmgbm_bo_cpu_unmap(gbo);
            }
            break;
        case GBM_PERFORM_GET_GPU_ADDR_FOR_BO:
            {
                struct gbm_bo *gbo = va_arg(args, struct gbm_bo *);
                uint64_t *gpu_addr = va_arg(args,uint64_t *);
                struct msmgbm_bo *msm_gbm_bo = to_msmgbm_bo(gbo);

                if(msm_gbm_bo!=NULL) {
                    *gpu_addr = msm_gbm_bo->gpuaddr;
                    res = GBM_ERROR_NONE;
                }
                else
                    res = GBM_ERROR_BAD_VALUE;
            }
            break;
        case GBM_PERFORM_SET_GPU_ADDR_FOR_BO:
            {
                struct gbm_bo *gbo = va_arg(args, struct gbm_bo *);
                uint64_t gpu_addr = va_arg(args,uint64_t);
                struct msmgbm_bo *msm_gbm_bo = to_msmgbm_bo(gbo);

                if(msm_gbm_bo!=NULL) {
                    msm_gbm_bo->gpuaddr = gpu_addr;
                    res = GBM_ERROR_NONE;
                }
                else
                    res = GBM_ERROR_BAD_VALUE;
            }
            break;
        case GBM_PERFORM_GET_BO_SIZE:
            {
                struct gbm_bo *gbo = va_arg(args, struct gbm_bo *);
                size_t* size = va_arg(args,size_t*);

                *size = msmgbm_bo_get_size(gbo);
                if(*size)
                    res = GBM_ERROR_NONE;
                else
                    res = GBM_ERROR_BAD_HANDLE;
            }
            break;
        case GBM_PERFORM_GET_BO_NAME:
            {
                struct gbm_bo *gbo = va_arg(args, struct gbm_bo *);
                int* name = va_arg(args,int*);

                *name = msmgbm_bo_get_name(gbo);
                if(*name > 0)
                    res = GBM_ERROR_NONE;
                else
                    res = GBM_ERROR_BAD_HANDLE;
            }
            break;
        case GBM_PERFORM_IMPORT_BO_FROM_NAME:
            {
                struct gbm_device *gbm_dev = va_arg(args, struct gbm_device *);
                struct gbm_bo **gbo = va_arg(args,struct gbm_bo **);
                int name = va_arg(args,int);

                *gbo = NULL;
                *gbo = msmgbm_bo_import_from_name(gbm_dev,name);
                if(*gbo)
                    res = GBM_ERROR_NONE;
                else
                    res = GBM_ERROR_BAD_HANDLE;
            }
            break;
        case GBM_PERFORM_GET_DRM_DEVICE_MAGIC:
            {
                struct gbm_device *gbm_dev = va_arg(args, struct gbm_device *);
                drm_magic_t *magic_id = va_arg(args,drm_magic_t*);

                *magic_id = msmgbm_device_get_magic(gbm_dev);
                if(*magic_id)
                    res = GBM_ERROR_NONE;
                else
                    res = GBM_ERROR_BAD_HANDLE;
            }
            break;
        case GBM_PERFORM_AUTH_DRM_DEVICE_MAGIC:
            {
                struct gbm_device *gbm_dev = va_arg(args, struct gbm_device *);
                drm_magic_t magic_id = va_arg(args,drm_magic_t);

                res = msmgbm_device_authenticate_magic(gbm_dev,magic_id);
            }
            break;
        case GBM_PERFORM_GET_DRM_DEVICE_NAME:
            {
                char *drm_dev_name = va_arg(args,char *);
                strcpy(drm_dev_name, DRM_DEVICE_NAME);
                res = GBM_ERROR_NONE;
            }
            break;
        case GBM_PERFORM_VALIDATE_DEVICE:
            {
                struct gbm_device *gbm_dev = va_arg(args, struct gbm_device *);
                res = msmgbm_validate_device(gbm_dev);
            }
            break;
        case GBM_PERFORM_GET_METADATA:
            {
                struct gbm_bo *gbo = va_arg(args, struct gbm_bo *);
                int paramType = va_arg(args,int);
                void* param = va_arg(args,void*);
                LOG(LOG_DBG," Passed param address & value = 0x%x, 0x%x\n",
                             (unsigned int *)param,*(unsigned int *)param);
                res = msmgbm_get_metadata(gbo,paramType,param);
            }
            break;
        case GBM_PERFORM_SET_METADATA:
            {
                struct gbm_bo *gbo = va_arg(args, struct gbm_bo *);
                int paramType = va_arg(args,int);
                void* param = va_arg(args,void*);
                LOG(LOG_DBG," Passed param address & value = 0x%x, 0x%x\n",
                             (unsigned int *)param,*(unsigned int *)param);

                res = msmgbm_set_metadata(gbo,paramType,param);
            }
            break;
        case GBM_PERFORM_GET_UBWC_STATUS:
            {
                struct gbm_bo *gbo = va_arg(args, struct gbm_bo *);
                int *ubwc_status = va_arg(args,int *);

                 res = msmgbm_get_metadata(gbo, GBM_METADATA_GET_UBWC_BUF_STAT,
                                           (void *)ubwc_status);
            }
            break;
        case GBM_PERFORM_GET_YUV_PLANE_INFO:
            {
                struct gbm_bo *gbo = va_arg(args, struct gbm_bo *);
                generic_buf_layout_t *buf_lyt = va_arg(args, generic_buf_layout_t *);

                res = msmgbm_yuv_plane_info(gbo,buf_lyt);
            }
            break;
        case GBM_PERFORM_GET_SECURE_BUFFER_STATUS:
            {
                struct gbm_bo *gbo = va_arg(args, struct gbm_bo *);
                bool *sec_buf_stat = va_arg(args,int *);

                *sec_buf_stat = 0;
                res = msmgbm_get_metadata(gbo, GBM_METADATA_GET_SECURE_BUF_STAT,
                                                             (void *)sec_buf_stat);
            }
            break;
        case GBM_PERFORM_GET_METADATA_ION_FD:
            {
                struct gbm_bo *gbo = va_arg(args, struct gbm_bo *);
                int *metadata_fd = va_arg(args,int *);

                if((gbo == NULL) || (metadata_fd == NULL))
                    return GBM_ERROR_BAD_HANDLE;

                if((gbo->ion_metadata_fd) < 0)
                {
                    //Let us try looking through the map table in case if we have
                    //an update, since last import call?
                    struct gbm_buf_info temp_buf_info;
                    res = search_hashmap(gbo->ion_fd, &temp_buf_info);

                    if((res == GBM_ERROR_NONE) && (temp_buf_info.metadata_fd > 0))
                    {
                        LOG(LOG_DBG,"MAP retrieved buf info\n");
                        LOG(LOG_DBG,"temp_buf_info.metadata_fd=%d\n",
                                          temp_buf_info.metadata_fd);
                        LOG(LOG_DBG,"temp_buf_info.width=%d\n",
                                              temp_buf_info.width);
                        LOG(LOG_DBG,"temp_buf_info.height=%d\n",
                                             temp_buf_info.height);
                        LOG(LOG_DBG,"temp_buf_info.format=%d\n",
                                              temp_buf_info.format);

                        //save the same in the gbo handle as well
                        gbo->ion_metadata_fd = temp_buf_info.metadata_fd;


                    }

                }

                *metadata_fd = gbo->ion_metadata_fd;

                return GBM_ERROR_NONE;
            }
            break;
        case GBM_PERFORM_GET_BO_ALIGNED_WIDTH:
            {
                struct gbm_bo *gbo = va_arg(args, struct gbm_bo *);
                uint32_t *align_wdth = va_arg(args, uint32_t *);

                *align_wdth = gbo->aligned_width;

                res = GBM_ERROR_NONE;
            }
            break;
        case GBM_PERFORM_GET_BO_ALIGNED_HEIGHT:
            {
                struct gbm_bo *gbo = va_arg(args, struct gbm_bo *);
                uint32_t *align_hght = va_arg(args, uint32_t *);

                *align_hght = gbo->aligned_height;

                res = GBM_ERROR_NONE;
            }
            break;
        case GBM_PERFORM_DUMP_HASH_MAP:
            {
                 msmgbm_dump_hashmap();
                 res = GBM_ERROR_NONE;
            }
             break;
        case GBM_PERFORM_DUMP_BO_CONTENT:
            {
                struct gbm_bo *gbo = va_arg(args, struct gbm_bo *);
                res = msmgbm_bo_dump(gbo);
            }
            break;
        case GBM_PERFORM_GET_PLANE_INFO:
            {
                struct gbm_bo *gbo = va_arg(args, struct gbm_bo *);
                struct generic_buf_layout_t * buf_lyt = va_arg(args, struct generic_buf_layout_t *);
                res = msmgbm_get_buf_lyout(gbo, buf_lyt);
            }
            break;
        case GBM_PERFORM_DEFAULT_INIT_COLOR_META:
            {
                struct ColorMetaData *clr_mta = va_arg(args, struct ColorMetaData *);
                msmsgbm_default_init_hdr_color_info_mdata(clr_mta);
                res = GBM_ERROR_NONE;
            }
            break;
        case GBM_PERFORM_DUMP_COLOR_META:
            {
                struct ColorMetaData *clr_mta = va_arg(args, struct ColorMetaData *);
                msmgbm_log_hdr_color_info_mdata(clr_mta);
                res = GBM_ERROR_NONE;
            }
            break;
        case GBM_PERFORM_GET_BUFFER_SIZE_DIMENSIONS:
            {
                struct gbm_buf_info * buf_info = va_arg(args, struct gbm_buf_info *);
                uint32_t *align_wdth = va_arg(args, uint32_t *);
                uint32_t *align_hght = va_arg(args, uint32_t *);
                uint32_t *size = va_arg(args, uint32_t *);

                struct gbm_bufdesc bufdesc = {buf_info->width, buf_info->height,
                                              buf_info->format, 0};

                qry_aligned_wdth_hght(&bufdesc, align_wdth, align_hght);

                *size = qry_size(&bufdesc, *align_wdth, *align_hght);

                res = GBM_ERROR_NONE;
            }
            break;
        case GBM_PERFORM_GET_SURFACE_UBWC_STATUS:
            {
                struct gbm_surface *gbm_surf = va_arg(args, struct gbm_surface *);
                int *ubwc_status = va_arg(args,int *);

                *ubwc_status =  is_ubwc_enabled(gbm_surf->format, gbm_surf->flags, gbm_surf->flags);

                res = GBM_ERROR_NONE;
            }
            break;
        case GBM_PERFORM_GET_RGB_DATA_ADDRESS:
            {
                struct gbm_bo *gbo = va_arg(args, struct gbm_bo *);
                void **rgb_data = va_arg(args, void **);
                res = msmgbm_get_rgb_data_address(gbo, rgb_data);
            }
			break;
        case GBM_PERFORM_GET_WL_RESOURCE_FROM_GBM_BUF_INFO:
            {
                struct gbm_buf_info *buffer_info = va_arg(args, struct gbm_buf_info *);
                struct wl_resource *resource = va_arg(args, struct wl_resource *);
                struct gbm_buf_resource *buf_resource;

                buf_resource = (struct gbm_buf_resource *)calloc(1, sizeof(struct gbm_buf_resource));
                buf_resource->buffer_info = buffer_info;
                buf_resource->magic = 0x1;
                wl_resource_set_user_data(resource, (void *)buf_resource);

                res = GBM_ERROR_NONE;
            }
            break;
        case GBM_PERFORM_GET_GBM_BUF_INFO_FROM_WL_RESOURCE:
            {
                struct wl_resource *resource = va_arg(args, struct wl_resource *);
                struct gbm_buf_info *buffer_info = va_arg(args, struct gbm_buf_info *);
                struct gbm_buf_resource *buf_resource;

                if (resource != NULL) {
                    buf_resource = wl_resource_get_user_data(resource);
                    if (buf_resource == NULL) {
                        LOG(LOG_ERR,"INVALID buffer_info\n");
                        res = GBM_ERROR_UNDEFINED;
                    } else if (buf_resource->magic != 0x1) {
                      LOG(LOG_ERR,"INVALID buffer_info\n");
                      res = GBM_ERROR_BAD_HANDLE;
                    } else {
                        buffer_info->fd = buf_resource->buffer_info->fd;
                        buffer_info->metadata_fd = buf_resource->buffer_info->metadata_fd;
                        buffer_info->width = buf_resource->buffer_info->width;
                        buffer_info->height = buf_resource->buffer_info->height;
                        buffer_info->format = buf_resource->buffer_info->format;
                        res = GBM_ERROR_NONE;
                    }
                }
            }
            break;
         default:
                LOG(LOG_INFO,"PERFORM Operation not supported\n");
            break;
    }
    va_end(args);
    return res;
}

int msmgbm_get_rgb_data_address(struct gbm_bo *gbo, void **rgb_data) {
    int ret = GBM_ERROR_NONE;
    int ubwc_status = 0;
    int Bpp; //Bytes per pixel
    int metaBuffer_size;

    // This api is for RGB* formats
    if (!is_valid_uncmprsd_rgb_format(gbo->format)) {
      return GBM_ERROR_BAD_VALUE;
    }

    // Query whether BO is UBWC allocated
    msmgbm_get_metadata(gbo, GBM_METADATA_GET_UBWC_BUF_STAT, &ubwc_status);

    if (!ubwc_status) {
      // BO is Linearly allocated. Return cpu_address
      *rgb_data = msmgbm_bo_cpu_map(gbo);
    } else {
      // BO is UBWC allocated
      // Compute bytes per pixel
      Bpp = get_bpp_for_uncmprsd_rgb_format(gbo->format);

      // Compute meta size
      metaBuffer_size = get_rgb_ubwc_metabuffer_size(gbo->aligned_width, gbo->aligned_height, Bpp);

      *rgb_data = (void *) (msmgbm_bo_cpu_map(gbo) + metaBuffer_size);
    }

    return ret;
}

int msmgbm_set_metadata(struct gbm_bo *gbo, int paramType,void *param) {
    struct msmgbm_bo *msm_gbm_bo = to_msmgbm_bo(gbo);
    struct meta_data_t *data = NULL;
    size_t size = 0;
    void *base = NULL;
    int res = GBM_ERROR_NONE;
    int map_flg = 0;

    if(!msm_gbm_bo)
        return GBM_ERROR_BAD_HANDLE;

    if((gbo->ion_metadata_fd) <= 0)
    {
        LOG(LOG_ERR," Invalid metadata_fd=%d\n",gbo->ion_metadata_fd);
        return GBM_ERROR_BAD_HANDLE;
    }

    size = query_metadata_size();

    base = msm_gbm_bo->mt_cpuaddr;

    if(!base)
    {
        base = mmap(NULL, size, PROT_READ|PROT_WRITE, MAP_SHARED, gbo->ion_metadata_fd, 0);
        if (base == MAP_FAILED) {
            LOG(LOG_ERR," Map failed for metadata_fd=%d\n %s\n",
                               gbo->ion_metadata_fd,strerror(errno));
            return GBM_ERROR_BAD_HANDLE;
        }
        map_flg = 1;
    }

    data = (struct meta_data_t *)base;

    // If parameter is NULL reset the specific MetaData Key
    if (!param)
       data->operation &= ~paramType;

    data->operation |= paramType;

    LOG(LOG_DBG," operation Enabled %d\n",data->operation);
    LOG(LOG_DBG," Passed param address & value = 0x%x, 0x%x\n",
                                               (unsigned int *)param,*(unsigned int *)param);

    switch (paramType) {
        case GBM_METADATA_SET_INTERLACED:
             data->interlaced = *((unsigned int *)param);
             break;
        case GBM_METADATA_SET_REFRESH_RATE:
             data->refresh_rate = *((float *)param);
             break;
        case GBM_METADATA_SET_COLOR_SPACE:
             data->color_space = *((int *)param);
             break;
        case GBM_METADATA_SET_MAP_SECURE_BUFFER:
             data->map_secure_buffer = *((uint32_t *)param);
             break;
        case GBM_METADATA_SET_S3DFORMAT:
             data->s3d_format = *((uint32_t *)param);
             break;
        case GBM_METADATA_SET_LINEAR_FORMAT:
             data->linear_format = *((uint32_t *)param);
             break;
        case GBM_METADATA_SET_IGC:
             data->igc = *((int *)param);
             break;
        case GBM_METADATA_SET_COLOR_METADATA:
             data->color_info = *((ColorMetaData *)param);
             break;
        default:
            LOG(LOG_ERR," Operation currently not supported\n");
            res = GBM_ERROR_UNSUPPORTED;
            break;
    }

    if(map_flg)
    {
        if(munmap(base, size)){
            LOG(LOG_ERR,"failed to unmap ptr %p\n%s\n",(void*)base,strerror(errno));
            res = GBM_ERROR_BAD_VALUE;
        }
    }
    return res;
}

int msmgbm_get_metadata(struct gbm_bo *gbo, int paramType,void *param) {
    struct msmgbm_bo *msm_gbm_bo = to_msmgbm_bo(gbo);
    struct meta_data_t *data = NULL;
    size_t size = 0;
    void *base;
    int res = GBM_ERROR_NONE;
    int map_flg = 0;

    if(!msm_gbm_bo)
        return GBM_ERROR_BAD_HANDLE;

    if((gbo->ion_metadata_fd) <= 0)
    {
        //Let us try looking through the map table in case if we have
        //an update, since last import call?
        struct gbm_buf_info temp_buf_info;
        res = search_hashmap(gbo->ion_fd, &temp_buf_info);

        if((res==GBM_ERROR_NONE) && (temp_buf_info.metadata_fd > 0))
        {
            LOG(LOG_DBG,"MAP retrieved buf info\n");
            LOG(LOG_DBG,"temp_buf_info.metadata_fd=%d\n",
                              temp_buf_info.metadata_fd);
            LOG(LOG_DBG,"temp_buf_info.width=%d\n",
                                  temp_buf_info.width);
            LOG(LOG_DBG,"temp_buf_info.height=%d\n",
                                 temp_buf_info.height);
            LOG(LOG_DBG,"temp_buf_info.format=%d\n",
                                  temp_buf_info.format);

            //save the same in the gbo handle as well
            gbo->ion_metadata_fd = temp_buf_info.metadata_fd;
        }
        else
        {
            LOG(LOG_INFO,"metadata_fd=%d and hence valid meta info cannot be retrieved\n",
                                                                      gbo->ion_metadata_fd);
            LOG(LOG_INFO,"We will make a graceful exit\n");
            return GBM_ERROR_NONE;
        }

    }

    //Calculate the size for mapping the structure
    size = query_metadata_size();

    base=msm_gbm_bo->mt_cpuaddr;

    if(!base)
    {
        base = mmap(NULL, size, PROT_READ|PROT_WRITE, MAP_SHARED, gbo->ion_metadata_fd, 0);
        if (base == MAP_FAILED) {
            LOG(LOG_ERR," Map failed for gbo->ion_metadata_fd\n %d %s\n",
                                       gbo->ion_metadata_fd,strerror(errno));
            return GBM_ERROR_BAD_HANDLE;
        }
        map_flg = 1; //set it to address unmapping
    }

    data = (struct meta_data_t *)base;

    if (!param) {
        LOG(LOG_ERR," Null or Invalid Param Pointer\n");
        return GBM_ERROR_BAD_HANDLE;
    }

    LOG(LOG_DBG,"gbo->ion_fd=%d\n",gbo->ion_fd);
    LOG(LOG_DBG,"gbo->ion_metadata_fd=%d\n",gbo->ion_metadata_fd);

    switch (paramType) {
        case GBM_METADATA_GET_INTERLACED:
            *((uint32_t *)param) = data->interlaced;
            break;
        case GBM_METADATA_GET_REFRESH_RATE:
            *((float *)param) = data->refresh_rate;
            break;
        case GBM_METADATA_GET_COLOR_SPACE:
            *((int *)param) = 0;

            if (data->operation & GBM_METADATA_SET_COLOR_SPACE) {
              *((int *)param) = data->color_space;
            } else if (data->operation & GBM_METADATA_SET_COLOR_METADATA) {
              switch (data->color_info.colorPrimaries) {
                case ColorPrimaries_BT709_5:
                  *((int *)param) = GBM_METADATA_COLOR_SPACE_ITU_R_709;
                  break;
                case ColorPrimaries_BT601_6_525:
                  *((int *)param) = (data->color_info.range) ?
                                      GBM_METADATA_COLOR_SPACE_ITU_R_601_FR :
                                      GBM_METADATA_COLOR_SPACE_ITU_R_601;
                  break;
                case ColorPrimaries_BT2020:
                  *((int *)param) = (data->color_info.range) ?
                                     GBM_METADATA_COLOR_SPACE_ITU_R_2020_FR :
                                     GBM_METADATA_COLOR_SPACE_ITU_R_2020;
                  break;
                default:
                  LOG(LOG_ERR," Unknown Color Space:%d\n", data->color_info.colorPrimaries);
                  break;
              }
            }
            break;
        case GBM_METADATA_GET_MAP_SECURE_BUFFER:
            *((uint32_t *)param) = data->map_secure_buffer;
            break;
        case GBM_METADATA_GET_SECURE_BUF_STAT:
            *((int *)param) = data->is_buffer_secure;
            break;
        case GBM_METADATA_GET_S3DFORMAT:
            *((uint32_t *)param) = data->s3d_format;
            break;
        case GBM_METADATA_GET_LINEAR_FORMAT:
            *((uint32_t *)param) = data->linear_format;
            break;
        case GBM_METADATA_GET_IGC:
            *((int *)param) = data->igc;
            break;
        case GBM_METADATA_GET_COLOR_METADATA:
            *((ColorMetaData *)param) = data->color_info;
            break;
        case GBM_METADATA_GET_UBWC_BUF_STAT:
            *((int *)param) = data->is_buffer_ubwc;
            break;
        default:
            LOG(LOG_ERR," Operation currently not supported\n");
            res = GBM_ERROR_UNSUPPORTED;
            break;
    }

    if(map_flg)
    {
        if(munmap(base, size)){
            LOG(LOG_ERR," Map failed \n %s\n",strerror(errno));
            res = GBM_ERROR_BAD_VALUE;
        }
    }
    return res;
}


void get_yuv_sp_plane_info(int width, int height, int bpp,
                       generic_buf_layout_t *buf_lyt)
{
    unsigned int ystride, cstride;

    ystride=width * bpp;
    cstride=width * bpp;

    buf_lyt->num_planes = DUAL_PLANES;

    buf_lyt->planes[0].top_left = buf_lyt->planes[0].offset = 0;
    buf_lyt->planes[1].top_left = buf_lyt->planes[1].offset = ystride * height;
    buf_lyt->planes[2].top_left = buf_lyt->planes[2].offset = ystride * height + 1;
    buf_lyt->planes[0].v_increment = ystride; //stride     in bytes
    buf_lyt->planes[1].v_increment = cstride;
    buf_lyt->planes[2].v_increment = cstride;
    buf_lyt->planes[0].h_increment = CHROMA_STEP*bpp; //chroma step
    buf_lyt->planes[1].h_increment = CHROMA_STEP*bpp;
    buf_lyt->planes[2].h_increment = CHROMA_STEP*bpp;

}


void get_yuv_ubwc_sp_plane_info(int width, int height,
                          int color_format, generic_buf_layout_t *buf_lyt)
{
   // UBWC buffer has these 4 planes in the following sequence:
   // Y_Meta_Plane, Y_Plane, UV_Meta_Plane, UV_Plane
   unsigned int y_meta_stride, y_meta_height, y_meta_size;
   unsigned int y_stride, y_height, y_size;
   unsigned int c_meta_stride, c_meta_height, c_meta_size;
   unsigned int alignment = 4096;

   y_meta_stride = VENUS_Y_META_STRIDE(color_format, width);
   y_meta_height = VENUS_Y_META_SCANLINES(color_format, height);
   y_meta_size = ALIGN((y_meta_stride * y_meta_height), alignment);

   y_stride = VENUS_Y_STRIDE(color_format, width);
   y_height = VENUS_Y_SCANLINES(color_format, height);
   y_size = ALIGN((y_stride * y_height), alignment);

   c_meta_stride = VENUS_UV_META_STRIDE(color_format, width);
   c_meta_height = VENUS_UV_META_SCANLINES(color_format, height);
   c_meta_size = ALIGN((c_meta_stride * c_meta_height), alignment);

   buf_lyt->num_planes = DUAL_PLANES;

   buf_lyt->planes[0].top_left = buf_lyt->planes[0].offset = y_meta_size;
   buf_lyt->planes[1].top_left = buf_lyt->planes[1].offset = y_meta_size + y_size + c_meta_size;
   buf_lyt->planes[2].top_left = buf_lyt->planes[2].offset = y_meta_size + y_size + c_meta_size + 1;
   buf_lyt->planes[0].v_increment = y_stride;
   buf_lyt->planes[1].v_increment = VENUS_UV_STRIDE(color_format, width);
}



int msmgbm_yuv_plane_info(struct gbm_bo *gbo,generic_buf_layout_t *buf_lyt){
    struct msmgbm_bo *msm_gbm_bo = to_msmgbm_bo(gbo);
    int res = GBM_ERROR_NONE;

    if(!msm_gbm_bo || !buf_lyt)
        return GBM_ERROR_BAD_HANDLE;

     switch(gbo->format){
       //Semiplanar
        case GBM_FORMAT_YCbCr_420_SP:
        case GBM_FORMAT_YCrCb_420_SP:
        case GBM_FORMAT_YCbCr_420_SP_VENUS:
        case GBM_FORMAT_NV12:
        case GBM_FORMAT_NV12_ENCODEABLE: //Same as YCbCr_420_SP_VENUS
             get_yuv_sp_plane_info(gbo->aligned_width, gbo->aligned_height,
                                   YUV_420_SP_BPP, buf_lyt);
             break;
        case GBM_FORMAT_YCbCr_420_TP10_UBWC:
             get_yuv_ubwc_sp_plane_info(gbo->aligned_width, gbo->aligned_height,
                                        COLOR_FMT_NV12_BPP10_UBWC, buf_lyt);
             break;
        case GBM_FORMAT_YCbCr_420_P010_UBWC:
             get_yuv_ubwc_sp_plane_info(gbo->aligned_width, gbo->aligned_height,
                                        COLOR_FMT_P010_UBWC, buf_lyt);
             break;
        default:
             res = GBM_ERROR_UNSUPPORTED;
             break;
     }

    return res;
}

void msmgbm_log_hdr_color_info_mdata(ColorMetaData * color_mdata)
{
    uint8_t i = 0;
    uint8_t j = 0;

    LOG(LOG_DBG,"setMetaData COLOR_METADATA : color_primaries = 0x%x,"
                "range = 0x%x, transfer = 0x%x, matrix = 0x%x",
                 color_mdata->colorPrimaries, color_mdata->range,
                 color_mdata->transfer, color_mdata->matrixCoefficients);

    for(i = 0; i < 3; i++) {
        for(j = 0; j < 2; j++) {
            LOG(LOG_DBG,"setMetadata COLOR_METADATA : rgb_primaries[%d][%d] = 0x%x",
                i, j, color_mdata->masteringDisplayInfo.primaries.rgbPrimaries[i][j]);
        }
    }

    LOG(LOG_DBG,"setMetadata COLOR_METADATA : white_point[0] = 0x%x white_point[1] = 0x%x",
                    color_mdata->masteringDisplayInfo.primaries.whitePoint[0],
                    color_mdata->masteringDisplayInfo.primaries.whitePoint[1]);

    LOG(LOG_DBG,"setMetadata COLOR_METADATA : max_disp_lum = 0x%x min_disp_lum = 0x%x",
                    color_mdata->masteringDisplayInfo.maxDisplayLuminance,
                    color_mdata->masteringDisplayInfo.minDisplayLuminance);

    LOG(LOG_DBG,"setMetadata COLOR_METADATA : max_cll = 0x%x min_pall = 0x%x",
                    color_mdata->contentLightLevel.maxContentLightLevel,
                    color_mdata->contentLightLevel.minPicAverageLightLevel);

}


void msmsgbm_default_init_hdr_color_info_mdata(ColorMetaData * color_mdata)
{
    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t k = 0;

    color_mdata->colorPrimaries      = 0xAB;
    color_mdata->range               = 0xCD;
    color_mdata->transfer            = 0xEF;
    color_mdata->matrixCoefficients  = 0xDE;

    for(i = 0, k = 0xAE; i < 3; i++) {
        for(j = 0; j < 2; j++, k++)
            color_mdata->masteringDisplayInfo.primaries.rgbPrimaries[i][j] =(i+j+k);
    }

    color_mdata->masteringDisplayInfo.primaries.whitePoint[0]   = 0xFA;
    color_mdata->masteringDisplayInfo.primaries.whitePoint[1]   = 0xFB;
    color_mdata->masteringDisplayInfo.maxDisplayLuminance   = 0xABCEDF00;
    color_mdata->masteringDisplayInfo.minDisplayLuminance   = 0xFABADEEF;
    color_mdata->contentLightLevel.maxContentLightLevel     = 0xDAA0BAAC;
    color_mdata->contentLightLevel.minPicAverageLightLevel  = 0xFAB0C007;

}



int msmgbm_get_buf_lyout(struct gbm_bo *gbo, generic_buf_layout_t *buf_lyt)
{
    struct msmgbm_bo *msm_gbm_bo = to_msmgbm_bo(gbo);
    int res = GBM_ERROR_NONE;
    int Bpp;

    if(!gbo || !buf_lyt)
        return GBM_ERROR_BAD_HANDLE;

    if(gbo->width  <= 0 || gbo->height <= 0){
        LOG(LOG_ERR,"INVALID width or height\n");
        return NULL;
    }

    if(1 == IsFormatSupported(gbo->format))
        Bpp = GetFormatBpp(gbo->format);
    else
    {
        LOG(LOG_ERR,"Format (0x%x) not supported\n",gbo->format);
        return NULL;
    }

    buf_lyt->pixel_format = gbo->format;

    if(is_format_rgb(gbo->format))
    {
        buf_lyt->num_planes = 1;
        buf_lyt->planes[0].aligned_width = gbo->aligned_width;
        buf_lyt->planes[0].aligned_height = gbo->aligned_height;
        buf_lyt->planes[0].top_left = buf_lyt->planes[0].offset = 0;
        buf_lyt->planes[0].bits_per_component = Bpp;
        buf_lyt->planes[0].v_increment = ((gbo->aligned_width)*Bpp); //stride
    }
    else
    {
        switch(gbo->format){
           //Semiplanar
            case GBM_FORMAT_YCbCr_420_SP:
            case GBM_FORMAT_YCrCb_420_SP:
            case GBM_FORMAT_YCbCr_420_SP_VENUS:
            case GBM_FORMAT_NV12:
            case GBM_FORMAT_NV12_ENCODEABLE: //Same as YCbCr_420_SP_VENUS
                 get_yuv_sp_plane_info(gbo->aligned_width, gbo->aligned_height,
                                       YUV_420_SP_BPP, buf_lyt);
                 break;
            case GBM_FORMAT_YCbCr_420_TP10_UBWC:
                 get_yuv_ubwc_sp_plane_info(gbo->aligned_width, gbo->aligned_height,
                                            COLOR_FMT_NV12_BPP10_UBWC, buf_lyt);
                 break;
            case GBM_FORMAT_YCbCr_420_P010_UBWC:
                 get_yuv_ubwc_sp_plane_info(gbo->aligned_width, gbo->aligned_height,
                                            COLOR_FMT_P010_UBWC, buf_lyt);
                 break;
            default:
                 res = GBM_ERROR_UNSUPPORTED;
                 break;
        }
    }
    return res;
}

//File read for debug level configuration
void config_dbg_lvl(void)
{
    FILE *fp = NULL;

    fp = fopen("/data/misc/display/gbm_dbg_cfg.txt", "r");
    if(fp) {
        fscanf(fp, "%d", &g_debug_level);
        LOG(LOG_INFO,"\nGBM debug level set=%d\n",g_debug_level);
        fclose(fp);
    }
}

//helper function to get timestamp in usec
void get_time_in_usec(long long int *time_usec)
{
  struct timeval timer_usec;
  long long int timestamp_usec; /* timestamp in microsecond */
  if (!gettimeofday(&timer_usec, NULL)) {
    timestamp_usec = ((long long int) timer_usec.tv_sec) * 1000000ll +
                        (long long int) timer_usec.tv_usec;
  }
  else {
    timestamp_usec = -1;
  }
  printf("%lld microseconds since epoch\n", timestamp_usec);

  *time_usec = timestamp_usec;
}


int msmgbm_bo_dump(struct gbm_bo * gbo)
{
    FILE *fptr = NULL;
    static int count = 1;
    const char file_nme[100] = "/data/misc/display/gbm_dump";
    struct msmgbm_bo *msm_gbm_bo = to_msmgbm_bo(gbo);
    int mappedNow = 0;
    size_t size = gbo->size;
    int ret = GBM_ERROR_NONE;
    char tmp_str[50];
    long long int time_usec;
    uint32_t width = gbo->width;
    uint32_t height = gbo->height;
    uint32_t format = gbo->format;
    int ion_fd = gbo->ion_fd;

    //Dump Files are created per dump call reference
    //Get time in usec from system
    get_time_in_usec(&time_usec);

    //sprintf(tmp_str, "%d", count++);
    sprintf(tmp_str, "__%d_%d_%d_%d_%d_%lld", getpid(),ion_fd,width,height,format,time_usec);
    strcat(file_nme,tmp_str);
    strcat(file_nme,".dat");

    fptr=fopen(file_nme, "w+");
    if(fptr == NULL)
    {
        LOG(LOG_ERR,"Failed to open file %s\n",file_nme);
        return GBM_ERROR_BAD_HANDLE;
    }

    if(msm_gbm_bo->cpuaddr == NULL)
    {
        if(msmgbm_bo_cpu_map(gbo) == NULL){
             LOG(LOG_ERR,"Unable to Map to CPU, cannot write to BO\n");
             if(fptr)
                fclose(fptr);
             return GBM_ERROR_BAD_HANDLE;
        }
        mappedNow =1;
    }

    //Read from BO and write to file
    ret = fwrite(msm_gbm_bo->cpuaddr, 1, size, fptr);
    if(ret != size)
    {
        LOG(LOG_ERR,"File write size mismatch i/p=%d o/p=%d\n %s\n",size,ret,strerror(errno));
        ret = GBM_ERROR_BAD_VALUE;
    }else
        ret = GBM_ERROR_NONE;

    if(mappedNow){ //Unmap BO, if we mapped it.
        msmgbm_bo_cpu_unmap(gbo);
    }

    if(fptr)
      fclose(fptr);

    return ret;
}
