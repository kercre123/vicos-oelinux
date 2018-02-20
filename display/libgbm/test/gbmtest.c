//**************************************************************************************************
// Copyright (c) 2017 Qualcomm Technologies, Inc.
// All Rights Reserved.
// Confidential and Proprietary - Qualcomm Technologies, Inc.
//**************************************************************************************************
/*
 * Copyright 2014 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */
#include <assert.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <gbm.h>
#include <gbm_priv.h>
#include <wayland-server.h>

#define CHECK(cond) do {\
    if (!(cond)) {\
        printf("CHECK failed in %s() %s:%d\n", __func__, __FILE__, __LINE__);\
        return 0;\
    }\
} while(0)

#define ARRAY_SIZE(A) (sizeof(A)/sizeof(*(A)))

#define ENODRM     -1
#define ENODISPLAY -2
#define NUM_BACK_BUFFERS 3
#define MAX_BUFFER  4
#define MAX_REPEAT  4

static int fd;
static struct gbm_device *gbm;

static const uint32_t format_list[] = {
    GBM_FORMAT_RGB565,
    GBM_FORMAT_RGB888,
    GBM_FORMAT_XRGB8888,
    GBM_FORMAT_ARGB8888,
    GBM_FORMAT_RGBX8888,
    GBM_FORMAT_RGBA8888,
    GBM_FORMAT_BGRA8888,
    GBM_FORMAT_XBGR8888,
    GBM_FORMAT_ABGR8888,
    GBM_FORMAT_NV12,
    GBM_FORMAT_BGR565,
    GBM_FORMAT_ABGR2101010,
    GBM_FORMAT_YCbCr_420_TP10_UBWC,
    GBM_FORMAT_P010,
};

static const uint32_t usage_list[] = {
    GBM_BO_USE_SCANOUT,
    GBM_BO_USE_CURSOR_64X64,
    GBM_BO_USE_RENDERING,
    GBM_BO_USE_WRITE,
};

/*
 * Tests user data.
 */
static int been_there1;
static int been_there2;
static int check_bo(struct gbm_bo *bo)
{
    CHECK(bo);
    CHECK(gbm_bo_get_width(bo) >= 0);
    CHECK(gbm_bo_get_height(bo) >= 0);
    CHECK(gbm_bo_get_stride(bo) >= gbm_bo_get_width(bo));

    return 1;
}

/*
 * Tests initialization.
 */
static int test_init()
{
    fd = open("/dev/dri/card0", O_RDWR | O_CLOEXEC);
    if (fd < 0)
    if (fd == ENODISPLAY)
        return ENODISPLAY;
    CHECK(fd >= 0);

    gbm = gbm_create_device(fd);

    CHECK(gbm_device_get_fd(gbm) == fd);

    const char* backend_name = gbm_device_get_backend_name(gbm);

    CHECK(backend_name);

    printf("[ test_init(): gbm_create_device(),gbm_device_get_fd] success\n");

    return 1;
}

/*
 * Tests reinitialization.
 */
static int test_reinit()
{
    char *buf=NULL;
    char *readbuf=NULL;
    char  **readbufPtr=&readbuf;
    int size=1024;
    int i=0;
    char j=0;
    int ret=GBM_ERROR_NONE;
    int data_cmp_flg=0;
    unsigned int intr_lce_set=0xABCD;
    unsigned int intr_lce_get=0;
    int mgc_id=0;
    void *prm;
    unsigned int bo_hdl=0;
    unsigned int imp_bo_hdl=0;
    struct gbm_bo *bo=NULL;
    struct gbm_bo *imp_bo=NULL;


    gbm_device_destroy(gbm);
    close(fd);


    fd = open("/dev/dri/card0", O_RDWR | O_CLOEXEC);
    CHECK(fd >= 0);

    //Allocate a buffer
    buf=malloc(size);

    //Write to the buffer
    for(i=0,j=0;i<size;i++,j++)
        buf[i]=j;

    gbm = gbm_create_device(fd);

    CHECK(gbm_device_get_fd(gbm) == fd);

    printf("[ test_reinit(): gbm_create_device(),gbm_device_get_fd] success\n");



    bo = gbm_bo_create(gbm, 1024, 1024, GBM_FORMAT_XRGB8888, GBM_BO_USE_RENDERING);
    CHECK(check_bo(bo));

    //Write to the gbm bo
    bo->usage_flags|=GBM_BO_USE_WRITE;

    ret=bo->bo_write(bo,buf,size);
    if(ret < 0)
        printf("BO write failed\n");
    else
        printf("BO write is successful\n");

    ret=gbm_perform(GBM_PERFORM_CPU_MAP_FOR_BO,bo,readbufPtr);
    if(ret==GBM_ERROR_NONE)
    {
        printf("BO CPU Map Success\n");

        for(i=0,j=0;i<size;i++,j++)
            if(readbuf[i]!=j){
                printf("Data Mismatch @ index=%d\n",i);
                data_cmp_flg=1;
            }

        if(!data_cmp_flg)
            printf("Data matched successfully\n");
    }else
        printf("BO CPU Map Failed\n");

    //Testing Device Magic Authentication
    ret=gbm_perform(GBM_PERFORM_GET_DRM_DEVICE_MAGIC,gbm,&mgc_id);
    if(ret==GBM_ERROR_NONE)
        printf("Get DRM Magic ID Success Magic ID=%u\n",mgc_id);
    else
        printf("Get DRM Magic ID Failed\n");


    ret=gbm_perform(GBM_PERFORM_AUTH_DRM_DEVICE_MAGIC,gbm,mgc_id);
    if(ret==GBM_ERROR_NONE)
        printf("Auth DRM Magic ID Success\n");
    else
        printf("Auth DRM Magic ID Failed\n");

    prm=(void *)&intr_lce_set;
    //Testing Set and Get Metadata
    printf("Set param addr=%x , value=%u\n",prm,*(unsigned int *)prm);
    ret=gbm_perform(GBM_PERFORM_SET_METADATA,bo,GBM_METADATA_SET_INTERLACED,prm);
    if(ret==GBM_ERROR_NONE)
        printf("Set Metadata Success\n");
    else
        printf("Set Metadata Failed\n");

    prm=(void *)&intr_lce_get;
    ret=gbm_perform(GBM_PERFORM_GET_METADATA,bo,GBM_METADATA_GET_INTERLACED,prm);
    if(ret==GBM_ERROR_NONE) {
        printf("Get Metadata Success\n");
        if(intr_lce_get!=intr_lce_set)
            printf("Metdata, set-get mismatch Set=%d, Get=%x\n",intr_lce_set,intr_lce_get);
        else
            printf("Metdata-validate, set-get Matched => %x\n",intr_lce_get);
    }
    else
        printf("Get Metadata Failed\n");


   //Query gbm bo handle to compare with import handle
    bo_hdl=gbm_bo_get_handle(bo).u32;
    printf("Obtained bo handle=%x\n",bo_hdl);

    imp_bo=gbm_bo_import(gbm,GBM_BO_IMPORT_FD,bo->ion_fd,0);
    if(imp_bo!=NULL)
    {
        imp_bo_hdl=gbm_bo_get_handle(imp_bo).u32;
        if(imp_bo_hdl!=bo_hdl)
            printf("BO Import handle does not match with current BO handle");
        else
            printf("Import and current BO handle match %x\n",imp_bo_hdl);
    }else
        printf("BO Import failed");

    ret=gbm_perform(GBM_PERFORM_CPU_UNMAP_FOR_BO,bo);
    if(ret!=GBM_ERROR_NONE)
        printf("BO CPU UNMap Failed\n");
    else
        printf("BO CPU UNMap Success\n");

    gbm_bo_destroy(bo);

    return 1;
}

/*
 * Validate Set/Get Metdadta
 */
static int test_metadata()
{
    char *buf=NULL;
    char *readbuf=NULL;
    char  **readbufPtr=&readbuf;
    int size=1024;
    int i=0;
    char j=0;
    int ret=GBM_ERROR_NONE;
    int data_cmp_flg=0;
    unsigned int intr_lce_set=0xABCD;
    unsigned int intr_lce_get=0;
    int mgc_id=0;
    void *prm;
    unsigned int bo_hdl=0;
    unsigned int imp_bo_hdl=0;
    struct gbm_bo *bo=NULL;
    struct gbm_bo *imp_bo=NULL;
    uint64_t  gu_addr =0xABCDEF;
    uint64_t  gu_get_addr =0;

    fd = open("/dev/dri/card0", O_RDWR | O_CLOEXEC);
    CHECK(fd >= 0);

    //Allocate a buffer
    buf=malloc(size);

    //Write to the buffer
    for(i=0,j=0;i<size;i++,j++)
        buf[i]=j;

    gbm = gbm_create_device(fd);

    CHECK(gbm_device_get_fd(gbm) == fd);

    printf("[ test_reinit(): gbm_create_device(),gbm_device_get_fd] success\n");



    bo = gbm_bo_create(gbm, 1024, 1024, GBM_FORMAT_XRGB8888, GBM_BO_USE_RENDERING);
    CHECK(check_bo(bo));

    prm=(void *)&intr_lce_set;
    //Testing Set and Get Metadata
    printf("Set param addr=%x , value=%u\n",prm,*(unsigned int *)prm);
    ret=gbm_perform(GBM_PERFORM_SET_METADATA,bo,GBM_METADATA_SET_INTERLACED,prm);
    if(ret==GBM_ERROR_NONE)
        printf("Set Metadata Success\n");
    else {
        printf("Set Metadata Failed\n");
        return 0;
    }

    //Testing Set and Get GPU ADDR
    printf("Set gu_addr =%x\n",gu_addr);
    ret=gbm_perform(GBM_PERFORM_SET_GPU_ADDR_FOR_BO,bo,gu_addr);
    if(ret==GBM_ERROR_NONE)
        printf("Set GPU Addr Success\n");
    else {
        printf("Set GPU Addr Failed\n");
        return 0;
    }


    //Testing Set and Get GPU ADDR
    printf("Get gu_addr =%x\n",gu_get_addr);
    ret=gbm_perform(GBM_PERFORM_GET_GPU_ADDR_FOR_BO,bo,&gu_get_addr);
    if(ret==GBM_ERROR_NONE)
        printf("Get GPU Addr Success GPU Addr =%x\n",gu_get_addr);
    else {
        printf("Get GPU Addr Failed\n");
        return 0;
    }

    prm=(void *)&intr_lce_get;
    ret=gbm_perform(GBM_PERFORM_GET_METADATA,bo,GBM_METADATA_GET_INTERLACED,prm);
    if(ret==GBM_ERROR_NONE) {
        printf("Get Metadata Success\n");
        if(intr_lce_get!=intr_lce_set)
            printf("Metdata, set-get mismatch Set=%d, Get=%x\n",intr_lce_set,intr_lce_get);
        else
            printf("Metdata-validate, set-get Matched => %x\n",intr_lce_get);
    }
    else {
        printf("Get Metadata Failed\n");
        return 0;
    }

    gbm_bo_destroy(bo);

    gbm_device_destroy(gbm);
    close(fd);

    return 1;
}

/*
 * Tests Get and Auth DRM Magic
 */
static int test_drm_magic()
{
    char *buf=NULL;
    char *readbuf=NULL;
    char  **readbufPtr=&readbuf;
    int size=1024;
    int i=0;
    char j=0;
    int ret=GBM_ERROR_NONE;
    int data_cmp_flg=0;
    unsigned int intr_lce_set=0xABCD;
    unsigned int intr_lce_get=0;
    int mgc_id=0;
    void *prm;
    unsigned int bo_hdl=0;
    unsigned int imp_bo_hdl=0;
    struct gbm_bo *bo=NULL;
    struct gbm_bo *imp_bo=NULL;


    fd = open("/dev/dri/card0", O_RDWR | O_CLOEXEC);
    CHECK(fd >= 0);

    gbm = gbm_create_device(fd);

    CHECK(gbm_device_get_fd(gbm) == fd);

    printf("[ test_reinit(): gbm_create_device(),gbm_device_get_fd] success\n");

    //Testing Device Magic Authentication
    ret=gbm_perform(GBM_PERFORM_GET_DRM_DEVICE_MAGIC,gbm,&mgc_id);
    if(ret==GBM_ERROR_NONE)
        printf("Get DRM Magic ID Success Magic ID=%u\n",mgc_id);
    else {
        printf("Get DRM Magic ID Failed\n");
        return 0;
    }

    //Testing Device Magic Authentication
    ret=gbm_perform(GBM_PERFORM_VALIDATE_DEVICE,gbm);
    if(ret==GBM_ERROR_NONE)
        printf("Validate GBM Magic ID Success=%u\n",mgc_id);
    else {
        printf("Validate GBM Magic ID Failed\n");
        return 0;
    }


    ret=gbm_perform(GBM_PERFORM_AUTH_DRM_DEVICE_MAGIC,gbm,mgc_id);
    if(ret==GBM_ERROR_NONE)
        printf("Auth DRM Magic ID Success\n");
    else {
        printf("Auth DRM Magic ID Failed\n");
        return 0;
    }

    gbm_device_destroy(gbm);
    close(fd);

    return 1;
}

/*
 * Tests Write/Read of BO and Map/Unmap
 */
static int test_bo_write(int flag)
{
    char *buf=NULL;
    char *readbuf=NULL;
    char  **readbufPtr=&readbuf;
    int size=1024;
    int i=0;
    int k=0;
    int l=0;
    char j=0;
    int ret=GBM_ERROR_NONE;
    int data_cmp_flg=0;
    int cp_stat=0;
    size_t bo_size=0;
    int mgc_id=0;
    unsigned int name=0;
    int res=0;
    void *prm;
    unsigned int bo_hdl=0;
    unsigned int imp_bo_hdl=0;
    struct gbm_bo *bo[MAX_BUFFER]={NULL,};
    int tmp_ion_meta_fd[MAX_BUFFER]={0,};

    struct gbm_bo *imp_bo=NULL;


    fd = open("/dev/dri/card0", O_RDWR | O_CLOEXEC);
    CHECK(fd >= 0);

    //Allocate a buffer
    buf=malloc(size);

    //Write to the buffer
    for(i=0,j=0;i<size;i++,j++)
        buf[i]=j;

    gbm = gbm_create_device(fd);

    CHECK(gbm_device_get_fd(gbm) == fd);

    printf("[ test_reinit(): gbm_create_device(),gbm_device_get_fd] success\n");

    //Secure OR Non-secure buffer

    for(k=0;k<MAX_BUFFER;k++)
    {

        bo[k] = gbm_bo_create(gbm, 1080, 1920, GBM_FORMAT_XRGB8888,flag);
        CHECK(check_bo(bo[k]));

        //repetitievely call the perform API's
        for(l=0;l<MAX_REPEAT;l++)
        {

            //Read the global name
            ret=gbm_perform(GBM_PERFORM_GET_BO_NAME, bo[k], &name);
            if(ret == GBM_ERROR_NONE)
                printf("GET BO Name=%d success\n",name);
            else{
                printf("GET BO Name failed\n");
                return 0;
            }

            ret=gbm_perform(GBM_PERFORM_GET_BO_SIZE, bo[k], &bo_size);
            if(ret == GBM_ERROR_NONE)
                printf("GET BO size=%d success\n",bo_size);
            else{
                printf("GET BO size failed\n");
                return 0;
            }

            //Check for secure buffer or not
            prm=(void *)&cp_stat;
            ret=gbm_perform(GBM_PERFORM_GET_SECURE_BUFFER_STATUS,bo[k],prm);
            if(ret==GBM_ERROR_NONE) {
                printf("Get Secure Buffer stat Success\n");
                if(cp_stat!=false)
                    printf("GBM BO is a secure buffer\n");
                else
                    printf("GBM BO is a non-secure buffer\n");
            }
            else {
                printf("Get Metadata Failed\n");
                return 0;
            }
        }


        //Validate for invalid metadata fd
        tmp_ion_meta_fd[k]=bo[k]->ion_metadata_fd;
        bo[k]->ion_metadata_fd=-1;
        //repetitievely call the perform API's
        for(l=0;l<MAX_REPEAT;l++)
        {

            //Read the global name
            ret=gbm_perform(GBM_PERFORM_GET_BO_NAME, bo[k], &name);
            if(ret == GBM_ERROR_NONE)
                printf("GET BO Name=%d success\n",name);
            else{
                printf("GET BO Name failed\n");
                return 0;
            }

            ret=gbm_perform(GBM_PERFORM_GET_BO_SIZE, bo[k], &bo_size);
            if(ret == GBM_ERROR_NONE)
                printf("GET BO size=%d success\n",bo_size);
            else{
                printf("GET BO size failed\n");
                return 0;
            }

            //Check for secure buffer or not
            prm=(void *)&cp_stat;
            ret=gbm_perform(GBM_PERFORM_GET_SECURE_BUFFER_STATUS,bo[k],prm);
            if(ret==GBM_ERROR_NONE) {
                printf("Get Secure Buffer stat Success\n");
                if(cp_stat!=false)
                    printf("GBM BO is a secure buffer\n");
                else
                    printf("GBM BO is a non-secure buffer\n");
            }
            else {
                printf("Get Metadata Failed\n");
                return 0;
            }
        }


        //Write to the buffer
        for(i=0,j=k;i<size;i++,j++)
             buf[i]=j;

        //Write to the gbm bo
        bo[k]->usage_flags|=GBM_BO_USE_WRITE;

        ret=bo[k]->bo_write(bo[k],buf,size);
        if(ret < 0){
            printf("BO write failed\n");
            gbm_bo_destroy(bo[k]);
            res=0;
            goto end;
        }
        else
            printf("BO write is successful\n");
    }

    for(k=0;k<MAX_BUFFER;k++)
    {

        CHECK((gbm_perform(GBM_PERFORM_CPU_MAP_FOR_BO,bo[k],readbufPtr))==GBM_ERROR_NONE);
        printf("BO CPU map Success\n");

        for(i=0,j=k;i<size;i++,j++)
            if(readbuf[i]!=j){
                printf("Data Mismatch @ index=%d\n",i);
                data_cmp_flg=1;
            }

        CHECK(data_cmp_flg==0);
        printf("Data matched successfully\n");

        CHECK((gbm_perform(GBM_PERFORM_CPU_UNMAP_FOR_BO,bo[k]))==GBM_ERROR_NONE);
        printf("BO CPU unmap Success\n");

        //Dump hash map table
        ret=gbm_perform(GBM_PERFORM_DUMP_HASH_MAP);
        if(ret == GBM_ERROR_NONE)
            printf("Hash map dump success\n");
        else{
            printf("Hash map dump failed\n");
            return 0;
        }

        //Dump BO data
        ret=gbm_perform(GBM_PERFORM_DUMP_BO_CONTENT, bo[k]);
        if(ret == GBM_ERROR_NONE)
            printf("Dump BO content Success\n");
        else{
            printf("Dump BO content failed\n");
            return 0;
        }

        gbm_bo_destroy(bo[k]);

    }

    //successful completion of the Data check loop
    res=1;

end:

    gbm_device_destroy(gbm);
    close(fd);

    if(buf)
        free(buf);

    return res;
}


/*
 * Tests Write/Read of BO and Map/Unmap
 */
static int test_bo_write_secure(int flag)
{
    char *buf=NULL;
    char *readbuf=NULL;
    char  **readbufPtr=&readbuf;
    int size=1024;
    int i=0;
    int k=0;
    int l=0;
    char j=0;
    int ret=GBM_ERROR_NONE;
    int data_cmp_flg=0;
    int cp_stat=0;
    size_t bo_size=0;
    int mgc_id=0;
    unsigned int name=0;
    int res=0;
    void *prm;
    unsigned int bo_hdl=0;
    unsigned int imp_bo_hdl=0;
    struct gbm_bo *bo[MAX_BUFFER]={NULL,};
    int tmp_ion_meta_fd[MAX_BUFFER]={0,};

    struct gbm_bo *imp_bo=NULL;


    fd = open("/dev/dri/card0", O_RDWR | O_CLOEXEC);
    CHECK(fd >= 0);

    //Allocate a buffer
    buf=malloc(size);

    //Write to the buffer
    for(i=0,j=0;i<size;i++,j++)
        buf[i]=j;

    gbm = gbm_create_device(fd);

    CHECK(gbm_device_get_fd(gbm) == fd);

    printf("[ test_reinit(): gbm_create_device(),gbm_device_get_fd] success\n");

    //Secure OR Non-secure buffer

    for(k=0;k<MAX_BUFFER;k++)
    {

        bo[k] = gbm_bo_create(gbm, 1080, 1920, GBM_FORMAT_XRGB8888,flag);
        CHECK(check_bo(bo[k]));


        //Validate for valid metadata fd
        //repetitievely call the perform API's
        for(l=0;l<MAX_REPEAT;l++)
        {

            //Read the global name
            ret=gbm_perform(GBM_PERFORM_GET_BO_NAME, bo[k], &name);
            if(ret == GBM_ERROR_NONE)
                printf("GET BO Name=%d success\n",name);
            else{
                printf("GET BO Name failed\n");
                return 0;
            }

            ret=gbm_perform(GBM_PERFORM_GET_BO_SIZE, bo[k], &bo_size);
            if(ret == GBM_ERROR_NONE)
                printf("GET BO size=%d success\n",bo_size);
            else{
                printf("GET BO size failed\n");
                return 0;
            }

            //Check for secure buffer or not
            prm=(void *)&cp_stat;
            ret=gbm_perform(GBM_PERFORM_GET_SECURE_BUFFER_STATUS,bo[k],prm);
            if(ret==GBM_ERROR_NONE) {
                printf("Get Secure Buffer stat Success\n");
                if(cp_stat!=false)
                    printf("GBM BO is a secure buffer\n");
                else
                    printf("GBM BO is a non-secure buffer\n");
            }
            else {
                printf("Get Metadata Failed\n");
                return 0;
            }
        }


        //Validate for invalid metadata fd
        tmp_ion_meta_fd[k]=bo[k]->ion_metadata_fd;
        bo[k]->ion_metadata_fd=-1;
        //repetitievely call the perform API's
        for(l=0;l<MAX_REPEAT;l++)
        {

            //Read the global name
            ret=gbm_perform(GBM_PERFORM_GET_BO_NAME, bo[k], &name);
            if(ret == GBM_ERROR_NONE)
                printf("GET BO Name=%d success\n",name);
            else{
                printf("GET BO Name failed\n");
                return 0;
            }

            ret=gbm_perform(GBM_PERFORM_GET_BO_SIZE, bo[k], &bo_size);
            if(ret == GBM_ERROR_NONE)
                printf("GET BO size=%d success\n",bo_size);
            else{
                printf("GET BO size failed\n");
                return 0;
            }

            //Check for secure buffer or not
            prm=(void *)&cp_stat;
            ret=gbm_perform(GBM_PERFORM_GET_SECURE_BUFFER_STATUS,bo[k],prm);
            if(ret==GBM_ERROR_NONE) {
                printf("Get Secure Buffer stat Success\n");
                if(cp_stat!=false)
                    printf("GBM BO is a secure buffer\n");
                else
                    printf("GBM BO is a non-secure buffer\n");
            }
            else {
                printf("Get Metadata Failed\n");
                return 0;
            }
        }


        //Write to the buffer
        for(i=0,j=k;i<size;i++,j++)
             buf[i]=j;

        //Write to the gbm bo
        bo[k]->usage_flags|=GBM_BO_USE_WRITE;

        ret=bo[k]->bo_write(bo[k],buf,size);
        if(ret < 0){
            printf("Writes are not valid for a secure buffer\n");
        }
        else
            printf("BO write is successful\n");

        gbm_bo_destroy(bo[k]);
    }

    //successful completion of the Data check loop
    res=1;

end:

    gbm_device_destroy(gbm);
    close(fd);

    if(buf)
        free(buf);

    return res;
}



/*
 * Tests repeated alloc/free.
 */
static int test_alloc_free()
{
    int i;
    for(i = 0; i < 1000; i++) {
        struct gbm_bo *bo;
        printf("test_alloc_free run(%d)\n",i);
        bo = gbm_bo_create(gbm, 1024, 1024, GBM_FORMAT_XRGB8888, GBM_BO_USE_RENDERING);
        CHECK(check_bo(bo));
        gbm_bo_destroy(bo);
    }
    return 1;
}

/*
 * Tests repeated alloc/free.
 */
static int test_alloc_free_validate_meta_fd()
{
    int i;
    struct gbm_bo *gb_bo[1000];
    int ret;
    int meta_fd;

    for(i = 0; i < 2; i++) {
        printf("test_alloc_free run(%d)\n",i);
        gb_bo[i] = gbm_bo_create(gbm, 1024, 1024, GBM_FORMAT_XRGB8888, GBM_BO_USE_RENDERING);
        CHECK(check_bo(gb_bo[i]));

        ret=gbm_perform(GBM_PERFORM_GET_METADATA_ION_FD, gb_bo[i], &meta_fd);
        if(ret == GBM_ERROR_NONE)
            printf("GET BO Metadata fd=%d success\n",meta_fd);
        else{
            printf("GET BO Metadata fd failed\n");
            return 0;
        }

    }
    for(i = 0; i < 2; i++)
        gbm_bo_destroy(gb_bo[i]);

    return 1;
}

/*
 * Tests repeated alloc/free with
 *  aligned width and height.
 */
static int test_get_aligned_width_height()
{
    int i;
    struct gbm_bo *gb_bo[1000];
    int ret;
    int meta_fd;
    uint32_t align_wdth;
    uint32_t align_hght;

    for(i = 0; i < 2; i++) {
        printf("test_alloc_free run(%d)\n",i);
        gb_bo[i] = gbm_bo_create(gbm, 1024, 1024, GBM_FORMAT_XRGB8888, GBM_BO_USE_RENDERING);
        CHECK(check_bo(gb_bo[i]));

        ret=gbm_perform(GBM_PERFORM_GET_BO_ALIGNED_WIDTH, gb_bo[i], &align_wdth);
        if(ret == GBM_ERROR_NONE)
            printf("GET BO Aligned width=%d success\n",align_wdth);
        else{
            printf("GET BO Aligned width failed\n");
            return 0;
        }

        ret=gbm_perform(GBM_PERFORM_GET_BO_ALIGNED_HEIGHT, gb_bo[i], &align_hght);
        if(ret == GBM_ERROR_NONE)
            printf("GET BO Aligned height=%d success\n",align_hght);
        else{
            printf("GET BO Aligned height failed\n");
            return 0;
        }


    }

    for(i = 0; i < 2; i++)
        gbm_bo_destroy(gb_bo[i]);

    return 1;
}

#define MAX_FORMAT_STRING_SIZE 50
struct gbm_image_format_list {
     char gbm_format_str[MAX_FORMAT_STRING_SIZE];
     int  gbm_format;
     char gbm_usage_flag_str[MAX_FORMAT_STRING_SIZE];
     int  gbm_usage_flag;
     int  expected_test_result;
};
/*
 * Tests repeated alloc/free with
 *  querying UBWC status
 */
static int test_get_ubwc_status()
{
    int i, j;
    int num_tests = 0;
    char *test_result_buf, *buf;
    char expected_result[10], test_result[10];
    char *failure = "FAIL";
    char *success = "PASS";
    struct gbm_bo *gb_bo;
    int ret;
    int meta_fd;
    int ubwc_status;
    bool ubwc_test_status = true;
    void *prm;

    static const struct gbm_image_format_list gbm_format_test[] =
           {
             {"GBM_FORMAT_XBGR8888", GBM_FORMAT_XBGR8888, "GBM_BO_USAGE_UBWC_ALIGNED_QTI",
                                                          GBM_BO_USAGE_UBWC_ALIGNED_QTI, 1},
             {"GBM_FORMAT_XBGR8888", GBM_FORMAT_XBGR8888, "GBM_BO_USAGE_HW_RENDERING_QTI",
                                                          GBM_BO_USAGE_HW_RENDERING_QTI, 0},
             {"GBM_FORMAT_XBGR8888", GBM_FORMAT_XBGR8888, "GBM_BO_USE_RENDERING         ",
                                                                   GBM_BO_USE_RENDERING, 0},
             {"GBM_FORMAT_XBGR8888", GBM_FORMAT_XBGR8888, "GBM_BO_USE_WRITE             ",
                                                                       GBM_BO_USE_WRITE, 0},

             {"GBM_FORMAT_ABGR8888", GBM_FORMAT_ABGR8888, "GBM_BO_USAGE_UBWC_ALIGNED_QTI",
                                                          GBM_BO_USAGE_UBWC_ALIGNED_QTI, 1},
             {"GBM_FORMAT_ABGR8888", GBM_FORMAT_ABGR8888, "GBM_BO_USAGE_HW_RENDERING_QTI",
                                                          GBM_BO_USAGE_HW_RENDERING_QTI, 0},
             {"GBM_FORMAT_ABGR8888", GBM_FORMAT_ABGR8888, "GBM_BO_USE_RENDERING         ",
                                                                   GBM_BO_USE_RENDERING, 0},
             {"GBM_FORMAT_ABGR8888", GBM_FORMAT_ABGR8888, "GBM_BO_USE_WRITE             ",
                                                                       GBM_BO_USE_WRITE, 0},

             {"GBM_FORMAT_BGR565", GBM_FORMAT_BGR565, "GBM_BO_USAGE_UBWC_ALIGNED_QTI",
                                                          GBM_BO_USAGE_UBWC_ALIGNED_QTI, 1},
             {"GBM_FORMAT_BGR565", GBM_FORMAT_BGR565, "GBM_BO_USAGE_HW_RENDERING_QTI",
                                                          GBM_BO_USAGE_HW_RENDERING_QTI, 0},
             {"GBM_FORMAT_BGR565", GBM_FORMAT_BGR565, "GBM_BO_USE_RENDERING         ",
                                                                   GBM_BO_USE_RENDERING, 0},
             {"GBM_FORMAT_BGR565", GBM_FORMAT_BGR565, "GBM_BO_USE_WRITE             ",
                                                                       GBM_BO_USE_WRITE, 0},

             {"GBM_FORMAT_ABGR2101010", GBM_FORMAT_ABGR2101010,
                         "GBM_BO_USAGE_UBWC_ALIGNED_QTI", GBM_BO_USAGE_UBWC_ALIGNED_QTI, 1},
             {"GBM_FORMAT_ABGR2101010", GBM_FORMAT_ABGR2101010,
                         "GBM_BO_USAGE_HW_RENDERING_QTI", GBM_BO_USAGE_HW_RENDERING_QTI, 0},
             {"GBM_FORMAT_ABGR2101010", GBM_FORMAT_ABGR2101010,
                         "GBM_BO_USE_RENDERING         ", GBM_BO_USE_RENDERING, 0},
             {"GBM_FORMAT_ABGR2101010", GBM_FORMAT_ABGR2101010,
                         "GBM_BO_USE_WRITE             ", GBM_BO_USE_WRITE, 0},
             {"GBM_FORMAT_P010", GBM_FORMAT_P010,
                         "GBM_BO_USE_WRITE             ", GBM_BO_USE_WRITE, 0},
             {"GBM_FORMAT_NONE", -1, "GBM_BO_NONE", -1, 0},
           };

    /* count of testcases */
    unsigned int height = 1024, width = 1024;

    for (j = 0; 0 != strcmp(gbm_format_test[j].gbm_format_str, "GBM_FORMAT_NONE"); j++)
        num_tests++;

    test_result_buf = buf = (char *) malloc(num_tests * (sizeof(struct gbm_image_format_list) + 5));
    memset(buf, '\0', (num_tests * (sizeof(struct gbm_image_format_list) + 5)));

    sprintf(buf, "%stest_get_ubwc_status results:\n", buf);
    sprintf(buf, "%s=============================================================================\n", buf);
    sprintf(buf, "%sGBM Image Format\t\tUsage Flag\t\tExpected Result\tTestResult\n", buf);

    for (j = 0; j < num_tests; j++) {

         gb_bo = gbm_bo_create(gbm, width, height, gbm_format_test[j].gbm_format,
                                                   gbm_format_test[j].gbm_usage_flag);
         printf("Format: %s => width: %d height:%d stride:%d \n",
                                                    gbm_format_test[j].gbm_format_str,
                                                    gbm_bo_get_width(gb_bo),
                                                    gbm_bo_get_height(gb_bo),
                                                    gbm_bo_get_stride(gb_bo));
         CHECK(check_bo(gb_bo));

         prm=(void *)&ubwc_status;
         strcpy(expected_result, (gbm_format_test[j].expected_test_result? success: failure));

         ret=gbm_perform(GBM_PERFORM_GET_UBWC_STATUS,gb_bo, prm);

         strcpy(test_result, ubwc_status? success: failure);

         sprintf(buf, "%s%s\t%s\t%s\t\t%s\n", buf, gbm_format_test[j].gbm_format_str,
                                                 gbm_format_test[j].gbm_usage_flag_str,
                                                 expected_result, test_result);
         if(ret==GBM_ERROR_NONE) {
             printf("UBWC status for allocated BO for %s is: %d \n",
                            gbm_format_test[j].gbm_format_str, ubwc_status);
             if (0 != strcmp(test_result, expected_result))
                 ubwc_test_status = false;

             if(ubwc_status) {
               printf("Allocated GBM BO format is UWBC for %s\n", gbm_format_test[j].gbm_format_str);
             }
             else {
               printf("Allocated GBM BO format is not UBWC for %s\n", gbm_format_test[j].gbm_format_str);
             }
         } else {
             printf("UBWC test for %s has failed.\n", gbm_format_test[j].gbm_format_str);
             ubwc_test_status = false;
         }

         gbm_bo_destroy(gb_bo);
    }

    sprintf(buf, "%s\nOVERALL TEST RESULT: %s\n\n", buf, ubwc_test_status? success: failure);
    sprintf(buf, "%s=============================================================================\n", buf);
    sprintf(buf, "%s \0", buf);
    printf("%s", test_result_buf);
    free(test_result_buf);

    return (ubwc_test_status? 1:0);
}

/*
 * Tests repeated alloc/free.
 */
static int test_vns_alloc_free()
{
    int i,j;
    int ret;
    int tmp_fd;
    generic_buf_layout_t buf_lyt;

    for(i = 0; i < MAX_REPEAT; i++) {
        struct gbm_bo *bo;
        printf("----------------test_alloc_free run(%d)---------------------\n",i);


        //Secure and non-secure buffer
        if(i < (MAX_REPEAT/2))
        {
            printf("----------------Secured Buffer Test----------------------\n");
            bo = gbm_bo_create(gbm, 1024, 1024, GBM_FORMAT_NV12, GBM_BO_USAGE_PROTECTED_QTI);
        }
        else
        {
            printf("----------------Simple Buffer Test----------------------\n");
            bo = gbm_bo_create(gbm, 1024, 1024, GBM_FORMAT_NV12, GBM_BO_USE_RENDERING);
        }

        printf("test_vns_alloc_free BO width(%d)\n",gbm_bo_get_width(bo));
        printf("test_vns_alloc_free BO height(%d)\n",gbm_bo_get_height(bo));
        printf("test_vns_alloc_free BO stride(%d)\n",gbm_bo_get_stride(bo));
        printf("test_vns_alloc_free BO format(0x%x)\n",gbm_bo_get_format(bo));
        CHECK(check_bo(bo));

        tmp_fd = bo->ion_metadata_fd;

        //Toggle valid and invalid metadata fd
        if(i%2)
            bo->ion_metadata_fd = -1;

        ret=gbm_perform(GBM_PERFORM_GET_YUV_PLANE_INFO, bo, &buf_lyt);
        if(ret == GBM_ERROR_NONE){
            printf("GET YUV Info success\n");
            for(j=0;j<(buf_lyt.num_planes);j++){
                printf("plane[%d].h_increment=%d\n",j,buf_lyt.planes[j].h_increment);
                printf("plane[%d].v_increment=%d\n",j,buf_lyt.planes[j].v_increment);
                printf("plane[%d].offset=%p\n",j,buf_lyt.planes[j].offset);
            }
        }
        else{
            printf("GET YUV Info failed\n");
            return 0;
        }

        ret=gbm_perform(GBM_PERFORM_GET_PLANE_INFO, bo, &buf_lyt);
        if(ret == GBM_ERROR_NONE){
            printf("GET YUV Info success\n");
            for(j=0;j<(buf_lyt.num_planes);j++){
                printf("plane[%d].h_increment=%d\n",j,buf_lyt.planes[j].h_increment);
                printf("plane[%d].v_increment=%d\n",j,buf_lyt.planes[j].v_increment);
                printf("plane[%d].offset=%p\n",j,buf_lyt.planes[j].offset);
            }
        }
        else{
            printf("GET YUV Info failed\n");
            return 0;
        }

        //Retreive the valid metadata fd
        bo->ion_metadata_fd = tmp_fd;

        gbm_bo_destroy(bo);
    }
    return 1;
}

/*
 * Tests surface create/destroy along with other
 * surface operations
 */
static int test_surface_create_destroy()
{
    int  i=0;
    int  k=0;
    int  l=0;
    char j=0;
    char *readbuf=NULL;
    char  **readbufPtr=&readbuf;
    unsigned int set_width=1080;
    unsigned int set_height=1920;
    unsigned int set_fmt=GBM_FORMAT_XRGB8888;
    size_t       bo_size=0;
    char         *buf=NULL;
    struct gbm_surface* gbm_surf=NULL;

    for(i = 0; i < 1000; i++) {

        unsigned int width=0;
        unsigned int height=0;
        unsigned int fmt=0;
        int ret=-1;
        struct gbm_bo* pfree_bo=NULL;

        printf("Loop count %d\n",i);

        gbm_surf = gbm_surface_create(gbm, set_width, set_height, set_fmt, GBM_BO_USE_RENDERING);

        if(!gbm_surf){
            printf("Surface creation failed\n");
            return 0;
        }

         ret= gbm_perform(GBM_PERFORM_VALIDATE_SURFACE, gbm_surf);
         if(ret==GBM_ERROR_NONE)
             printf("Validate Surface Success \n");
         else {
             printf("Validate Surface Failed\n");
             return 0;
         }


        ret=gbm_perform(GBM_PERFORM_GET_SURFACE_WIDTH,gbm_surf,&width);
        if(ret==GBM_ERROR_NONE)
            printf("Get Surface Width Success \n");
        else {
            printf("Get Surface Width Failed\n");
            return 0;
        }

        ret=gbm_perform(GBM_PERFORM_GET_SURFACE_HEIGHT,gbm_surf,&height);
        if(ret==GBM_ERROR_NONE)
            printf("Get Surface Height Success \n");
        else {
            printf("Get Surface Height Failed\n");
            return 0;
        }

        ret=gbm_perform(GBM_PERFORM_GET_SURFACE_FORMAT,gbm_surf,&fmt);
        if(ret==GBM_ERROR_NONE)
            printf("Get Surface Format Success \n");
        else {
            printf("Get Surface Format Failed\n");
            return 0;
        }

        if(width!= set_width){
            printf("Surface width Mismatch\n",i);
            return 0;
        }
        if(height!= set_height){
            printf("Surface height Mismatch\n",i);
            return 0;
        }
        if(fmt!= set_fmt){
            printf("Surface Format Mismatch\n");
            return 0;
        }

        for(l=0;l<NUM_BACK_BUFFERS;l++)
        {

            //Get Surface free BO
            ret = gbm_perform(GBM_PERFORM_GET_SURFACE_FREE_BO, gbm_surf, &pfree_bo);
            if(ret==GBM_ERROR_NONE)
                printf("Get Surface Free BO Success \n");
            else {
                printf("Get Surface Free BO Failed\n");
                return 0;
            }

            gbm_perform(GBM_PERFORM_SET_SURFACE_FRONT_BO, gbm_surf,pfree_bo);
            if(ret==GBM_ERROR_NONE)
                printf("Set Surface Front BO Success \n");
            else {
                printf("Set Surface Front BO Failed\n");
                return 0;
            }

            ret=gbm_perform(GBM_PERFORM_GET_BO_SIZE, pfree_bo, &bo_size);
            if(ret == GBM_ERROR_NONE)
                printf("GET BO size=%d success\n",bo_size);
            else{
                printf("GET BO size failed\n");
                return 0;
            }

            //Allocate a buffer
            buf=malloc(bo_size);
            for(k=0;k<bo_size;k++,j++)
                buf[k]=j;

            //Write to the gbm bo
            pfree_bo->usage_flags|=GBM_BO_USE_WRITE;

            ret=pfree_bo->bo_write(pfree_bo,buf,bo_size);
            if(ret < 0){
                printf("BO write failed\n");
                goto fail_end;
            }
            else
                printf("BO write is successful\n");


            CHECK((gbm_perform(GBM_PERFORM_CPU_MAP_FOR_BO,pfree_bo,readbufPtr))==GBM_ERROR_NONE);
            printf("BO CPU map Success\n");

            for(k=0;k<bo_size;k++,j++)
                if(readbuf[k]!=j){
                    printf("Data Mismatch @ index=%d\n",k);
                    goto fail_end;
                }

            if(buf)
                free(buf);


            CHECK((gbm_perform(GBM_PERFORM_CPU_UNMAP_FOR_BO,pfree_bo))==GBM_ERROR_NONE);
            printf("BO CPU unmap Success\n");

        }

        gbm_surface_destroy(gbm_surf);

    }
    return 1;

fail_end:
        if(buf)
            free(buf);
        gbm_surface_destroy(gbm_surf);
        return 0;

}

/*
 * Tests surface perform UBWC status operation
 */
static int test_surface_ubwc_status()
{
  int j;
  int num_tests = 0;
  char *test_result_buf, *buf;
  char expected_result[10], test_result[10];
  char *failure = "FAIL";
  char *success = "PASS";
  int ret;
  int ubwc_status;
  bool ubwc_test_status = true;
  struct gbm_surface* gbm_surf=NULL;

  static const struct gbm_image_format_list gbm_format_test[] =
         {
           {"GBM_FORMAT_XBGR8888", GBM_FORMAT_XBGR8888, "GBM_BO_USAGE_UBWC_ALIGNED_QTI",
                                                        GBM_BO_USAGE_UBWC_ALIGNED_QTI, 1},
           {"GBM_FORMAT_XBGR8888", GBM_FORMAT_XBGR8888, "GBM_BO_USAGE_HW_RENDERING_QTI",
                                                        GBM_BO_USAGE_HW_RENDERING_QTI, 0},
           {"GBM_FORMAT_XBGR8888", GBM_FORMAT_XBGR8888, "GBM_BO_USE_RENDERING         ",
                                                                 GBM_BO_USE_RENDERING, 0},
           {"GBM_FORMAT_XBGR8888", GBM_FORMAT_XBGR8888, "GBM_BO_USE_WRITE             ",
                                                                     GBM_BO_USE_WRITE, 0},

           {"GBM_FORMAT_ABGR8888", GBM_FORMAT_ABGR8888, "GBM_BO_USAGE_UBWC_ALIGNED_QTI",
                                                        GBM_BO_USAGE_UBWC_ALIGNED_QTI, 1},
           {"GBM_FORMAT_ABGR8888", GBM_FORMAT_ABGR8888, "GBM_BO_USAGE_HW_RENDERING_QTI",
                                                        GBM_BO_USAGE_HW_RENDERING_QTI, 0},
           {"GBM_FORMAT_ABGR8888", GBM_FORMAT_ABGR8888, "GBM_BO_USE_RENDERING         ",
                                                                 GBM_BO_USE_RENDERING, 0},
           {"GBM_FORMAT_ABGR8888", GBM_FORMAT_ABGR8888, "GBM_BO_USE_WRITE             ",
                                                                     GBM_BO_USE_WRITE, 0},

           {"GBM_FORMAT_BGR565", GBM_FORMAT_BGR565, "GBM_BO_USAGE_UBWC_ALIGNED_QTI",
                                                        GBM_BO_USAGE_UBWC_ALIGNED_QTI, 1},
           {"GBM_FORMAT_BGR565", GBM_FORMAT_BGR565, "GBM_BO_USAGE_HW_RENDERING_QTI",
                                                        GBM_BO_USAGE_HW_RENDERING_QTI, 0},
           {"GBM_FORMAT_BGR565", GBM_FORMAT_BGR565, "GBM_BO_USE_RENDERING         ",
                                                                 GBM_BO_USE_RENDERING, 0},
           {"GBM_FORMAT_BGR565", GBM_FORMAT_BGR565, "GBM_BO_USE_WRITE             ",
                                                                     GBM_BO_USE_WRITE, 0},

           {"GBM_FORMAT_ABGR2101010", GBM_FORMAT_ABGR2101010,
                       "GBM_BO_USAGE_UBWC_ALIGNED_QTI", GBM_BO_USAGE_UBWC_ALIGNED_QTI, 1},
           {"GBM_FORMAT_ABGR2101010", GBM_FORMAT_ABGR2101010,
                       "GBM_BO_USAGE_HW_RENDERING_QTI", GBM_BO_USAGE_HW_RENDERING_QTI, 0},
           {"GBM_FORMAT_ABGR2101010", GBM_FORMAT_ABGR2101010,
                       "GBM_BO_USE_RENDERING         ", GBM_BO_USE_RENDERING, 0},
           {"GBM_FORMAT_ABGR2101010", GBM_FORMAT_ABGR2101010,
                       "GBM_BO_USE_WRITE             ", GBM_BO_USE_WRITE, 0},
           {"GBM_FORMAT_P010", GBM_FORMAT_P010,
                       "GBM_BO_USE_WRITE             ", GBM_BO_USE_WRITE, 0},
           {"GBM_FORMAT_NONE", -1, "GBM_BO_NONE", -1, 0},
         };

  /* count of testcases */
  unsigned int height = 1024, width = 1024;

  for (j = 0; 0 != strcmp(gbm_format_test[j].gbm_format_str, "GBM_FORMAT_NONE"); j++)
      num_tests++;

  test_result_buf = buf = (char *) malloc(num_tests * (sizeof(struct gbm_image_format_list) + 5));
  memset(buf, '\0', (num_tests * (sizeof(struct gbm_image_format_list) + 5)));

  sprintf(buf, "%stest_surface_ubwc_status results:\n", buf);
  sprintf(buf, "%s========================================================================\n", buf);
  sprintf(buf, "%sGBM Surface Format\t\tUsage Flag\t\tExpected Result\tTestResult\n", buf);

  for (j = 0; j < num_tests; j++) {

       gbm_surf = gbm_surface_create(gbm, width, height, gbm_format_test[j].gbm_format,
                                     gbm_format_test[j].gbm_usage_flag);

       strcpy(expected_result, (gbm_format_test[j].expected_test_result? success: failure));

       ret = gbm_perform(GBM_PERFORM_GET_SURFACE_UBWC_STATUS, gbm_surf, &ubwc_status);

       strcpy(test_result, ubwc_status? success: failure);

       sprintf(buf, "%s%s\t%s\t%s\t\t%s\n", buf, gbm_format_test[j].gbm_format_str,
                                               gbm_format_test[j].gbm_usage_flag_str,
                                               expected_result, test_result);
       if (ret == GBM_ERROR_NONE) {
           printf("UBWC status for Surface[%s]: %d \n", gbm_format_test[j].gbm_format_str,
                                                        ubwc_status);

           if (0 != strcmp(test_result, expected_result))
               ubwc_test_status = false;
       } else {
           printf("UBWC test for %s has failed.\n", gbm_format_test[j].gbm_format_str);
           ubwc_test_status = false;
       }

       gbm_surface_destroy(gbm_surf);
  }

  sprintf(buf, "%s\nOVERALL TEST RESULT: %s\n\n", buf, ubwc_test_status? success: failure);
  sprintf(buf, "%s========================================================================\n", buf);
  sprintf(buf, "%s \0", buf);
  printf("%s", test_result_buf);
  free(test_result_buf);

  return (ubwc_test_status? 1:0);

}

/*
 * Tests to validate gbm buf info from wl resource
 */
static int test_validate_gbmbuf_wlresource()
{
    struct wl_resource resource;
    struct gbm_buf_info buffer_info, validate_buffer_info;
    struct gbm_bo *bo;
    unsigned int res = 0;
    unsigned int gbm_ret;

    bo = gbm_bo_create(gbm, 1024, 1024, GBM_FORMAT_XRGB8888, GBM_BO_USE_RENDERING);

    buffer_info.width = bo->width;
    buffer_info.height = bo->height;
    buffer_info.format = bo->format;
    buffer_info.fd = bo->ion_fd;
    buffer_info.metadata_fd = bo->ion_metadata_fd;

    gbm_ret = gbm_perform(GBM_PERFORM_GET_GBM_BUF_INFO_FROM_WL_RESOURCE, &resource, &validate_buffer_info);

    if (gbm_ret == GBM_ERROR_NONE) {
        printf("validate_buffer_info.fd: %d \n", validate_buffer_info.fd);
        printf("validate_buffer_info.metadata_fd: %d \n", validate_buffer_info.metadata_fd);
        printf("validate_buffer_info.width: %d \n", validate_buffer_info.width);
        printf("validate_buffer_info.height: %d \n", validate_buffer_info.height);
        printf("validate_buffer_info.format: 0x%x \n", validate_buffer_info.format);
        printf("test_validate_gbmbuf_wlresource: Test passed \n");

        if (buffer_info.fd != validate_buffer_info.fd ||
            buffer_info.metadata_fd != validate_buffer_info.metadata_fd ||
            buffer_info.width != validate_buffer_info.width ||
            buffer_info.height != validate_buffer_info.height ||
            buffer_info.format != validate_buffer_info.format) {
          printf("test_validate_gbmbuf_wlresource: Validation of gbm buf info failed \n");
        } else {
            res = 1;
        }
    } else {
        printf("test_validate_gbmbuf_wlresource: Test failed \n");
    }

    gbm_ret = gbm_perform(GBM_PERFORM_GET_WL_RESOURCE_FROM_GBM_BUF_INFO, &buffer_info, &resource);

    gbm_ret = gbm_perform(GBM_PERFORM_GET_GBM_BUF_INFO_FROM_WL_RESOURCE, &resource, &validate_buffer_info);

    if (gbm_ret == GBM_ERROR_NONE) {
        printf("validate_buffer_info.fd: %d \n", validate_buffer_info.fd);
        printf("validate_buffer_info.metadata_fd: %d \n", validate_buffer_info.metadata_fd);
        printf("validate_buffer_info.width: %d \n", validate_buffer_info.width);
        printf("validate_buffer_info.height: %d \n", validate_buffer_info.height);
        printf("validate_buffer_info.format: 0x%x \n", validate_buffer_info.format);
        printf("test_validate_gbmbuf_wlresource: Test passed \n");

        if (buffer_info.fd != validate_buffer_info.fd ||
            buffer_info.metadata_fd != validate_buffer_info.metadata_fd ||
            buffer_info.width != validate_buffer_info.width ||
            buffer_info.height != validate_buffer_info.height ||
            buffer_info.format != validate_buffer_info.format) {
          printf("test_validate_gbmbuf_wlresource: Validation of gbm buf info failed \n");
        } else {
            res = 1;
        }
    } else {
        printf("test_validate_gbmbuf_wlresource: Test failed \n");
    }
    gbm_bo_destroy(bo);

    return res;
}

/*
 * Tests that we can allocate different buffer dimensions.
 */
static int test_alloc_free_sizes()
{
    int i;
    int ret;
    size_t bo_size=0;

    for(i = 1; i < 1920; i++) {
        struct gbm_bo *bo;
        bo = gbm_bo_create(gbm, i, i, GBM_FORMAT_XRGB8888, GBM_BO_USE_RENDERING);
        CHECK(check_bo(bo));
        printf("test_alloc_free BO width(%d)\n",gbm_bo_get_width(bo));
        printf("test_alloc_free BO height(%d)\n",gbm_bo_get_height(bo));
        printf("test_alloc_free BO stride(%d)\n",gbm_bo_get_stride(bo));
        printf("test_alloc_free BO format(%d)\n",gbm_bo_get_format(bo));

        ret=gbm_perform(GBM_PERFORM_GET_BO_SIZE, bo, &bo_size);
        if(ret == GBM_ERROR_NONE)
            printf("GET BO size=%d success\n",bo_size);
        else{
            printf("GET BO size failed\n");
            return 0;
        }

        if(gbm_bo_get_width(bo)!= i){
            printf("test_alloc_free BO width mismatch (expected =%d)\n",i);
            return 0;
        }
        if(gbm_bo_get_height(bo)!= i){
            printf("test_alloc_free BO height mismatch (expected=%d)\n",i);
            return 0;
        }
        if(gbm_bo_get_format(bo)!= GBM_FORMAT_XRGB8888){
            printf("test_alloc_free BO format mismatch (expected=%d)\n",GBM_FORMAT_XRGB8888);
            return 0;
        }

        gbm_bo_destroy(bo);
    }

    for(i = 1; i < 1920; i++) {
        struct gbm_bo *bo;
        bo = gbm_bo_create(gbm, i, 1, GBM_FORMAT_XRGB8888, GBM_BO_USE_RENDERING);
        CHECK(check_bo(bo));
        printf("test_alloc_free BO width(%d)\n",gbm_bo_get_width(bo));
        printf("test_alloc_free BO height(%d)\n",gbm_bo_get_height(bo));
        printf("test_alloc_free BO stride(%d)\n",gbm_bo_get_stride(bo));
        printf("test_alloc_free BO format(%d)\n",gbm_bo_get_format(bo));

        ret=gbm_perform(GBM_PERFORM_GET_BO_SIZE, bo, &bo_size);
        if(ret == GBM_ERROR_NONE)
            printf("GET BO size=%d success\n",bo_size);
        else{
            printf("GET BO size failed\n");
            return 0;
        }

        if(gbm_bo_get_width(bo)!= i){
            printf("test_alloc_free BO width mismatch (expected =%d)\n",i);
            return 0;
        }
        if(gbm_bo_get_height(bo)!= 1){
            printf("test_alloc_free BO height mismatch (expected=%d)\n",i);
            return 0;
        }
        if(gbm_bo_get_format(bo)!= GBM_FORMAT_XRGB8888){
            printf("test_alloc_free BO format mismatch (expected=%d)\n",GBM_FORMAT_XRGB8888);
            return 0;
        }
        gbm_bo_destroy(bo);
    }

    for(i = 1; i < 1920; i++) {
        struct gbm_bo *bo;
        bo = gbm_bo_create(gbm, 1, i, GBM_FORMAT_XRGB8888, GBM_BO_USE_RENDERING);
        CHECK(check_bo(bo));
        printf("test_alloc_free BO width(%d)\n",gbm_bo_get_width(bo));
        printf("test_alloc_free BO height(%d)\n",gbm_bo_get_height(bo));
        printf("test_alloc_free BO stride(%d)\n",gbm_bo_get_stride(bo));
        printf("test_alloc_free BO format(%d)\n",gbm_bo_get_format(bo));

        ret=gbm_perform(GBM_PERFORM_GET_BO_SIZE, bo, &bo_size);
        if(ret == GBM_ERROR_NONE)
            printf("GET BO size=%d success\n",bo_size);
        else{
            printf("GET BO size failed\n");
            return 0;
        }

        if(gbm_bo_get_width(bo)!= 1){
            printf("test_alloc_free BO width mismatch (expected =%d)\n",i);
            return 0;
        }
        if(gbm_bo_get_height(bo)!= i){
            printf("test_alloc_free BO height mismatch (expected=%d)\n",i);
            return 0;
        }
        if(gbm_bo_get_format(bo)!= GBM_FORMAT_XRGB8888){
            printf("test_alloc_free BO format mismatch (expected=%d)\n",GBM_FORMAT_XRGB8888);
            return 0;
        }
        gbm_bo_destroy(bo);
    }

    return 1;
}

/*
 * Tests that we can allocate different buffer formats.
 */
static int test_alloc_free_formats()
{
    int i;
    int size=ARRAY_SIZE(format_list);

    printf("test_alloc_free_formats array size(%d)\n",size);

    for(i = 0; i < ARRAY_SIZE(format_list); i++) {
        uint32_t format = format_list[i];
        if (gbm_device_is_format_supported(gbm, format, GBM_BO_USE_RENDERING)) {
            struct gbm_bo *bo;
            bo = gbm_bo_create(gbm, 1080, 1920, format, GBM_BO_USE_RENDERING);
            CHECK(check_bo(bo));
            printf("test_alloc_free_formats BO width(%d)\n",gbm_bo_get_width(bo));
            printf("test_alloc_free_formats BO height(%d)\n",gbm_bo_get_height(bo));
            printf("test_alloc_free_formats BO stride(%d)\n",gbm_bo_get_stride(bo));
            printf("test_alloc_free_formats BO format(%d)\n",gbm_bo_get_format(bo));

            if(gbm_bo_get_width(bo)!= 1080){
                printf("test_alloc_free_formats BO width mismatch (expected =%d)\n",1080);
                return 0;
            }
            if(gbm_bo_get_height(bo)!= 1920){
                printf("test_alloc_free_formats BO height mismatch (expected=%d)\n",1920);
                return 0;
            }
            if(gbm_bo_get_format(bo)!= format){
                printf("test_alloc_free_formats BO format mismatch (expected=%d)\n",format);
                return 0;
            }
            gbm_bo_destroy(bo);

        }else{
                printf("Current format (%d) at index =%d is not supported \n",format,i);
                return 0;
        }


    }


    return 1;
}

/*
 * Tests that we find at least one working format for each usage.
 */
static int test_alloc_free_usage()
{
    int i, j;

    for(i = 0; i < ARRAY_SIZE(usage_list); i++) {
        uint32_t usage = usage_list[i];
        int found = 0;
        for(j = 0; j < ARRAY_SIZE(format_list); j++) {
            uint32_t format = format_list[j];
            if (gbm_device_is_format_supported(gbm, format, usage)) {
                struct gbm_bo *bo;
                bo = gbm_bo_create(gbm, 1024, 1024, format, usage);
                CHECK(check_bo(bo));
                found = 1;
            }
        }
        CHECK(found);
    }

    return 1;
}

void destroy_data1(struct gbm_bo *bo, void *data)
{
    been_there1 = 1;
}

void destroy_data2(struct gbm_bo *bo, void *data)
{
    been_there2 = 1;
}

static int test_user_data()
{
    struct gbm_bo *bo1, *bo2;
    char *data1, *data2;

    been_there1 = 0;
    been_there2 = 0;

    bo1 = gbm_bo_create(gbm, 1024, 1024, GBM_FORMAT_XRGB8888, GBM_BO_USE_RENDERING);
    bo2 = gbm_bo_create(gbm, 1024, 1024, GBM_FORMAT_XRGB8888, GBM_BO_USE_RENDERING);
    data1 = (char*)malloc(1);
    data2 = (char*)malloc(1);
    CHECK(data1);
    CHECK(data2);

    gbm_bo_set_user_data(bo1, data1, destroy_data1);
    gbm_bo_set_user_data(bo2, data2, destroy_data2);

    CHECK((char*)gbm_bo_get_user_data(bo1) == data1);
    CHECK((char*)gbm_bo_get_user_data(bo2) == data2);

    gbm_bo_destroy(bo1);
    CHECK(been_there1 == 1);

    gbm_bo_set_user_data(bo2, NULL, NULL);
    gbm_bo_destroy(bo2);
    CHECK(been_there2 == 0);

    free(data1);
    free(data2);

    return 1;
}

static int test_import_fd()
{
    struct gbm_bo *bo1, *bo2;
    char *data1, *data2;
    struct gbm_import_fd_data buf_data;
    size_t bo_size;
    uint32_t ret=GBM_ERROR_NONE;

    bo1 = gbm_bo_create(gbm, 1024, 1024, GBM_FORMAT_XRGB8888, GBM_BO_USE_RENDERING);

    buf_data.fd=bo1->ion_fd;
    buf_data.height=1024;
    buf_data.width=1024;
    buf_data.format=GBM_FORMAT_XRGB8888;

    bo2 = gbm_bo_import(gbm, GBM_BO_IMPORT_FD,&buf_data,GBM_BO_USE_RENDERING);

    ret=gbm_perform(GBM_PERFORM_GET_BO_SIZE, bo2, &bo_size);
    if(ret == GBM_ERROR_NONE)
        printf("GET BO size=%d success\n",bo_size);
    else{
        printf("GET BO size failed\n");
        return 0;
    }

    if(gbm_bo_get_width(bo2)!= 1024){
        printf("test_alloc_free BO width mismatch (expected =%d)\n",1024);
        return 0;
    }
    if(gbm_bo_get_fd(bo2)!=buf_data.fd){
        printf("test_alloc_free BO width mismatch (expected =%d)\n",buf_data.fd);
        return 0;
    }
    if(gbm_bo_get_height(bo2)!= 1024){
        printf("test_alloc_free BO height mismatch (expected=%d)\n",1024);
        return 0;
    }
    if(gbm_bo_get_format(bo2)!= GBM_FORMAT_XRGB8888){
        printf("test_alloc_free BO format mismatch (expected=%d)\n",GBM_FORMAT_XRGB8888);
        return 0;
    }

    gbm_bo_destroy(bo1);

    return 1;
}


static int test_import_gbm_buf()
{
    struct gbm_bo *bo1, *bo2;
    char *data1, *data2;
    struct gbm_buf_info buf_info;
    size_t bo_size;
    uint32_t ret=GBM_ERROR_NONE;

    bo1 = gbm_bo_create(gbm, 1024, 1024, GBM_FORMAT_XRGB8888, GBM_BO_USE_RENDERING);

    buf_info.fd=bo1->ion_fd;
    buf_info.metadata_fd=bo1->ion_metadata_fd;
    buf_info.height=1024;
    buf_info.width=1024;
    buf_info.format=GBM_FORMAT_XRGB8888;

    bo2 = gbm_bo_import(gbm, GBM_BO_IMPORT_GBM_BUF_TYPE,&buf_info,GBM_BO_USE_RENDERING);

    ret=gbm_perform(GBM_PERFORM_GET_BO_SIZE, bo2, &bo_size);
    if(ret == GBM_ERROR_NONE)
        printf("GET BO size=%d success\n",bo_size);
    else{
        printf("GET BO size failed\n");
        return 0;
    }

    if(gbm_bo_get_width(bo2)!= 1024){
        printf("test_alloc_free BO width mismatch (expected =%d)\n",1024);
        return 0;
    }
    if(gbm_bo_get_fd(bo2)!=buf_info.fd){
        printf("test_alloc_free BO width mismatch (expected =%d)\n",buf_info.fd);
        return 0;
    }
    if(gbm_bo_get_height(bo2)!= 1024){
        printf("test_alloc_free BO height mismatch (expected=%d)\n",1024);
        return 0;
    }
    if(gbm_bo_get_format(bo2)!= GBM_FORMAT_XRGB8888){
        printf("test_alloc_free BO format mismatch (expected=%d)\n",GBM_FORMAT_XRGB8888);
        return 0;
    }

    gbm_bo_destroy(bo1);

    return 1;
}

static int test_import_wl_buffer()
{
    struct gbm_bo *bo1, *bo2;
    char *data1, *data2;
    struct wl_resource resource;
    static struct gbm_buf_info buf_info;
    size_t bo_size;
    uint32_t ret=GBM_ERROR_NONE;

    resource.data=&buf_info;

    bo1 = gbm_bo_create(gbm, 1024, 1024, GBM_FORMAT_XRGB8888, GBM_BO_USE_RENDERING);

    buf_info.fd=bo1->ion_fd;
    buf_info.metadata_fd=bo1->ion_metadata_fd;
    buf_info.height=1024;
    buf_info.width=1024;
    buf_info.format=GBM_FORMAT_XRGB8888;

    bo2 = gbm_bo_import(gbm, GBM_BO_IMPORT_WL_BUFFER,&resource,GBM_BO_USE_RENDERING);

    ret=gbm_perform(GBM_PERFORM_GET_BO_SIZE, bo2, &bo_size);
    if(ret == GBM_ERROR_NONE)
        printf("GET BO size=%d success\n",bo_size);
    else{
        printf("GET BO size failed\n");
        return 0;
    }

    if(gbm_bo_get_width(bo2)!= 1024){
        printf("test_alloc_free BO width mismatch (expected =%d)\n",1024);
        return 0;
    }
    if(gbm_bo_get_fd(bo2)!=buf_info.fd){
        printf("test_alloc_free BO width mismatch (expected =%d)\n",buf_info.fd);
        return 0;
    }
    if(gbm_bo_get_height(bo2)!= 1024){
        printf("test_alloc_free BO height mismatch (expected=%d)\n",1024);
        return 0;
    }
    if(gbm_bo_get_format(bo2)!= GBM_FORMAT_XRGB8888){
        printf("test_alloc_free BO format mismatch (expected=%d)\n",GBM_FORMAT_XRGB8888);
        return 0;
    }

    gbm_bo_destroy(bo1);

    return 1;
}


/*
 * Tests focussed on validating GBM_FORMAT_P010 format
 *  following apis are validated for above format
 */
static int test_validate_p010_format()
{
  int i;
  struct gbm_bo *gb_bo[1000];
  int ret;
  void *prm;
  int ubwc_status;

  for(i = 0; i < 2; i++) {
      printf("test_alloc_free run(%d)\n",i);
      gb_bo[i] = gbm_bo_create(gbm, 1024, 1024, GBM_FORMAT_P010, GBM_BO_USE_RENDERING);
      CHECK(check_bo(gb_bo[i]));

      prm = (void *)&ubwc_status;
      ret = gbm_perform(GBM_PERFORM_GET_UBWC_STATUS, gb_bo[i], prm);
      printf("ubwc status for BO is %d\n", ubwc_status);
   }

  for(i = 0; i < 2; i++)
      gbm_bo_destroy(gb_bo[i]);

}

/*
 * Tests repeated alloc/free with
 *  color space
 */
static int test_validate_colorspace()
{
    int i;
    struct gbm_bo *gb_bo[1000];
    int ret;
    int meta_fd;
    uint32_t align_wdth;
    uint32_t align_hght;
    void *prm;
    int colorspace_set = 1;
    int colorspace_get;

    for(i = 0; i < 2; i++) {
        printf("test_alloc_free run(%d)\n",i);
        gb_bo[i] = gbm_bo_create(gbm, 1024, 1024, GBM_FORMAT_XRGB8888, GBM_BO_USE_RENDERING);
        CHECK(check_bo(gb_bo[i]));

        prm = (void *)&colorspace_set;
        //Testing Set and Get Metadata
        printf("Set param addr=%x , value=%u\n",prm,*(unsigned int *)prm);
        ret = gbm_perform(GBM_PERFORM_SET_METADATA, gb_bo[i], GBM_METADATA_SET_COLOR_SPACE, prm);
        if(ret == GBM_ERROR_NONE)
            printf("Set Metadata Success\n");
        else
            printf("Set Metadata Failed\n");

        prm = (void *)&colorspace_get;
        ret = gbm_perform(GBM_PERFORM_GET_METADATA, gb_bo[i], GBM_METADATA_GET_COLOR_SPACE, prm);

        if (ret == GBM_ERROR_NONE) {
            printf("Get Metadata Success\n");
            if (colorspace_get != colorspace_set)
                printf("set-get mismatch Set=%d, Get=%d\n", colorspace_set, colorspace_get);
            else
                printf("Metdata-validate, set-get Matched => %x\n", colorspace_get);
        }
        else
            printf("Get Metadata Failed\n");
     }

    for(i = 0; i < 2; i++)
        gbm_bo_destroy(gb_bo[i]);

    return 1;
}


/*
 * Tests destruction.
 */
static int test_destroy()
{
    printf("Inside test_destroy\n");
    gbm_device_destroy(gbm);
    close(fd);

    return 1;
}

static int test_multi_create_device(void)
{
    int fd;
    struct gbm_device *gbm1;
    struct gbm_device *gbm2;
    const char* backend_name;
    int ret;
    int mgc_id;
    int i=0;

    fd = open("/dev/dri/card0", O_RDWR | O_CLOEXEC);
    if (fd < 0)
    if (fd == ENODISPLAY)
        return 0;

    CHECK(fd >= 0);
    gbm1 = gbm_create_device(fd);
    CHECK(gbm_device_get_fd(gbm1) == fd);
    backend_name = gbm_device_get_backend_name(gbm1);
    CHECK(backend_name);

    gbm2 = gbm_create_device(fd);
    CHECK(gbm_device_get_fd(gbm2) == fd);
    backend_name = gbm_device_get_backend_name(gbm2);
    CHECK(backend_name);

    for(i=0;i<10;i++)
    {

        //Testing Device Magic Authentication
        ret=gbm_perform(GBM_PERFORM_GET_DRM_DEVICE_MAGIC,gbm1,&mgc_id);
        if(ret==GBM_ERROR_NONE)
            printf("Get DRM Magic ID Success Magic ID=%u\n",mgc_id);
        else {
            printf("Get DRM Magic ID Failed\n");
            return 0;
        }

        //Testing Device Magic Authentication
        ret=gbm_perform(GBM_PERFORM_VALIDATE_DEVICE,gbm1);
        if(ret==GBM_ERROR_NONE)
            printf("Validate GBM Magic ID Success=%u\n",mgc_id);
        else {
            printf("Validate GBM Magic ID Failed\n");
            return 0;
        }


        ret=gbm_perform(GBM_PERFORM_AUTH_DRM_DEVICE_MAGIC,gbm1,mgc_id);
        if(ret==GBM_ERROR_NONE)
            printf("Auth DRM Magic ID Success\n");
        else {
            printf("Auth DRM Magic ID Failed\n");
            return 0;
        }

    }


    if(gbm1==gbm2)
        printf("GBM Device for both instance is the same:\n");

    printf("First reference to gbm_device_destroy:\n");
    gbm_device_destroy(gbm1);
    printf("Second reference to gbm_device_destroy:\n");
    gbm_device_destroy(gbm2);

    close(fd);

    return 1;

}
int gbm_test_help() {
  printf("Please Enter Test No:\n");
  printf("1 for Create/Destroy GBM device\n");
  printf("2 for BO Write/Read, Hash Map dump, BO Dump and CPU map/unmap Operation\n");
  printf("3 for Validate DRM Magic ID\n");
  printf("4 for Get/Set Metadata Operation\n");
  printf("5 for stress Alloc/Destroy BO \n");
  printf("6 for stress different size Alloc/Destroy BO \n");
  printf("7 for stress different standard formats Alloc/Destroy BO \n");
  printf("8 for Surface Create/Destroy and other Surface related operations\n");
  printf("9 for BO Create/Destroy and with VENUS FORMAT\n");
  printf("10 for BO Create/Destroy and import fd\n");
  printf("11 for BO Create/Destroy and import gbm buf\n");
  printf("12 for BO Create/Destroy and import wl buf\n");
  printf("13 for BO Write/Read and CPU map/unmap Operation on Secure Buffer\n");
  printf("14 for GBM  Multiple device/destroy calls\n");
  printf("15 Test Metadata fd\n");
  printf("16 Test Get aligned width and height\n");
  printf("17 Test Get UBWC status\n");
  printf("18 Test UBWC status on Surface\n");
  printf("19 Validate gbm_buf_info from wl_resource\n");
  printf("20 Test Colorspace metadata operations on BO\n");
  printf("21 Test GBM_FORMAT_P010 format operations on BO\n");
  return 0;
}
int main(int argc, char *argv[])
{
    int result=1;
    int param=0;

    if(argc > 1)
        param=atoi(argv[argc-1]);
    else {
        return gbm_test_help();
    }

    switch(param) {
        case 1: //Create and Destroy GBM
            result &= test_init();
            result &= test_destroy();
        break;
        case 2: //Perform BO Write/Read and Perform CPU map/Unmap
            result &= test_bo_write(GBM_BO_USE_RENDERING);
        break;
        case 3: //Validate DRM Magic
            result &= test_drm_magic();
        break;
        case 4:
            result &= test_metadata();
        break;
        case 5:
            result &= test_init();
            result &= test_alloc_free();
            result &= test_destroy();
        break;
        case 6:
            result &= test_init();
            result &= test_alloc_free_sizes();
            result &= test_destroy();
        break;
        case 7:
            result &= test_init();
            result &= test_alloc_free_formats();
            result &= test_destroy();
        break;
        case 8:
            result &= test_init();
            result &= test_surface_create_destroy();
            result &= test_destroy();
            break;
        case 9:
            result &= test_init();
            result &= test_vns_alloc_free();
            result &= test_destroy();
            break;
        case 10:
            result &= test_init();
            result &= test_import_fd();
            break;
        case 11:
            result &= test_init();
            result &= test_import_gbm_buf();
            break;
        case 12:
            result &= test_init();
            result &= test_import_wl_buffer();
            break;
        case 13:
            //This test is suppose to fail so negatre the result
            result &= test_bo_write_secure(GBM_BO_USAGE_PROTECTED_QTI);
            break;
        case 14:
            result &= test_multi_create_device();
            break;
        case 15:
            result &= test_init();
            result &= test_alloc_free_validate_meta_fd();
            result &= test_destroy();
            break;
        case 16:
            result &= test_init();
            result &= test_get_aligned_width_height();
            result &= test_destroy();
            break;
        case 17:
            result &= test_init();
            result &= test_get_ubwc_status();
            result &= test_destroy();
            break;
        case 18:
            result &= test_init();
            result &= test_surface_ubwc_status();
            result &= test_destroy();
            break;
        case 19:
            result &= test_init();
            result &= test_validate_gbmbuf_wlresource();
            result &= test_destroy();
            break;
        case 20:
            result &= test_init();
            result &= test_validate_colorspace();
            result &= test_destroy();
            break;
        case 21:
            result &= test_init();
            result &= test_validate_p010_format();
            result &= test_destroy();
            break;
        default:
            gbm_test_help();
            return 0;
    }

    if (!result) {
        printf("[  FAILED  ] graphics_Gbm test failed\n");
        return EXIT_FAILURE;
    } else {
        printf("[  PASSED  ] graphics_Gbm test success\n");
        return EXIT_SUCCESS;
    }
}

