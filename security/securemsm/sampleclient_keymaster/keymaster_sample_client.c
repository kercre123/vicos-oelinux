/*
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*/

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <unistd.h>
#include <dirent.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdbool.h>
#include "keymaster.h"

#define RSA_KEY_SIZE (2048)
#define RSA_PUBLIC_EXP (0x10001)
#define BLOB_FILE_NAME "/data/key_blob"
#define EMK_FILE_NAME "/data/eMK"

int EncryptAndStore(unsigned char *blob,unsigned int length)
{
    FILE *fptr;
    int ret=0;
    keymaster_key_blob_t key_blob = {NULL,0};
    keymaster_error_t error;
    size_t input_consumed,count;
    keymaster_blob_t input = { blob, length };
    keymaster_blob_t encrypted_blob = {NULL,0};
    qti_keymaster_handle_t *kd=keymaster_get_device();
    if(!kd)
        return -1;
    keymaster_key_param_t gen_key_params[] = {
    keymaster_param_enum(KM_TAG_ALGORITHM, KM_ALGORITHM_RSA),
    keymaster_param_int(KM_TAG_KEY_SIZE, RSA_KEY_SIZE),
    keymaster_param_long(KM_TAG_RSA_PUBLIC_EXPONENT, RSA_PUBLIC_EXP),
    /* The only allowed purpose for this key is encrypting and decrypting */
    keymaster_param_enum(KM_TAG_PURPOSE, KM_PURPOSE_ENCRYPT),
    keymaster_param_enum(KM_TAG_PURPOSE, KM_PURPOSE_DECRYPT),
    /* Padding & digest specifications. */
    keymaster_param_enum(KM_TAG_PADDING, KM_PAD_NONE),
    keymaster_param_enum(KM_TAG_DIGEST, KM_DIGEST_NONE),
    /* Require that the key be usable in standalone mode.  File system isn't available. */
    keymaster_param_enum(KM_TAG_BLOB_USAGE_REQUIREMENTS, KM_BLOB_STANDALONE),
    keymaster_param_bool(KM_TAG_NO_AUTH_REQUIRED),
    /* Rate-limit key usage attempts, to rate-limit brute force */
    keymaster_param_int(KM_TAG_MIN_SECONDS_BETWEEN_OPS, 1),
    };

    keymaster_key_param_set_t gen_key_param_set = { gen_key_params, sizeof(gen_key_params)/sizeof(*gen_key_params) };
    keymaster_key_param_t encrypt_params[] = {
        keymaster_param_enum(KM_TAG_PADDING, KM_PAD_NONE),
        keymaster_param_enum(KM_TAG_DIGEST, KM_DIGEST_NONE),
    };
    keymaster_key_param_set_t encrypt_param_set = { encrypt_params, sizeof(encrypt_params)/sizeof(*encrypt_params) };
    keymaster_operation_handle_t op_handle;
    error = generate_key(kd, &gen_key_param_set,&key_blob, NULL );
    if(error!=KM_ERROR_OK)
    {
        printf("Key generation failed\n");
        return -1;
    }
    fptr=fopen(BLOB_FILE_NAME,"wb");
    if( fptr==NULL )
    {
        printf("Couldn't open file %s\n",BLOB_FILE_NAME);
        ret=-1;
        goto exit;
    }
    count=fwrite(key_blob.key_material,1,key_blob.key_material_size,fptr);
    fclose(fptr);
    if(count!=key_blob.key_material_size)
    {
        printf("Error writing to file %s\n",BLOB_FILE_NAME);
        ret=-1;
        goto exit;
    }

    error = begin_operation(kd, KM_PURPOSE_ENCRYPT, &key_blob,&encrypt_param_set, NULL, &op_handle);
    if(error!=KM_ERROR_OK)
    {
        printf("Begin Operation failed\n");
        ret=-1;
        goto exit;
    }
    error = update_operation(kd, op_handle, NULL ,&input, &input_consumed, NULL, NULL );
    if(error!=KM_ERROR_OK)
    {
        printf("Update Operation failed\n");
        ret=-1;
        goto exit;
    }
    error =finish_operation(kd, op_handle, NULL, NULL, NULL,&encrypted_blob);
    if(error!=KM_ERROR_OK)
    {
        printf("Finish Operation failed\n");
        ret=-1;
        goto exit;
    }
    fptr=fopen(EMK_FILE_NAME,"wb");
    if(!fptr)
    {
        printf("Failed to open encrypted storage file\n");
        ret=-1;
        goto exit;
    }
    count=fwrite(encrypted_blob.data,1,encrypted_blob.data_length,fptr);
    fclose(fptr);
    if(count!=encrypted_blob.data_length)
    {
        printf("Error writing to file %s\n",EMK_FILE_NAME);
        ret=-1;
    }
exit:
    keymaster_release_device(kd);
    if(key_blob.key_material)
        free(key_blob.key_material);
    if(encrypted_blob.data)
        free(encrypted_blob.data);
    return ret;
}
int RetreiveAndDecrypt(keymaster_blob_t *clear)
{
    FILE *fptr;
    unsigned char encrypted[256];
    unsigned int key_blob_size;
    int ret=0;
    keymaster_error_t error;
    keymaster_operation_handle_t op_handle;
    size_t input_consumed,count;
    keymaster_key_blob_t key_blob = {NULL,0};
    size_t encrypted_size=256;
    keymaster_blob_t encrypt__blob = { encrypted, encrypted_size };
    keymaster_key_param_t decrypt_params[] = {
       keymaster_param_enum(KM_TAG_PADDING, KM_PAD_NONE),
       keymaster_param_enum(KM_TAG_DIGEST, KM_DIGEST_NONE),
    };
    keymaster_key_param_set_t decrypt_params_set = { decrypt_params, sizeof(decrypt_params)/sizeof(*decrypt_params) };
    qti_keymaster_handle_t *kd=keymaster_get_device();
    if(!kd)
        return -1;
    fptr=fopen(EMK_FILE_NAME,"rb");
    if(!fptr)
    {
        printf("Couldn't open eMK file\n");
        return -1;
    }
    count=fread(encrypt__blob.data,1,encrypt__blob.data_length,fptr);
    fclose(fptr);
    if(count != encrypt__blob.data_length)
    {
        printf("Error reading eMK file\n");
        return -1;
    }
    fptr=fopen(BLOB_FILE_NAME,"rb");
    if(!fptr)
    {
        printf("Couldn't open the encrypted data file\n");
        return -1;
    }
    fseek(fptr, 0L, SEEK_END);
    key_blob_size = ftell(fptr);
    fseek(fptr, 0L, SEEK_SET);
    key_blob.key_material=malloc(key_blob_size);
    if(!key_blob.key_material)
    {
        printf("malloc failed\n");
        fclose(fptr);
        return -1;
    }
    key_blob.key_material_size=key_blob_size;
    count=fread(key_blob.key_material,1,key_blob.key_material_size,fptr);
    fclose(fptr);
    if(count!=key_blob.key_material_size)
    {
        printf("read error %s\n",BLOB_FILE_NAME);
        ret=-1;
        goto exit;
    }
    error = begin_operation(kd, KM_PURPOSE_DECRYPT, &key_blob,&decrypt_params_set, NULL, &op_handle);
    if(error!=KM_ERROR_OK)
    {
        printf("Begin Operation failed\n");
        ret=-1;
        goto exit;
    }

    error = update_operation(kd,op_handle, NULL ,&encrypt__blob, &input_consumed, NULL, NULL );
    if(error!=KM_ERROR_OK)
    {
        printf("Update Operation failed\n");
        ret=-1;
        goto exit;
    }

    error =finish_operation(kd,op_handle, NULL, NULL, NULL,clear);
    if(error!=KM_ERROR_OK)
    {
        printf("Finish Operation failed\n");
        ret=-1;
        goto exit;
    }
exit:
    keymaster_release_device(kd);
    if(key_blob.key_material)
        free(key_blob.key_material);
    return ret;
}
void main(int argc, char *argv[])
{
    unsigned char MasterKey[256],pass=1;
    keymaster_blob_t clear;
    int i;
    for(i=0;i<256;i++)
        MasterKey[i]=i;
    if(EncryptAndStore(MasterKey,256)!=0)
    {
        printf("Encrypt and store failed\n");
        return;
    }
    printf("Encrypt and store succeeded\n");
    if(RetreiveAndDecrypt(&clear)!=0)
    {
        printf("RetreiveAndDecrypt failed\n");
        return;
    }
    for(i=0;i<256;i++)
    {
        if(i%64==0)
            printf("\n");
        printf("[%02X]",clear.data[i]);
        if(clear.data[i] != i)
            pass=0;
    }
    free(clear.data);
    printf("\nTest result: %s\n",(pass)?"PASS":"FAIL");
}
