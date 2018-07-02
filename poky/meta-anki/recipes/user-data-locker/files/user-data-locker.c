#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <dlfcn.h>
#include <string.h>

#define MAX_PASS_LEN 32

#define TZ_APP_PATH "/firmware/image/"
//#define TZ_APP_NAME "keymaste"
#define TZ_APP_NAME "sampleapp"
#define TZ_APP_BUFFER_SIZE 0x10000 


#define QSEECOM_DISK_ENCRYPTION 1
#define QSEECOM_LIBRARY_PATH "/usr/lib/libQseeComApi.so"
struct qseecom_handle {
        unsigned char *ion_sbuffer;
};

static int loaded_library = 0;
static int (*qseecom_create_key)(int, void*);
static int (*qseecom_update_key)(int, void*, void*);
static int (*qseecom_wipe_key)(int);
static int (*qseecom_start_app)(struct qseecom_handle ** handle, char* path,
                               char* appname, size_t size);
static int (*qseecom_shutdown_app)(struct qseecom_handle **handle);

static int load_qseecom_library()
{
    const char *error = NULL;
    static struct qseecom_handle *qhandle;

    fprintf(stderr,"%s: start\n",__func__);
    if (loaded_library) {
        fprintf(stderr,"%s: already loaded\n",__func__);
        return loaded_library;
    }

    void * handle = dlopen(QSEECOM_LIBRARY_PATH, RTLD_NOW);
    if(!handle) {
        fprintf(stderr,"Could not load %s\n",QSEECOM_LIBRARY_PATH);
        goto final;
    }

    dlerror(); /* Clear any existing error */
    *(void **) (&qseecom_create_key) = dlsym(handle,"QSEECom_create_key");
    if((error = dlerror()) != NULL) {
        goto final;
    }
    fprintf(stderr,"Success loading QSEECom_create_key \n");

    *(void **) (&qseecom_update_key) = dlsym(handle,"QSEECom_update_key_user_info");
    if ((error = dlerror()) != NULL) {
        goto final;
    }
    fprintf(stderr,"Success loading QSEECom_update_key_user_info\n");

    *(void **) (&qseecom_wipe_key) = dlsym(handle,"QSEECom_wipe_key");
    if ((error = dlerror()) != NULL) {
        goto final;
    }
    fprintf(stderr,"Success loading QSEECom_wipe_key \n");

    *(void **)(&qseecom_start_app) = dlsym(handle,"QSEECom_start_app");
    if ((error = dlerror()) != NULL) {
        goto final;
    }
    fprintf(stderr,"Success loading QSEECom_start_app \n");

    *(void **)(&qseecom_shutdown_app) = dlsym(handle,"QSEECom_shutdown_app");
    if ((error = dlerror()) != NULL) {
        goto final;
    }
    fprintf(stderr,"Success loading QSEECom_shutdown_app \n");

    loaded_library = 1;

final:
    if(error) {
        fprintf(stderr,"Error %s loading symbols for QSEECom APIs \n", error);
        dlclose(handle);
    }

    fprintf(stderr,"%s: end\n",__func__);
    return loaded_library;
}

int main(int argc, char** argv) {
    int r;

    unsigned char *passwd = "123456";
    unsigned char *tmp_passwd = NULL;

	static struct qseecom_handle *app_handle;

    printf("Anki user data locker\n");

    tmp_passwd = (unsigned char*)malloc(sizeof(unsigned char)*MAX_PASS_LEN);
    memcpy(tmp_passwd, passwd, strnlen(passwd,MAX_PASS_LEN));

    fprintf(stderr,"loading QSEECom library...");
    r=load_qseecom_library();
    if(!r) {
        fprintf(stderr,"ERROR\n");
        return 1;
    }
    fprintf(stderr,"OK\n");

    r = qseecom_start_app( &app_handle, TZ_APP_PATH, TZ_APP_NAME, TZ_APP_BUFFER_SIZE );
    if (r < 0) {
        fprintf(stderr,"%s: ERROR: could not load %s/%s\n",__func__,TZ_APP_PATH,TZ_APP_NAME);
        return 1;
    }
	fprintf(stderr,"Success starting QSEECom app %s\n",TZ_APP_NAME);

//    r = qseecom_start_app( &app_handle, TZ_APP_PATH, "keymaste", TZ_APP_BUFFER_SIZE );
//    if (r < 0) {
//        fprintf(stderr,"%s: ERROR: could not load %s/%s\n",__func__,TZ_APP_PATH,"keymaste");
//        return 1;
//    }
//	fprintf(stderr,"Success starting QSEECom app %s\n",TZ_APP_NAME);


    r=qseecom_create_key(QSEECOM_DISK_ENCRYPTION, tmp_passwd);
    fprintf(stderr,"%s: qseecom_create_key() returned err=%d\n",__func__,r);
//    fprintf(stderr,"wiping hw encryption key...");
//    r=clear_hw_device_encryption_key();
//    if(r<0) {
//        fprintf(stderr,"ERROR: %d\n",r);
//        //return 1;
//    } else {
//        fprintf(stderr,"OK\n");
//    }
//
//    fprintf(stderr,"setting hw encryption key...");
//    r=set_hw_device_encryption_key("123456789 123456789 123456789 12","aes-xts");
//    if(r<0) {
//        fprintf(stderr,"ERROR: %d\n",r);
//        // return 1;
//    } else {
//        fprintf(stderr,"OK\n");
//    }
//    
//    fprintf(stderr,"should_use_keymaster() => %d\n",should_use_keymaster());
//    fprintf(stderr,"is_ice_enabled() => %d\n",is_ice_enabled());
    
    return 0;
}

