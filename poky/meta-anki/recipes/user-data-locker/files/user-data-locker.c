#include <stdio.h>
#include <string.h>

#define DEBUG 0

void encrypt(int length, char *clear, char *encrypted, int key_length, char *key)
{
    int i,j;
    for (i=0, j=0; i<length; i++, j++) {
        if (j>=key_length) {
            j=0;
        }
        encrypted[i] = clear[i] ^ key[j]; 
    }
}
void decrypt(int length, char *clear, char *encrypted, int key_length, char *key)
{
    int i,j;
    for (i=0, j=0; i<length; i++, j++) {
        if (j>=key_length) {
            j=0;
        }
        clear[i] = encrypted[i] ^ key[j]; 
    }
}

#if DEBUG
void hexdump(int length, char *buf)
{
    for(int i=0; i<length; i++) {
        fprintf(stderr,"%02x",buf[i]);
    }
}
#endif

int main(int argc, char** argv)
{
    int r=0;
    FILE *fp=NULL;

    // WARNING: switchboard uses the first 256kb of the switchboard partition
    //          user-data-locker is using the last 256kb
    char *vault_path="/dev/block/bootdevice/by-name/switchboard";
    unsigned int vault_size=256*1024;

    int key_length=65;
    char clear_key[key_length];
    char encrypted_key[key_length];
   
    char *serial_path="/sys/devices/soc0/serial_number";
    int serial_length=9;
    char serial[serial_length];

    char *rng_path="/dev/urandom";

    fp = fopen(serial_path,"rb");
    if (fread(serial,sizeof(char),serial_length,fp)!=serial_length) {
        fprintf(stderr,"%s: ERROR: cannot read %d bytes from %s\n",argv[0],serial_length,serial_path);
        r=5;
        goto close_file;
    }
    fclose(fp);

#if DEBUG
    fprintf(stderr,"serial: ");
    fwrite(serial,sizeof(char),serial_length,stderr);
    fprintf(stderr,"\n");
#endif

    if(argc>1 && !strcmp(argv[1],"reset")) {
        // generate new key;
        fp = fopen(rng_path,"rb");
        if (fread(clear_key,sizeof(char),key_length,fp)!=key_length) {
            fprintf(stderr,"%s: ERROR: cannot read %d bytes from %s\n",argv[0],key_length,rng_path);
            r=4;
            goto close_file;
        }
        fclose(fp);
        
#if DEBUG
        fprintf(stderr,"generated clear key: ");
        hexdump(key_length,clear_key);
        fprintf(stderr,"\n");
#endif

        encrypt(key_length, clear_key, encrypted_key, serial_length, serial);
        
#if DEBUG
        fprintf(stderr,"encrypted key: ");
        hexdump(key_length,encrypted_key);
        fprintf(stderr,"\n");
#endif

        // write key to vault
        fp = fopen(vault_path,"wb");
        if (fp==NULL) {
            fprintf(stderr,"%s: ERROR: cannot open %s\n",argv[0],vault_path);
            return 1;
        }

        if (fseek(fp,-vault_size,SEEK_END)) {
            fprintf(stderr,"%s: ERROR: cannot set stream position to -%d bytes from SEEK_END\n",argv[0],key_length);
            r=2;
            goto close_file;
        }

        if (fwrite(encrypted_key,sizeof(char),key_length,fp)!=key_length) {
            fprintf(stderr,"%s: ERROR: cannot write %d bytes to %s\n",argv[0],key_length,vault_path);
            r=3;
            goto close_file;
        }
        
    } else {
        // read key from vault

        fp = fopen(vault_path,"rb");
        if (fp==NULL) {
            fprintf(stderr,"%s: ERROR: cannot open %s\n",argv[0],vault_path);
            return 1;
        }

        if (fseek(fp,-vault_size,SEEK_END)) {
            fprintf(stderr,"%s: ERROR: cannot set stream position to -%d bytes from SEEK_END\n",argv[0],key_length);
            r=2;
            goto close_file;
        }

        if (fread(encrypted_key,sizeof(char),key_length,fp)!=key_length) {
            fprintf(stderr,"%s: ERROR: cannot read %d bytes from %s\n",argv[0],key_length,vault_path);
            r=4;
            goto close_file;
        }

#if DEBUG
        fprintf(stderr,"read encrypted key: ");
        hexdump(key_length,encrypted_key);
        fprintf(stderr,"\n");
#endif

        decrypt(key_length, clear_key, encrypted_key, serial_length, serial);

#if DEBUG
        fprintf(stderr,"decrypted clear key: ");
        hexdump(key_length,clear_key);
        fprintf(stderr,"\n");
#endif

    }

    //output key
    fwrite(clear_key,sizeof(char),key_length,stdout);

close_file:
    fclose(fp);

    return r;
}

