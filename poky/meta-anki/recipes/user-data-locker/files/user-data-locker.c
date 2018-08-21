#include <stdio.h>
#include <string.h>

int main(int argc, char** argv)
{
    int r=0;
    FILE *fp=NULL;

    // WARNING: switchboard uses the first 256kb of the switchboard partition
    //          user-data-locker is using the last 256kb
    char *vault_path="/dev/block/bootdevice/by-name/switchboard";
    unsigned int vault_size=256*1024;

    int key_length=65;
    char key[key_length];

    char *rng_path="/dev/urandom";

    if(argc>1 && !strcmp(argv[1],"reset")) {
        // generate new key;
        fp = fopen(rng_path,"rb");
        if (fread(key,sizeof(char),key_length,fp)!=key_length) {
            fprintf(stderr,"%s: ERROR: cannot read %d bytes from %s\n",argv[0],key_length,rng_path);
            r=4;
            goto close_file;
        }
        fclose(fp);


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

        if (fwrite(key,sizeof(char),key_length,fp)!=key_length) {
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

        if (fread(key,sizeof(char),key_length,fp)!=key_length) {
            fprintf(stderr,"%s: ERROR: cannot read %d bytes from %s\n",argv[0],key_length,vault_path);
            r=4;
            goto close_file;
        }

    }

    //output key
    fwrite(key,sizeof(char),key_length,stdout);

close_file:
    fclose(fp);

    return r;
}

