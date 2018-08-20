#include <stdio.h>

int main(int argc, char** argv)
{
    int r=0;
    char *vault_path="/sys/devices/soc0/serial_number";

    int key_length=9;
    char key[key_length];

    FILE *fp;

    fp = fopen(vault_path,"r");
    if (fp==NULL) {
        fprintf(stderr,"%s: ERROR: cannot open %s\n",argv[0],vault_path);
        return 1;
    }

    if (fread(key,sizeof(char),key_length,fp)!=key_length) {
        fprintf(stderr,"%s: ERROR: cannot read %d bytes from %s\n",argv[0],key_length,vault_path);
        r=2;
        goto close_file;
    }

    fwrite(key,sizeof(char),key_length,stdout);

close_file:
    fclose(fp);

    return r;
}

