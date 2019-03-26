#!/usr/bin/env bash
set -e
set -u

function usage {
	echo "Usage:"
	echo "$0 <path to extracted file system of firmware image>"
	echo "Example: $0 <mount_path_to_root_filesystem>"
    echo "To mount a Victor image type mount -o loop apq8009-robot-sysfs.img <mountdirector>"
	exit 1
}

function msg {
    echo "$1" | tee -a $FILE
}

function getArray {
    array=() # Create array
    while IFS= read -r line
    do
        array+=("$line")
    done < "$1"
}

PARENT_PATH=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )

# Check arguments
if [[ $# -gt 2 || $# -lt 1 ]]; then
    usage
fi

# Set variables
FIRMDIR=$1
if [[ $# -eq 2 ]]; then
    FILE=$2
else
    today=`date '+%Y%m%d%H%M%S'`;
    FILE="afat$today.log"
fi
# Remove previous file if it exists, is a file and doesn't point somewhere
if [[ -e "$FILE" && ! -h "$FILE" && -f "$FILE" ]]; then
    rm -f $FILE
fi

#mount filesystem
#MNTDIR="$1.mount"
#mkdir $MNTDIR
#udo mount -o loop $1 $MNTDIR

# Perform searches
msg "#Firmware Directory"
msg $FIRMDIR
msg "#Searching for password files"
getArray "${PARENT_PATH}/data/passfiles"
passfiles=("${array[@]}")
for passfile  in "${passfiles[@]}"
do
    msg "# $passfile"
    find $FIRMDIR -name $passfile | cut -c${#FIRMDIR}- | tee -a $FILE
    msg ""
done
msg "#Searching for Unix-MD5 hashes#"
egrep -sro '\$1\$\w{8}\S{23}' $FIRMDIR | tee -a $FILE
msg ""
if [[ -d "$FIRMDIR/etc/ssl" ]]; then
    msg "#List etc/ssl directory#"
    ls -l $FIRMDIR/etc/ssl | tee -a $FILE
fi
msg ""
msg "#Searching for SSL related files#"
getArray "${PARENT_PATH}/data/sslfiles"
sslfiles=("${array[@]}")
for sslfile in ${sslfiles[@]}
do
    msg "#$sslfile"
        certfiles=( $(find ${FIRMDIR} -name ${sslfile}) )
        : "${certfiles:=empty}"
        for certfile in "${certfiles[@]}"
        do
            if [ "${certfile##*.}" = "crt" ]; then
                echo $certfile | cut -c${#FIRMDIR}- | tee -a $FILE
                serialno=$(openssl x509 -in $certfile -serial -noout)
                echo $serialno | tee -a $FILE
                # Perform Shodan search. TOOD: Install Shodan CLI installed with an API key. 
                # serialnoformat=(ssl.cert.serial:${serialno##*=})
                # shocount=$(shodan count $serialnoformat)
                # echo "Number of devices found in Shodan =" $shocount | tee -a $FILE
                cat $certfile | tee -a $FILE
            else
                # all other SSL related files
                echo $certfile | cut -c${#FIRMDIR}- | tee -a $FILE
            fi
        done
    msg ""
done
msg ""
msg "#Searching for SSH related files#"
getArray "${PARENT_PATH}/data/sshfiles"
sshfiles=("${array[@]}")
for sshfile in ${sshfiles[@]}
do
    msg "# $sshfile"
    find $FIRMDIR -name $sshfile | cut -c${#FIRMDIR}- | tee -a $FILE
    msg ""
done
msg ""
msg "#Searching for configuration files#"
getArray "${PARENT_PATH}/data/conffiles"
conffiles=("${array[@]}")
for conffile in ${conffiles[@]}
do
    msg "# $conffile"
    find $FIRMDIR -name $conffile | cut -c${#FIRMDIR}- | tee -a $FILE
    msg ""
done
msg ""
msg "#Searching for database related files#"
getArray "${PARENT_PATH}/data/dbfiles"
dbfiles=("${array[@]}")
for dbfile in ${dbfiles[@]}
do
    msg " $dbfile"
    find $FIRMDIR -name $dbfile | cut -c${#FIRMDIR}- | tee -a $FILE
    msg ""
done
msg ""
msg "#Search for shell scripts#"
msg "#Shell scripts"
find $FIRMDIR -name "*.sh" | cut -c${#FIRMDIR}- | tee -a $FILE
msg ""
msg "#Search for other .bin files#"
msg "#bin files"
find $FIRMDIR -name "*.bin" | cut -c${#FIRMDIR}- | tee -a $FILE
msg ""
msg "#Search for patterns in files#"
getArray "${PARENT_PATH}/data/patterns"
patterns=("${array[@]}")
for pattern in "${patterns[@]}"
do
    msg "# $pattern"
    grep -lsirnw $FIRMDIR -e "$pattern" | cut -c${#FIRMDIR}- | tee -a $FILE
    msg ""
done
msg ""
msg "#Searching for web servers#"
getArray "${PARENT_PATH}/data/webservers"
webservers=("${array[@]}")
for webserver in ${webservers[@]}
do
    msg "# $webserver"
    find $FIRMDIR -name "$webserver" | cut -c${#FIRMDIR}- | tee -a $FILE
    msg ""
done
msg ""
msg "#Searching for debug / priv binaries#"
msg "#Debug / Priv binaries"
getArray "${PARENT_PATH}/data/binaries"
binaries=("${array[@]}")
for binary in "${binaries[@]}"
do
    msg "# $binary"
    find $FIRMDIR -name "$binary" | cut -c${#FIRMDIR}- | tee -a $FILE
    msg ""
done

msg ""
msg "#Searching for ip addresses#"
msg "#IP Addresses"
grep -sRIEh '[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}' --exclude='console' $FIRMDIR | sort | uniq | tee -a $FILE

msg ""
msg "#Searching for urls#"
msg "#URLs"
grep -sRIEo '(http|https)://[^/"]+' --exclude='console' $FIRMDIR | sort | uniq | tee -a $FILE

msg ""
msg "#Searching for emails#"
msg "#Emails"
grep -sRIEo '([[:alnum:]_.-]+@[[:alnum:]_.-]+?\.[[:alpha:].]{2,6})' "$@" --exclude='console' $FIRMDIR | sort | uniq | tee -a $FILE

#Perform static code analysis 
#eslint -c eslintrc.json $FIRMDIR | tee -a $FILE
