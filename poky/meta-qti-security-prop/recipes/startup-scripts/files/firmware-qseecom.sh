#!/bin/sh
#
# Copyright (c) 2013 Qualcomm Technologies, Inc. All Rights Reserved.
# Qualcomm Technologies Proprietary and Confidential.
#
# firmware-qseecomd.sh startup script to install the firmware links
#

# No path is set up at this point so we have to do it here.
PATH=/sbin:/bin:/usr/sbin:/usr/bin
export PATH

mkdir -p /firmware
mount -t vfat /dev/mmcblk0p1 /firmware

# Check for images and set up symlinks
cd /firmware/image

# Get the list of files in /firmware/image
# for which sym links have to be created

fwfiles=$(ls sample* cmnlib* playread* tzapps*)

# Check if the links with similar names
# have been created in /lib/firmware
if [ ! -d /lib/firmware ]; then
mkdir -p /lib/firmware
fi
cd /lib/firmware
linksNeeded=0

# For everyfile in fwfiles check if
# the corresponding file exists
for fwfile in $fwfiles
do
   # if (condition) does not seem to work
   # with the android shell. Therefore
   # make do with case statements instead.
   # if a file named $fwfile is present
   # no need to create links. If the file
   # with the name $fwfile is not present
   # need to create links.

   fw_file=$(ls $fwfile)
   if [ "$fw_file" == "$fwfile" ]
   then
      continue
   else
      linksNeeded=1
      break
   fi
done

case $linksNeeded in
   1)
      cd /firmware/image

      case `ls sampleapp.mdt 2>/dev/null` in
         sampleapp.mdt)
            for imgfile in sample*
            do
               ln -s /firmware/image/$imgfile /lib/firmware/$imgfile 2>/dev/null
               ln -s /firmware/image/$imgfile /etc/firmware/$imgfile 2>/dev/null
            done
            ;;
        *)
            # trying to log here but nothing will be logged since it is
            # early in the boot process. Is there a way to log this message?
            echo "PIL no sample image found"
            ;;
      esac

      case `ls cmnlib.mdt 2>/dev/null` in
         cmnlib.mdt)
            for imgfile in cmnlib*
            do
               ln -s /firmware/image/$imgfile /lib/firmware/$imgfile 2>/dev/null
               ln -s /firmware/image/$imgfile /etc/firmware/$imgfile 2>/dev/null
            done
            ;;
         *)
            echo "PIL no cmnlib image found"
            ;;
      esac
      case `ls playread.mdt 2>/dev/null` in
       playread.mdt)
	   for imgfile in playread*
	   do
	     ln -s /firmware/image/$imgfile /lib/firmware/$imgfile 2>/dev/null
	     ln -s /firmware/image/$imgfile /etc/firmware/$imgfile 2>/dev/null
	   done
	   ;;
	*)
	   echo "PIL no playread image found"
	   ;;
      esac
      case `ls tzapps.mdt 2>/dev/null` in
        tzapps.mdt)
	   for imgfile in tzapps*
	   do
	     ln -s /firmware/image/$imgfile /lib/firmware/$imgfile 2 >/dev/null
	     ln -s /firmware/image/$imgfile /etc/firmware/$imgfile 2 >/dev/null
	   done
	   ;;
	*)
	   echo "PIL no Tzapps image found"
	   ;;
      esac
       ;;

   *)
      echo "Nothing to do. No firmware links needed."
      ;;
esac

cd /
exit 0
