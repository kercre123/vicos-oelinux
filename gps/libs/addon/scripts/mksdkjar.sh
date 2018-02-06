#!/bin/bash
#==========================================================================
# Copyright (c) 2013, 2015, 2017 Qualcomm Technologies, Inc.
# All Rights Reserved.
# Confidential and Proprietary - Qualcomm Technologies, Inc.
#
# 2013 Qualcomm Atheros, Inc.
# All Rights Reserved.
# Qualcomm Atheros Confidential and Proprietary.
#
#==========================================================================
#
#==========================================================================
#
# Usage:
#     mksdkjar.sh [output-path]
#
# Note, this script requires the existence of the build binary -
# out/target/common/obj/JAVA_LIBRARIES/framework_intermediates/classes.jar
# This script can be called from any directory
# It's also called automatically from com.qti.location.sdk make file
#==========================================================================

DIR="$( cd "$( dirname "$0" )" && pwd )"
CMD=${0##*/}
OUT="/tmp/izatsdk"

if [ -n "$1" ]
then
    if [ "$1" == "-h" ] || [ "$1" == "--help" ]
    then
        echo "$0 [output-path]"
        echo "  e.g. $0"
        echo "       $0 /tmp/sdk"
        exit
    else
        OUT=$1
        if [ ! -d $OUT ]
        then
            mkdir -p $OUT
        fi
    fi
fi

if [ ! -e $DIR/../../../../../../../out/target/common/obj/JAVA_LIBRARIES/framework_intermediates/classes.jar ]
then
    echo "This script requires com.qualcomm.location frameworks to have been built."
    exit
fi

mkdir -p $OUT/classes
javac -cp $DIR/../../../../../../../out/target/common/obj/JAVA_LIBRARIES/frameworks-core-util-lib_intermediates/classes.jar:$DIR/../../../../../../../out/target/common/obj/JAVA_LIBRARIES/framework_intermediates/classes.jar -sourcepath $DIR/../java/:$DIR/../../glue/java -d $OUT/classes/ $(find $DIR/../java $DIR/../../../../../../../out/target/common/obj/JAVA_LIBRARIES/izat.lib.glue_intermediates/java/ $DIR/../../glue/java/com/qti/debugreport/ $DIR/../../glue/java/com/qti/wifidbreceiver/  $DIR/../../glue/java/com/qti/flp/ $DIR/../../glue/java/com/qti/geofence/ $DIR/../../glue/java/com/qti/izat/ $DIR/../../glue/java/com/qti/wifidbreceiver/ -name *.java)

jar cvf $OUT/izatsdk.jar -C $OUT/classes/ .

rm -rf $OUT/docs
mkdir -p $OUT/docs
javadoc -d $OUT/docs -sourcepath $DIR/../java/:$DIR/../../glue/java -exclude com.qualcomm.services.location -classpath $DIR/../../../../../../../out/target/common/obj/JAVA_LIBRARIES/frameworks-core-util-lib_intermediates/classes.jar:$DIR/../../../../../../../out/target/common/obj/JAVA_LIBRARIES/framework_intermediates/classes.jar:$OUT/izatsdk.jar com.qti.location.sdk $DIR/../../glue/java/com/qti/debugreport/*.java $DIR/../../glue/java/com/qti/wifidbreceiver/*.java

rm -rf $OUT/classes

tar cjvf $OUT/docs.tbz -C $OUT/ docs

echo "Java docs for sdk available at $OUT"
