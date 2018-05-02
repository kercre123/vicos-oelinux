#!/bin/bash -eu

VERBOSE=0
GIT=`which git`

if [ -z $GIT ];then
  echo git not found
  exit 1
fi

function usage() {
    echo "$0 [-h] [-b branch] [-s submodule location]"
    echo "-h                        : help. Print out this info"
    echo "-b branch                 : branch to switch to on submodule"
    echo "                              - default is master"
    echo "-s submodule_location     : location of submodule to change branch of."
    echo "                              - default is anki/victor"
    echo "-v                        : verbose output."
}

while getopts ":hs:b:v" opt; do
    case $opt in
        h)
            usage
            exit 0
            ;;
        b)
            VICTOR_ENGINE_BRANCH=$OPTARG
            ;;
        s)
            SUBMODULE_LOCATION=$OPTARG
            ;;
        v)
            VERBOSE=1
            set -x 
            ;;
        :)
            ;;
    esac
done

TOPLEVEL=`${GIT} rev-parse --show-toplevel`
if [ -z ${VICTOR_ENGINE_BRANCH+x} ]; then VICTOR_ENGINE_BRANCH=master; fi
if [ -z ${SUBMODULE_LOCATION+x} ]; then SUBMODULE_LOCATION=anki/victor/; fi

pushd ${TOPLEVEL}/${SUBMODULE_LOCATION}
${GIT} fetch
${GIT} checkout -f ${VICTOR_ENGINE_BRANCH}
${GIT} pull
${GIT} submodule update --init --recursive
if [ ${VERBOSE} -eq 1 ]; then $GIT log -3 --stat; fi
popd
