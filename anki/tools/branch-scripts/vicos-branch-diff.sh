#!/usr/bin/env bash
#
# anki/tools/branch-scripts/vicos-branch-diff.sh
#
# Helper script to list differences between master and release branch
#
# Exit for non-zero error
set -e
#
# Exit if a needed environment variable is not set
set -u

# Which branches are we comparing?
srcbranch=master
dstbranch=release/candidate

GIT=`which git`
if [ -z $GIT ]; then
  echo git not found
  exit 1
fi

TOPLEVEL=`$GIT rev-parse --show-toplevel`
SCRIPTDIR=$TOPLEVEL/anki/tools/branch-scripts

cd $TOPLEVEL
$SCRIPTDIR/anki-branch-diff.py $srcbranch $dstbranch
