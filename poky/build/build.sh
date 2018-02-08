#!/bin/bash

set -e

echo "Build starting at `date`"

SCRIPT_PATH=$(dirname $([ -L $0 ] && echo "$(dirname $0)/$(readlink -n $0)" || echo $0))
TOPLEVEL=$(cd "${SCRIPT_PATH}/../.." && pwd)

# remove any existing artifacts
pushd $TOPLEVEL/poky

pushd build
# known build artifact dirs
rm -rf bitbake.lock cache downloads  sstate-cache  tmp-glibc

# remove generated files under build/conf
git clean -xddf -- ./conf
popd

# Disable check for unset variables (it will make the bitbake scripts exit)
set +u
source build/conf/set_bb_env.sh
build-8009-robot-image

# Re-enable check for unset variables
set -u

echo "Build finished at `date`"

popd
