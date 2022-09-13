#!/bin/bash

set -e

while getopts b: flag
do
    case "${flag}" in
	b) BRANCH=${OPTARG};;
    esac
done

case "${BRANCH}" in
    master)
	GIT_BRANCH=master
	GIT_BRANCH_CLOUD=main
	;;
    rc)
	GIT_BRANCH=release/candidate
	GIT_BRANCH_CLOUD=release/candidate
    ;;
    *)
	echo "BAD OPTION. Use '-b master' for dev build or '-b rc' for release candidate."
        exit 1
   ;;
esac


# Find where the global conf directory is...
scriptdir="$(dirname "${BASH_SOURCE}")"
# Find where the workspace is...
WS=$(readlink -f $scriptdir/../../..)

cd $WS

git checkout -f # Blast away any local changes
git checkout $GIT_BRANCH
git fetch
git merge origin/$GIT_BRANCH

pushd anki/victor
git checkout -f # Blast away any local changes
git checkout $GIT_BRANCH
git fetch
git merge origin/$GIT_BRANCH
popd

pushd anki/vector-cloud
git checkout -f # Blast away any local changes
git checkout $GIT_BRANCH_CLOUD
git fetch
git merge origin/$GIT_BRANCH_CLOUD
popd

cd $WS

git status | grep 'new commits'

GIT_STATUS=$?

echo $GIT_STATUS

if [ ${GIT_STATUS} -eq 0 ]; then
    echo "BAD SUBMODULE REFERENCES. UPDATING"
    git add anki/victor
    git add anki/vector-cloud
    git commit -m"Update submodules for automated build"
    git push origin $GIT_BRANCH
else
    echo "GOOD SUBMODULES. NO NEED TO FIX"
fi
