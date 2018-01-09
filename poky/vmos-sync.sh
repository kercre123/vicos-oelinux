mkdir -p build/tmp-glibc/deploy
rsync -avz --delete vmos:le1.0.2/apps_proc/poky/build/tmp-glibc/deploy/ build/tmp-glibc/deploy/
