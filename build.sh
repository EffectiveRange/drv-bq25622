#!/bin/bash
pushd $(dirname $0)
LAST_KVER=""
for kver in $(ls -1 /var/chroot/buildroot/lib/modules);
do
LAST_KVER=$kver
make driver KVER=$kver
done
make all KVER=$LAST_KVER