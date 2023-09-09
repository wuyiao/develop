#!/bin/bash

. ../../../../../scripts/environment-setup.sh

MAKE_VERBOSE_OPTIONS="QUIET=0 COLOR=0"

CMDLINE $@
CFLAGS+=" -I${INCDIR} -pg -DLOG -DLF"
LDFLAGS=" --hash-style=gnu --as-needed"
MAKE

if [ "$?" != 0 ]; then
    exit
fi

IPKG_DIR=ipkg

CV controller ${ROOTFS_DIR}/usr/bin/
CV controller ${IPKG_DIR}/controller/usr/bin
BUILD_IPKG ${IPKG_DIR}/controller
