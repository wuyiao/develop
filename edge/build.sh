#!/bin/bash

. ../../../../../scripts/environment-setup.sh

MAKE_VERBOSE_OPTIONS="QUIET=0 COLOR=0"
CMDLINE $@
CFLAGS+=" -I${INCDIR} -pg -DLOG -DLF -DLORAMAC_CLASSB_ENABLED -DSOFT_SE -DSECURE_ELEMENT_PRE_PROVISIONED -DREGION_CN470 -DACTIVE_REGION=LORAMAC_REGION_CN470"
LDFLAGS=" --hash-style=gnu --as-needed"
MAKE

if [ "$?" != 0 ]; then
    exit
fi

IPKG_DIR=ipkg
#${STRIP} edge
CV edge     ${ROOTFS_DIR}/usr/bin/
CV edge     ${IPKG_DIR}/edge/usr/bin/
BUILD_IPKG ${IPKG_DIR}/edge
