#include $(TOPLEVEL)/.config

dirs-y :=
obj-y := concentrator.o lm/lm.o lm/lm-log.o
CFLAGS += -I../../driver/wrapper/include \
		  -I../../lib/include \
		  -I lm

#-I../../driver/proto/soft-se 
include $(TOPLEVEL)/Makefile.rules
