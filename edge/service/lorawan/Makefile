#include $(TOPLEVEL)/.config

SRCDIR = common common/LmHandler common/LmHandler/packages
SRCCFILE := $(addsuffix /*.c,$(SRCDIR))
SRC = $(wildcard $(SRCCFILE))
CFLAGS += $(addprefix -I,$(SRCDIR)) \
		  -I../../driver/board/include \
		  -I../../driver/proto/region/include \
		  -I../../driver/proto/mac/include \
		  -I../../driver/system/include \
		  -I../../driver/proto/soft-se
dirs-y :=
obj-y := lorawan.o $(SRC:.c=.o)

include $(TOPLEVEL)/Makefile.rules
