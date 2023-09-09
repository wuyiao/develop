all: check_env
	make clean
	make edge


export TOPLEVEL=$(CURDIR)
export QUIET ?= 1
export COLOR ?= 1

dirs-y := lib/ adapter/ device/ driver/ icc/ service/ softbus/

CFLAGS-y = -g -pg -O0 -Wall -I$(CURDIR)/include
CFLAGS+=$(CFLAGS-y)

LDFLAGS+=--no-gc-sections
export MAKEFLAGS+=--no-print-directory
#export MAKEFLAGS+=--trace

LIBS-y = -lm -luci -lubox -lubus -lmodbus -lusb-1.0 -lcbor -ljson-c -lipc -lds -lpthread
#LIBS-y = -L$(CONFIG_LIB_PATH) $(CONFIG_LIBS) -lcrypto
LIBS-$(CONFIG_DBUS) += -ldbus-1
LIBS=$(LIBS-y)

obj-y := main.o config.o ubus.o debug.o uart.o context.o lorae22.o

include Makefile.rules

.SUFFIXES: .c

.PHONY: all clean $(dirs-y) autoconf

check_env:
ifndef CROSS_COMPILE	
	$(error CROSS_COMPILE is not set)
endif

edge: $(obj-y)
	@$(CMD_STATUS) "\tLD $@"
	$(CMD_QUIET)$(CC) -pg -o $@ $^  $(LIBS)

autoconf: include/autoconf.h
include/autoconf.h: .config
	@echo "Rebuilding autoconf.h..."
	@cat .config | ./autconf.sh > include/autoconf.h

clean:
	@rm -f include/autoconf.h 
	@find ./ -name "*.o" | xargs rm -f
	@echo "Cleaned"

-include $(DEPENDS)


