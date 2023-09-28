all: check_env
	make clean
	make controller


export TOPLEVEL=$(CURDIR)
export QUIET ?= 1
export COLOR ?= 1
export MAKEFLAGS+=--no-print-directory

dirs-y := icc/ net/ proto/
objs-y := main.o ubus.o config.o message.o utils.o 

CFLAGS-y = -g -O0 -Wall -I$(CURDIR)/include
CFLAGS+=$(CFLAGS-y)

LDFLAGS+=--no-gc-sections

LIBS-y = -luci -lubox -lubus -lmosquitto -lipc -lds -lcbor -ljson-c -lblobmsg_json
LIBS=$(LIBS-y)

include Makefile.rules

.SUFFIXES: .c

check_env:
ifndef CROSS_COMPILE
	$(error CROSS_COMPILE is not set)
endif

controller: $(objs-y)
	@$(CMD_STATUS) "\tLD $@"
	$(CMD_QUIET)$(CC) -o $@ $^  $(LIBS) -lpthread

autoconf: include/autoconf.h
include/autoconf.h: .config
	@echo "Rebuilding autoconf.h..."
	@cat .config | ./autconf.sh > include/autoconf.h

clean:
	@rm -f include/autoconf.h 
	@find ./ -name "*.o" | xargs rm -f
	@echo "Cleaned"

-include $(DEPENDS)

.PHONY: all clean $(dirs-y) autoconf
