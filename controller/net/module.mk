#ROOT PATH
ROOT = $(firstword $(subst eabi_env, eabi_env, $(PWD)))eabi_env


connector_SRCS := connector.c
connector_SRCS := $(addprefix $(TOPLEVEL)/connector/, $(connector_SRCS))

TARGETS += connector



