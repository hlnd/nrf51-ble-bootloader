# List all source files the application uses.
APPLICATION_SRCS = ../main.c
APPLICATION_SRCS += ../ble_bls.c
APPLICATION_SRCS += $(wildcard ../../libhexwriter/src/*.c)
APPLICATION_SRCS += ble_advdata.c
APPLICATION_SRCS += ble_srv_common.c
APPLICATION_SRCS += app_timer.c
APPLICATION_SRCS += ble_stack_handler.c
APPLICATION_SRCS += ble_conn_params.c

# Use shell to find name of root folder. Possible but horrible to do in make.
PROJECT_NAME = $(shell basename "$(realpath ../)")

DEVICE = NRF51
BOARD = BOARD_NRF6310
USE_SOFTDEVICE = S110

LIBRARY_PATHS += ../../libhexwriter/inc/
SOURCE_PATHS += ../../libhexwriter/src/ 

CFLAGS = -O0 -g

SDK_PATH = ../../nrf51/nrf51822/

GDB_PORT_NUMBER = 2331

include $(SDK_PATH)Source/templates/pure-gcc/Makefile
