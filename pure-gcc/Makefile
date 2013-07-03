# List all source files the application uses.
APPLICATION_SRCS = ../main.c
APPLICATION_SRCS += ../ble_bls.c
APPLICATION_SRCS += $(notdir $(wildcard ../../libhexwriter/src/*.c))
APPLICATION_SRCS += ble_advdata.c
APPLICATION_SRCS += ble_srv_common.c
APPLICATION_SRCS += app_timer.c
APPLICATION_SRCS += ble_stack_handler.c
APPLICATION_SRCS += ble_conn_params.c

PROJECT_NAME = nrf51-ble-loader
START_CODE = bootloader_startup_nrf51.s

DEVICE = NRF51
BOARD = BOARD_PCA10001
USE_SOFTDEVICE = S110

LIBRARY_PATHS += ../../libhexwriter/inc/
SOURCE_PATHS += ../../libhexwriter/src/ 

CFLAGS = -O0 -g

SDK_PATH = ../../nrf51/nrf51822/

GDB_PORT_NUMBER = 2331

include $(SDK_PATH)Source/templates/pure-gcc/Makefile
