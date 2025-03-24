all : flash

TARGET:=main
CH32V003FUN ?= ./ch32fun/ch32fun
TARGET_MCU ?= CH32V003

ADDITIONAL_C_FILES += $(filter-out $(TARGET).c, $(wildcard *.c))


include $(CH32V003FUN)/ch32fun.mk

flash : cv_flash
clean : cv_clean

