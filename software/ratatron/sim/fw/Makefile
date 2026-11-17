# The robot firmware as a shared library for the MuJoCo simulator: the real application files and the gpio.c and
# infrared.c drivers, on the fake chip of sim_chip.c / sim_drivers.c.
#   make              competition goal (16x16 centre), with the development tools (CAL)
#   make PRACTICE=1   the 4x3 practice goal

FW       := ../../../../firmware/src
APP      := main motion control path search maze commands storage params crc32 telemetry calib gpio infrared
SIM      := sim_chip.c sim_drivers.c
PRACTICE ?= 0
BUILD    := build
LIB      := $(BUILD)/libratfw$(if $(filter 1,$(PRACTICE)),_practice,).so
CFLAGS   := -std=gnu11 -O2 -g -fPIC -Wall -Wextra -Wshadow -Wdouble-promotion -Wno-unused-parameter \
            -Ihal -I. -I$(FW) -DDEV_TOOLS=1 -DPRACTICE_MAZE=$(PRACTICE) -DSIM_CHIP=1

.PHONY: all clean
all: $(LIB)

$(LIB): $(addprefix $(FW)/,$(addsuffix .c,$(APP))) $(SIM) $(wildcard hal/*.h) sim_api.h $(wildcard $(FW)/*.h)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -shared -Wl,--no-undefined -o $@ $(filter %.c,$^) -Dmain=firmware_main -lm

clean:
	rm -rf $(BUILD)
