APPLICATION = PV-Element
BOARD ?= berta-h10

BASE ?= $(CURDIR)/../../..
LORA3ABASE ?= $(BASE)/lora3a-boards
RIOTBASE ?= $(BASE)/RIOT

EXTERNAL_BOARD_DIRS=$(LORA3ABASE)/boards
EXTERNAL_MODULE_DIRS+=$(LORA3ABASE)/modules
EXTERNAL_PKG_DIRS=$(LORA3ABASE)/pkg

DEVELHELP ?= 1
QUIET ?= 1
PORT ?= /dev/ttyUSB0

USEMODULE += periph_adc

USEMODULE += hdc3020
USEMODULE += printf_float
USEMODULE += saml21_backup_mode

USEMODULE += senseair
USEMODULE += fram

# Sensorite portide määramine (korpusel eelistatud valik 1, 0)
HDC3020_PORT ?= 1
SENSAIR_PORT ?= 0

# Kontroll, et sensorid poleks samasse porti määratud
#ifeq ($(HDC3020_PORT), $(SENSAIR_PORT))
#abort:
#	@echo Viga: Sensorid on määratud samasse porti! 
#endif

# HDC3020 pordi seadistamine --------------------------------------------------------
ifeq ($(HDC3020_PORT), 0)
CFLAGS += -DACME0_BUS_MODE=MODE_I2C
CFLAGS += -DHDC3020_ENABLE_PIN=ACME0_POWER_PIN
CFLAGS += -DHDC3020_PARAM_I2C_DEV=ACME0_I2C_DEV
endif

ifeq ($(HDC3020_PORT), 1)
CFLAGS += -DACME1_BUS_MODE=MODE_I2C
CFLAGS += -DHDC3020_ENABLE_PIN=ACME1_POWER_PIN
CFLAGS += -DHDC3020_PARAM_I2C_DEV=ACME1_I2C_DEV
endif

ifeq ($(HDC3020_PORT), 2)
CFLAGS += -DACME2_BUS_MODE=MODE_I2C
CFLAGS += -DHDC3020_ENABLE_PIN=ACME2_POWER_PIN
CFLAGS += -DHDC3020_PARAM_I2C_DEV=ACME2_I2C_DEV
endif

# SENSAIR pordi seadistamine --------------------------------------------------------
ifeq ($(SENSAIR_PORT), 0)
CFLAGS += -DACME0_BUS_MODE=MODE_I2C
CFLAGS += -DSENSEAIR_I2C_DEV=ACME0_I2C_DEV -DSENSEAIR_ENABLE_PIN=GPIO_PIN\(PA,19\) -DACME_BUS_POWER_PIN=ACME0_POWER_PIN
endif

ifeq ($(SENSAIR_PORT), 1)
CFLAGS += -DACME1_BUS_MODE=MODE_I2C
CFLAGS += -DSENSEAIR_I2C_DEV=ACME1_I2C_DEV -DSENSEAIR_ENABLE_PIN=GPIO_PIN\(PB,23\) -DACME_BUS_POWER_PIN=ACME1_POWER_PIN
endif

ifeq ($(SENSAIR_PORT), 2)
CFLAGS += -DACME2_BUS_MODE=MODE_I2C
CFLAGS += -DSENSEAIR_I2C_DEV=ACME2_I2C_DEV -DSENSEAIR_ENABLE_PIN=GPIO_PIN\(PA,7\) -DACME_BUS_POWER_PIN=ACME2_POWER_PIN
endif


include $(RIOTBASE)/Makefile.include
include $(LORA3ABASE)/Makefile.include
