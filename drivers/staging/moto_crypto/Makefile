EXTRA_CFLAGS += -DCONFIG_CRYPTO_MOTOROLA_FAULT_INJECTION

override EXTRA_CFLAGS += \
    -fno-pic \
    -DKERNEL \
    -D_KERNEL \
    -I$(src)/include/

export PWD :=	$(shell pwd)

obj-y += moto_crypto.o
moto_crypto-y := \
    src/moto_crypto_main.o \
    src/moto_crypto_util.o \
    src/moto_testmgr.o \
    src/moto_aes.o \
    src/moto_tdes.o \
    src/moto_sha1.o \
    src/moto_sha256.o \
    src/moto_sha512.o \
    src/moto_hmac.o \
    src/moto_ansi_cprng.o
