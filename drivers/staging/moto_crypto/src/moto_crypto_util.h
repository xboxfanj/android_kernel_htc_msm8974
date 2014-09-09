#ifndef _MOTO_CRYPTO_UTIL_H
#define _MOTO_CRYPTO_UTIL_H

#include <linux/types.h>

void moto_crypto_inc(u8 *a, unsigned int size);
void moto_crypto_xor(u8 *dst, const u8 *src, unsigned int size);
void moto_hexdump(unsigned char *buf, unsigned int len);

#endif
