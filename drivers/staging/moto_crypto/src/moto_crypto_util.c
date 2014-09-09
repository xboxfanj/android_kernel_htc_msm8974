#include <linux/kernel.h>
#include <asm/byteorder.h>
#include "moto_crypto_util.h"

static inline void moto_crypto_inc_byte(u8 *a, unsigned int size)
{
    u8 *b = (a + size);
    u8 c;

    for (; size; size--) {
        c = *--b + 1;
        *b = c;
        if (c)
            break;
    }
}

void moto_crypto_inc(u8 *a, unsigned int size)
{
    __be32 *b = (__be32 *)(a + size);
    u32 c;

    for (; size >= 4; size -= 4) {
        c = be32_to_cpu(*--b) + 1;
        *b = cpu_to_be32(c);
        if (c)
            return;
    }

    moto_crypto_inc_byte(a, size);
}

static inline void moto_crypto_xor_byte(u8 *a, const u8 *b, unsigned int size)
{
    for (; size; size--)
        *a++ ^= *b++;
}

void moto_crypto_xor(u8 *dst, const u8 *src, unsigned int size)
{
    u32 *a = (u32 *)dst;
    u32 *b = (u32 *)src;

    for (; size >= 4; size -= 4)
        *a++ ^= *b++;

    moto_crypto_xor_byte((u8 *)a, (u8 *)b, size);
}

void moto_hexdump(unsigned char *buf, unsigned int len)
{
    print_hex_dump(KERN_CONT, "", DUMP_PREFIX_OFFSET,
            16, 1,
            buf, len, false);
}
