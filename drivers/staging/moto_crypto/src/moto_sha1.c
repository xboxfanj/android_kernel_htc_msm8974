/*
 * Cryptographic API.
 *
 * SHA1 Secure Hash Algorithm.
 *
 * Derived from cryptoapi implementation, adapted for in-place
 * scatterlist interface.
 *
 * Copyright (c) Alan Smithee.
 * Copyright (c) Andrew McDonald <andrew@mcdonald.org.uk>
 * Copyright (c) Jean-Francois Dive <jef@linuxbe.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 */
#include <crypto/internal/hash.h>
#include <crypto/hash.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/types.h>
#include <moto_sha.h>
#include <asm/byteorder.h>

#include "moto_testmgr.h"
#include "moto_crypto_util.h"

/* The SHA f()-functions.  */

#define f1(x,y,z)   (z ^ (x & (y ^ z)))		/* x ? y : z */
#define f2(x,y,z)   (x ^ y ^ z)			/* XOR */
#define f3(x,y,z)   ((x & y) + (z & (x ^ y)))	/* majority */

/* The SHA Mysterious Constants */

#define K1  0x5A827999L			/* Rounds  0-19: sqrt(2) * 2^30 */
#define K2  0x6ED9EBA1L			/* Rounds 20-39: sqrt(3) * 2^30 */
#define K3  0x8F1BBCDCL			/* Rounds 40-59: sqrt(5) * 2^30 */
#define K4  0xCA62C1D6L			/* Rounds 60-79: sqrt(10) * 2^30 */

#define SHA_WORKSPACE_WORDS 80

/**
 * sha_transform - single block SHA1 transform
 *
 * @digest: 160 bit digest to update
 * @data:   512 bits of data to hash
 * @W:      80 words of workspace (see note)
 *
 * This function generates a SHA1 digest for a single 512-bit block.
 * Be warned, it does not handle padding and message digest, do not
 * confuse it with the full FIPS 180-1 digest algorithm for variable
 * length messages.
 *
 * Note: If the hash is security sensitive, the caller should be sure
 * to clear the workspace. This is left to the caller to avoid
 * unnecessary clears between chained hashing operations.
 */
void moto_sha_transform(__u32 *digest, const char *in, __u32 *W)
{
    __u32 a, b, c, d, e, t, i;

    for (i = 0; i < 16; i++)
        W[i] = be32_to_cpu(((const __be32 *)in)[i]);

    for (i = 0; i < 64; i++)
        W[i+16] = rol32(W[i+13] ^ W[i+8] ^ W[i+2] ^ W[i], 1);

    a = digest[0];
    b = digest[1];
    c = digest[2];
    d = digest[3];
    e = digest[4];

    for (i = 0; i < 20; i++) {
        t = f1(b, c, d) + K1 + rol32(a, 5) + e + W[i];
        e = d; d = c; c = rol32(b, 30); b = a; a = t;
    }

    for (; i < 40; i ++) {
        t = f2(b, c, d) + K2 + rol32(a, 5) + e + W[i];
        e = d; d = c; c = rol32(b, 30); b = a; a = t;
    }

    for (; i < 60; i ++) {
        t = f3(b, c, d) + K3 + rol32(a, 5) + e + W[i];
        e = d; d = c; c = rol32(b, 30); b = a; a = t;
    }

    for (; i < 80; i ++) {
        t = f2(b, c, d) + K4 + rol32(a, 5) + e + W[i];
        e = d; d = c; c = rol32(b, 30); b = a; a = t;
    }

    digest[0] += a;
    digest[1] += b;
    digest[2] += c;
    digest[3] += d;
    digest[4] += e;
}

static int moto_sha1_init(struct shash_desc *desc)
{
    struct moto_sha1_state *sctx = shash_desc_ctx(desc);

    *sctx = (struct moto_sha1_state){
        .state = { SHA1_H0, SHA1_H1, SHA1_H2, SHA1_H3, SHA1_H4 },
    };

    return 0;
}

static int moto_sha1_update(struct shash_desc *desc, const u8 *data,
        unsigned int len)
{
    struct moto_sha1_state *sctx = shash_desc_ctx(desc);
    unsigned int partial, done;
    const u8 *src;

    partial = sctx->count & 0x3f;
    sctx->count += len;
    done = 0;
    src = data;

    if ((partial + len) > 63) {
        u32 temp[SHA_WORKSPACE_WORDS];

        if (partial) {
            done = -partial;
            memcpy(sctx->buffer + partial, data, done + 64);
            src = sctx->buffer;
        }

        do {
            moto_sha_transform(sctx->state, src, temp);
            done += 64;
            src = data + done;
        } while (done + 63 < len);

        memset(temp, 0, sizeof(temp));
        partial = 0;
    }
    memcpy(sctx->buffer + partial, src, len - done);

    return 0;
}


/* Add padding and return the message digest. */
static int moto_sha1_final(struct shash_desc *desc, u8 *out)
{
    struct moto_sha1_state *sctx = shash_desc_ctx(desc);
    __be32 *dst = (__be32 *)out;
    u32 i, index, padlen;
    __be64 bits;
    static const u8 padding[64] = { 0x80, };

    bits = cpu_to_be64(sctx->count << 3);

    /* Pad out to 56 mod 64 */
    index = sctx->count & 0x3f;
    padlen = (index < 56) ? (56 - index) : ((64+56) - index);
    moto_sha1_update(desc, padding, padlen);

    /* Append length */
    moto_sha1_update(desc, (const u8 *)&bits, sizeof(bits));

    /* Store state in digest */
    for (i = 0; i < 5; i++)
        dst[i] = cpu_to_be32(sctx->state[i]);

    /* Wipe context */
    memset(sctx, 0, sizeof *sctx);
#ifdef CONFIG_CRYPTO_MOTOROLA_SHOW_ZEROIZATION
    printk(KERN_INFO "SHA1 context after zeroization:\n");
    moto_hexdump((unsigned char *)(sctx), sizeof *sctx);
#endif

    return 0;
}

static int moto_sha1_export(struct shash_desc *desc, void *out)
{
    struct moto_sha1_state *sctx = shash_desc_ctx(desc);

    memcpy(out, sctx, sizeof(*sctx));
    return 0;
}

static int moto_sha1_import(struct shash_desc *desc, const void *in)
{
    struct moto_sha1_state *sctx = shash_desc_ctx(desc);

    memcpy(sctx, in, sizeof(*sctx));
    return 0;
}

static struct shash_alg alg = {
        .digestsize = SHA1_DIGEST_SIZE,
        .init       = moto_sha1_init,
        .update     = moto_sha1_update,
        .final      = moto_sha1_final,
        .export     = moto_sha1_export,
        .import     = moto_sha1_import,
        .descsize   = sizeof(struct moto_sha1_state),
        .statesize  = sizeof(struct moto_sha1_state),
        .base       = {
                .cra_name       = "sha1",
                .cra_driver_name= "moto-sha1",
                .cra_flags      = CRYPTO_ALG_TYPE_SHASH,
                .cra_priority   = 1000,
                .cra_blocksize  = SHA1_BLOCK_SIZE,
                .cra_module     = THIS_MODULE,
        }
};

int moto_sha1_start(void)
{
    int err;

    err = crypto_register_shash(&alg);
    printk (KERN_INFO "sha1 register result: %d\n", err);
    if (!err) {
        err = moto_alg_test("moto-sha1", "sha1", 0, 0);
        printk (KERN_INFO "sha1 test result: %d\n", err);
    }
    return err;
}

void moto_sha1_finish(void)
{
    printk (KERN_INFO "sha1 finish\n");
}
