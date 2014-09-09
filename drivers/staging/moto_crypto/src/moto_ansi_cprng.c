/*
 * PRNG: Pseudo Random Number Generator
 *       Based on NIST Recommended PRNG From ANSI X9.31 Appendix A.2.4 using
 *       AES 128 cipher
 *
 *  (C) Neil Horman <nhorman@tuxdriver.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or (at your
 *  any later version.
 *
 *
 */

#include <crypto/internal/rng.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/string.h>

#include "moto_testmgr.h"
#include "moto_crypto_util.h"

#define DEFAULT_PRNG_KEY "0123456789abcdef"
#define DEFAULT_PRNG_KSZ 16
#define DEFAULT_BLK_SZ 16
#define DEFAULT_V_SEED "zaybxcwdveuftgsh"

/*
 * Flags for the prng_context flags field
 */

#define PRNG_FIXED_SIZE 0x1
#define PRNG_NEED_RESET 0x2

/*
 * Note: DT is our counter value
 *	 I is our intermediate value
 *	 V is our seed vector
 * See http://csrc.nist.gov/groups/STM/cavp/documents/rng/931rngext.pdf
 * for implementation details
 */


struct moto_prng_context {
    spinlock_t prng_lock;
    unsigned char rand_data[DEFAULT_BLK_SZ];
    unsigned char last_rand_data[DEFAULT_BLK_SZ];
    unsigned char DT[DEFAULT_BLK_SZ];
    unsigned char I[DEFAULT_BLK_SZ];
    unsigned char V[DEFAULT_BLK_SZ];
    u32 rand_data_valid;
    struct crypto_blkcipher *tfm;
    u32 flags;
};

static int dbg = 0;

static void hexdump(char *note, unsigned char *buf, unsigned int len)
{
    if (dbg) {
        printk(KERN_CRIT "%s", note);
        print_hex_dump(KERN_CONT, "", DUMP_PREFIX_OFFSET,
                16, 1,
                buf, len, false);
    }
}

#define dbgprint(format, args...) do {\
        if (dbg)\
        printk(format, ##args);\
} while (0)

static void moto_xor_vectors(unsigned char *in1, unsigned char *in2,
        unsigned char *out, unsigned int size)
{
    int i;

    for (i = 0; i < size; i++)
        out[i] = in1[i] ^ in2[i];

}
/*
 * Returns DEFAULT_BLK_SZ bytes of random data per call
 * returns 0 if generation succeeded, <0 if something went wrong
 */
static int _moto_get_more_prng_bytes(struct moto_prng_context *ctx, 
        int cont_test)
{
    int i;
    unsigned char tmp[DEFAULT_BLK_SZ];
    unsigned char *output = NULL;
    struct blkcipher_desc desc;
    struct scatterlist sg;
    struct scatterlist sg_out;


    dbgprint(KERN_CRIT 
            "Calling _moto_get_more_prng_bytes for context %p\n",
            ctx);

    hexdump("Input DT: ", ctx->DT, DEFAULT_BLK_SZ);
    hexdump("Input I: ", ctx->I, DEFAULT_BLK_SZ);
    hexdump("Input V: ", ctx->V, DEFAULT_BLK_SZ);

    /*
     * This algorithm is a 3 stage state machine
     */
    for (i = 0; i < 3; i++) {

        switch (i) {
        case 0:
            /*
             * Start by encrypting the counter value
             * This gives us an intermediate value I
             */
            memcpy(tmp, ctx->DT, DEFAULT_BLK_SZ);
            output = ctx->I;
            hexdump("tmp stage 0: ", tmp, DEFAULT_BLK_SZ);
            break;
        case 1:

            /*
             * Next xor I with our secret vector V
             * encrypt that result to obtain our
             * pseudo random data which we output
             */
            moto_xor_vectors(ctx->I, ctx->V, tmp, DEFAULT_BLK_SZ);
            hexdump("tmp stage 1: ", tmp, DEFAULT_BLK_SZ);
            output = ctx->rand_data;
            break;
        case 2:
            /*
             * First check that we didn't produce the same
             * random data that we did last time around through this
             */
            if (!memcmp(ctx->rand_data, ctx->last_rand_data,
                    DEFAULT_BLK_SZ)) {
                if (cont_test) {
                    /* FSM_TRANS:T5 */
                    panic("cprng %p Failed repetition check!\n",
                            ctx);
                }

                printk(KERN_ERR
                        "ctx %p Failed repetition check!\n",
                        ctx);

                ctx->flags |= PRNG_NEED_RESET;
                return -EINVAL;
            }
            memcpy(ctx->last_rand_data, ctx->rand_data,
                    DEFAULT_BLK_SZ);

            /*
             * Lastly xor the random data with I
             * and encrypt that to obtain a new secret vector V
             */
            moto_xor_vectors(ctx->rand_data, ctx->I, tmp,
                    DEFAULT_BLK_SZ);
            output = ctx->V;
            hexdump("tmp stage 2: ", tmp, DEFAULT_BLK_SZ);
            break;
        }

        desc.tfm = ctx->tfm;
        desc.flags = 0;

        sg_init_one(&sg, tmp, DEFAULT_BLK_SZ);

        sg_init_one(&sg_out, output, DEFAULT_BLK_SZ);

        /* do the encryption */
        crypto_blkcipher_encrypt(&desc, &sg_out, &sg, DEFAULT_BLK_SZ);

    }

    /*
     * Now update our DT value
     */
    for (i = DEFAULT_BLK_SZ - 1; i >= 0; i--) {
        ctx->DT[i] += 1;
        if (ctx->DT[i] != 0)
            break;
    }

    dbgprint("Returning new block for context %p\n", ctx);
    ctx->rand_data_valid = 0;

    hexdump("Output DT: ", ctx->DT, DEFAULT_BLK_SZ);
    hexdump("Output I: ", ctx->I, DEFAULT_BLK_SZ);
    hexdump("Output V: ", ctx->V, DEFAULT_BLK_SZ);
    hexdump("New Random Data: ", ctx->rand_data, DEFAULT_BLK_SZ);

    return 0;
}

/* Our exported functions */
static int moto_get_prng_bytes(char *buf, size_t nbytes, 
        struct moto_prng_context *ctx, int do_cont_test)
{
    unsigned char *ptr = buf;
    unsigned int byte_count = (unsigned int)nbytes;
    int err;


    spin_lock_bh(&ctx->prng_lock);

    err = -EINVAL;
    if (ctx->flags & PRNG_NEED_RESET)
        goto done;

    /*
     * If the FIXED_SIZE flag is on, only return whole blocks of
     * pseudo random data
     */
    err = -EINVAL;
    if (ctx->flags & PRNG_FIXED_SIZE) {
        if (nbytes < DEFAULT_BLK_SZ)
            goto done;
        byte_count = DEFAULT_BLK_SZ;
    }

    err = byte_count;

    dbgprint(KERN_CRIT "getting %d random bytes for context %p\n",
            byte_count, ctx);


    remainder:
    if (ctx->rand_data_valid == DEFAULT_BLK_SZ) {
        if (_moto_get_more_prng_bytes(ctx, do_cont_test) < 0) {
            memset(buf, 0, nbytes);
            err = -EINVAL;
            goto done;
        }
    }

    /*
     * Copy any data less than an entire block
     */
    if (byte_count < DEFAULT_BLK_SZ) {
        empty_rbuf:
        for (; ctx->rand_data_valid < DEFAULT_BLK_SZ;
                ctx->rand_data_valid++) {
            *ptr = ctx->rand_data[ctx->rand_data_valid];
            ptr++;
            byte_count--;
            if (byte_count == 0)
                goto done;
        }
    }

    /*
     * Now copy whole blocks
     */
    for (; byte_count >= DEFAULT_BLK_SZ; byte_count -= DEFAULT_BLK_SZ) {
        if (ctx->rand_data_valid == DEFAULT_BLK_SZ) {
            if (_moto_get_more_prng_bytes(ctx, do_cont_test) < 0) {
                memset(buf, 0, nbytes);
                err = -EINVAL;
                goto done;
            }
        }
        if (ctx->rand_data_valid > 0)
            goto empty_rbuf;
        memcpy(ptr, ctx->rand_data, DEFAULT_BLK_SZ);
        ctx->rand_data_valid += DEFAULT_BLK_SZ;
        ptr += DEFAULT_BLK_SZ;
    }

    /*
     * Now go back and get any remaining partial block
     */
    if (byte_count)
        goto remainder;

    done:
    spin_unlock_bh(&ctx->prng_lock);
    dbgprint(KERN_CRIT "returning %d from get_prng_bytes in context %p\n",
            err, ctx);
    return err;
}

static void moto_free_prng_context(struct moto_prng_context *ctx)
{
    crypto_free_blkcipher(ctx->tfm);
}

static int moto_reset_prng_context(struct moto_prng_context *ctx,
        unsigned char *key, size_t klen,
        unsigned char *V, unsigned char *DT)
{
    int ret;
    unsigned char *prng_key;

    spin_lock_bh(&ctx->prng_lock);
    ctx->flags |= PRNG_NEED_RESET;

    prng_key = (key != NULL) ? key : (unsigned char *)DEFAULT_PRNG_KEY;

    if (!key)
        klen = DEFAULT_PRNG_KSZ;

    if (V)
        memcpy(ctx->V, V, DEFAULT_BLK_SZ);
    else
        memcpy(ctx->V, DEFAULT_V_SEED, DEFAULT_BLK_SZ);

    if (DT)
        memcpy(ctx->DT, DT, DEFAULT_BLK_SZ);
    else
        memset(ctx->DT, 0, DEFAULT_BLK_SZ);

    memset(ctx->rand_data, 0, DEFAULT_BLK_SZ);
    memset(ctx->last_rand_data, 0, DEFAULT_BLK_SZ);

    ctx->rand_data_valid = DEFAULT_BLK_SZ;

    ret = crypto_blkcipher_setkey(ctx->tfm, prng_key, klen);
    if (ret) {
        dbgprint(KERN_CRIT "PRNG: setkey() failed \n");
        goto out;
    }

    ret = 0;
    ctx->flags &= ~PRNG_NEED_RESET;
    out:
    spin_unlock_bh(&ctx->prng_lock);
    return ret;
}

static int moto_cprng_init(struct crypto_tfm *tfm)
{
    struct moto_prng_context *ctx = crypto_tfm_ctx(tfm);

    spin_lock_init(&ctx->prng_lock);
    ctx->tfm = crypto_alloc_blkcipher("ecb(aes)", 0, 0);
    if (IS_ERR(ctx->tfm)) {
        dbgprint(KERN_CRIT "Failed to alloc tfm for context %p\n",
                ctx);
        return PTR_ERR(ctx->tfm);
    }

    if (moto_reset_prng_context(ctx, NULL, DEFAULT_PRNG_KSZ, 
            NULL, NULL) < 0)
        return -EINVAL;

    /*
     * after allocation, we should always force the user to reset
     * so they don't inadvertently use the insecure default values
     * without specifying them intentially
     */
    ctx->flags |= PRNG_NEED_RESET;
    return 0;
}

static void moto_cprng_exit(struct crypto_tfm *tfm)
{
    struct moto_prng_context *ctx = crypto_tfm_ctx(tfm);

    moto_free_prng_context(ctx);
    memset(ctx->V, 0, DEFAULT_BLK_SZ);
#ifdef CONFIG_CRYPTO_MOTOROLA_SHOW_ZEROIZATION
    printk(KERN_INFO "PRNG seed after zeroization:\n");
    moto_hexdump(ctx->V, DEFAULT_BLK_SZ);
#endif

}

/*
 *  This is the cprng_registered reset method the seed value is
 *  interpreted as the tuple { V KEY DT}
 *  V and KEY are required during reset, and DT is optional, detected
 *  as being present by testing the length of the seed
 */
static int moto_cprng_reset(struct crypto_rng *tfm, u8 *seed, 
        unsigned int slen)
{
    struct moto_prng_context *prng = crypto_rng_ctx(tfm);
    u8 *key = seed + DEFAULT_BLK_SZ;
    u8 *dt = NULL;

    if (slen < DEFAULT_PRNG_KSZ + DEFAULT_BLK_SZ)
        return -EINVAL;

    if (slen >= (2 * DEFAULT_BLK_SZ + DEFAULT_PRNG_KSZ))
        dt = key + DEFAULT_PRNG_KSZ;

    /* Prevent seed and seed key from having the same value */
    if (!memcmp(seed, key, DEFAULT_BLK_SZ)) 
        return -EINVAL;

    moto_reset_prng_context(prng, key, DEFAULT_PRNG_KSZ, seed, dt);

    if (prng->flags & PRNG_NEED_RESET)
        return -EINVAL;
    return 0;
}

static int moto_fips_cprng_get_random(struct crypto_rng *tfm, u8 *rdata,
        unsigned int dlen)
{
    struct moto_prng_context *prng = crypto_rng_ctx(tfm);

    return moto_get_prng_bytes(rdata, dlen, prng, 1);
}

static int moto_fips_cprng_reset(struct crypto_rng *tfm, u8 *seed, 
        unsigned int slen)
{
    u8 rdata[DEFAULT_BLK_SZ];
    int rc;

    struct moto_prng_context *prng = crypto_rng_ctx(tfm);

    rc = moto_cprng_reset(tfm, seed, slen);

    if (!rc)
        goto out;

    /* this primes our continuity test */
    rc = moto_get_prng_bytes(rdata, DEFAULT_BLK_SZ, prng, 0);
    prng->rand_data_valid = DEFAULT_BLK_SZ;

    out:
    return rc;
}

static struct crypto_alg moto_fips_rng_alg = {
        .cra_name           = "ansi_cprng",
        .cra_driver_name    = "moto_fips_ansi_cprng",
        .cra_priority       = 1000,
        .cra_flags          = CRYPTO_ALG_TYPE_RNG,
        .cra_ctxsize        = sizeof(struct moto_prng_context),
        .cra_type           = &crypto_rng_type,
        .cra_module         = THIS_MODULE,
        .cra_list           = LIST_HEAD_INIT(moto_fips_rng_alg.cra_list),
        .cra_init           = moto_cprng_init,
        .cra_exit           = moto_cprng_exit,
        .cra_u              = {
                .rng = {
                        .rng_make_random    = moto_fips_cprng_get_random,
                        .rng_reset          = moto_fips_cprng_reset,
                        .seedsize           = DEFAULT_PRNG_KSZ + 2*DEFAULT_BLK_SZ,
                }
        }
};

int moto_prng_init(void)
{
    int rc = 0;

    rc = crypto_register_alg(&moto_fips_rng_alg);
    printk (KERN_INFO "moto_ansi_cprng register result: %d\n", rc);
    if (!rc) {
        rc = moto_alg_test("moto_fips_ansi_cprng", "ansi_cprng", 0, 0);
        printk (KERN_INFO "moto_ansi_cprng test result: %d\n", rc);
    } 
    return rc;
}

void moto_prng_finish(void)
{
    int err = 0;

    err = crypto_unregister_alg(&moto_fips_rng_alg);
    printk (KERN_INFO "moto_ansi_cprng unregister result: %d\n", err);
}
