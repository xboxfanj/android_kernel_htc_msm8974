/*
 * Cryptographic API.
 *
 * HMAC: Keyed-Hashing for Message Authentication (RFC2104).
 *
 * Copyright (c) 2002 James Morris <jmorris@intercode.com.au>
 * Copyright (c) 2006 Herbert Xu <herbert@gondor.apana.org.au>
 *
 * The HMAC implementation is derived from USAGI.
 * Copyright (c) 2002 Kazunori Miyazawa <miyazawa@linux-ipv6.org> / USAGI
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 */

#include <crypto/internal/hash.h>
#include <crypto/scatterwalk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/string.h>

#include "moto_testmgr.h"

struct moto_hmac_ctx {
    struct crypto_shash *hash;
};

static inline void *align_ptr(void *p, unsigned int align)
{
    return (void *)ALIGN((unsigned long)p, align);
}

static inline struct moto_hmac_ctx *moto_hmac_ctx(struct crypto_shash *tfm)
{
    return align_ptr(crypto_shash_ctx_aligned(tfm) +
            crypto_shash_statesize(tfm) * 2,
            crypto_tfm_ctx_alignment());
}

static int moto_hmac_setkey(struct crypto_shash *parent,
        const u8 *inkey, unsigned int keylen)
{
    int bs = crypto_shash_blocksize(parent);
    int ds = crypto_shash_digestsize(parent);
    int ss = crypto_shash_statesize(parent);
    char *ipad = crypto_shash_ctx_aligned(parent);
    char *opad = ipad + ss;
    struct moto_hmac_ctx *ctx = align_ptr(opad + ss,
            crypto_tfm_ctx_alignment());
    struct crypto_shash *hash = ctx->hash;
    struct {
        struct shash_desc shash;
        char ctx[crypto_shash_descsize(hash)];
    } desc;
    unsigned int i;

    desc.shash.tfm = hash;
    desc.shash.flags = crypto_shash_get_flags(parent) &
            CRYPTO_TFM_REQ_MAY_SLEEP;

    if (keylen > bs) {
        int err;

        err = crypto_shash_digest(&desc.shash, inkey, keylen, ipad);
        if (err)
            return err;

        keylen = ds;
    } else
        memcpy(ipad, inkey, keylen);

    memset(ipad + keylen, 0, bs - keylen);
    memcpy(opad, ipad, bs);

    for (i = 0; i < bs; i++) {
        ipad[i] ^= 0x36;
        opad[i] ^= 0x5c;
    }

    return crypto_shash_init(&desc.shash) ?:
            crypto_shash_update(&desc.shash, ipad, bs) ?:
                    crypto_shash_export(&desc.shash, ipad) ?:
                            crypto_shash_init(&desc.shash) ?:
                                    crypto_shash_update(&desc.shash, opad, bs) ?:
                                            crypto_shash_export(&desc.shash, opad);
}

static int moto_hmac_export(struct shash_desc *pdesc, void *out)
{
    struct shash_desc *desc = shash_desc_ctx(pdesc);

    desc->flags = pdesc->flags & CRYPTO_TFM_REQ_MAY_SLEEP;

    return crypto_shash_export(desc, out);
}

static int moto_hmac_import(struct shash_desc *pdesc, const void *in)
{
    struct shash_desc *desc = shash_desc_ctx(pdesc);
    struct moto_hmac_ctx *ctx = moto_hmac_ctx(pdesc->tfm);

    desc->tfm = ctx->hash;
    desc->flags = pdesc->flags & CRYPTO_TFM_REQ_MAY_SLEEP;

    return crypto_shash_import(desc, in);
}

static int moto_hmac_init(struct shash_desc *pdesc)
{
    return moto_hmac_import(pdesc, crypto_shash_ctx_aligned(pdesc->tfm));
}

static int moto_hmac_update(struct shash_desc *pdesc,
        const u8 *data, unsigned int nbytes)
{
    struct shash_desc *desc = shash_desc_ctx(pdesc);

    desc->flags = pdesc->flags & CRYPTO_TFM_REQ_MAY_SLEEP;

    return crypto_shash_update(desc, data, nbytes);
}

static int moto_hmac_final(struct shash_desc *pdesc, u8 *out)
{
    struct crypto_shash *parent = pdesc->tfm;
    int ds = crypto_shash_digestsize(parent);
    int ss = crypto_shash_statesize(parent);
    char *opad = crypto_shash_ctx_aligned(parent) + ss;
    struct shash_desc *desc = shash_desc_ctx(pdesc);

    desc->flags = pdesc->flags & CRYPTO_TFM_REQ_MAY_SLEEP;

    return crypto_shash_final(desc, out) ?:
            crypto_shash_import(desc, opad) ?:
                    crypto_shash_finup(desc, out, ds, out);
}

static int moto_hmac_finup(struct shash_desc *pdesc, const u8 *data,
        unsigned int nbytes, u8 *out)
{

    struct crypto_shash *parent = pdesc->tfm;
    int ds = crypto_shash_digestsize(parent);
    int ss = crypto_shash_statesize(parent);
    char *opad = crypto_shash_ctx_aligned(parent) + ss;
    struct shash_desc *desc = shash_desc_ctx(pdesc);

    desc->flags = pdesc->flags & CRYPTO_TFM_REQ_MAY_SLEEP;

    return crypto_shash_finup(desc, data, nbytes, out) ?:
            crypto_shash_import(desc, opad) ?:
                    crypto_shash_finup(desc, out, ds, out);
}

static int moto_hmac_init_tfm(struct crypto_tfm *tfm)
{
    struct crypto_shash *parent = __crypto_shash_cast(tfm);
    struct crypto_shash *hash;
    struct crypto_instance *inst = (void *)tfm->__crt_alg;
    struct crypto_shash_spawn *spawn = crypto_instance_ctx(inst);
    struct moto_hmac_ctx *ctx = moto_hmac_ctx(parent);

    hash = crypto_spawn_shash(spawn);
    if (IS_ERR(hash))
        return PTR_ERR(hash);

    parent->descsize = sizeof(struct shash_desc) +
            crypto_shash_descsize(hash);

    ctx->hash = hash;
    return 0;
}

static void moto_hmac_exit_tfm(struct crypto_tfm *tfm)
{
    struct crypto_shash *parent = __crypto_shash_cast(tfm);
    struct moto_hmac_ctx *ctx = moto_hmac_ctx(parent);

    crypto_free_shash(ctx->hash);
}

static int moto_hmac_create(struct crypto_template *tmpl, struct rtattr **tb)
{
    struct shash_instance *inst;
    struct crypto_alg *alg;
    struct shash_alg *salg;
    int err;
    int ds;
    int ss;

    err = crypto_check_attr_type(tb, CRYPTO_ALG_TYPE_SHASH);
    if (err)
        return err;

    salg = shash_attr_alg(tb[1], 0, 0);
    if (IS_ERR(salg))
        return PTR_ERR(salg);

    err = -EINVAL;
    ds = salg->digestsize;
    ss = salg->statesize;
    alg = &salg->base;
    if (ds > alg->cra_blocksize ||
            ss < alg->cra_blocksize)
        goto out_put_alg;

    inst = shash_alloc_instance("moto_hmac", alg);
    err = PTR_ERR(inst);
    if (IS_ERR(inst))
        goto out_put_alg;

    err = crypto_init_shash_spawn(shash_instance_ctx(inst), salg,
            shash_crypto_instance(inst));
    if (err)
        goto out_free_inst;

    inst->alg.base.cra_priority = alg->cra_priority;
    inst->alg.base.cra_blocksize = alg->cra_blocksize;
    inst->alg.base.cra_alignmask = alg->cra_alignmask;

    ss = ALIGN(ss, alg->cra_alignmask + 1);
    inst->alg.digestsize = ds;
    inst->alg.statesize = ss;

    inst->alg.base.cra_ctxsize = sizeof(struct moto_hmac_ctx) +
            ALIGN(ss * 2, crypto_tfm_ctx_alignment());

    inst->alg.base.cra_init = moto_hmac_init_tfm;
    inst->alg.base.cra_exit = moto_hmac_exit_tfm;

    inst->alg.init = moto_hmac_init;
    inst->alg.update = moto_hmac_update;
    inst->alg.final = moto_hmac_final;
    inst->alg.finup = moto_hmac_finup;
    inst->alg.export = moto_hmac_export;
    inst->alg.import = moto_hmac_import;
    inst->alg.setkey = moto_hmac_setkey;

    err = shash_register_instance(tmpl, inst);
    if (err) {
        out_free_inst:
        shash_free_instance(shash_crypto_instance(inst));
    }

    out_put_alg:
    crypto_mod_put(alg);
    return err;
}

static struct crypto_template moto_hmac_tmpl = {
        .name   = "moto_hmac",
        .create = moto_hmac_create,
        .free   = shash_free_instance,
        .module = THIS_MODULE,
};

int moto_hmac_start(void)
{
    int err;

    err = crypto_register_template(&moto_hmac_tmpl);
    printk (KERN_INFO "moto_hmac register result: %d\n", err);
    if (!err) {
        err = moto_alg_test("moto_hmac(moto-sha1)", "moto_hmac(moto-sha1)", 0, 0);
        printk (KERN_INFO "moto_hmac(moto-sha1) test result: %d\n", err);
    }
    if (!err) {
        err = moto_alg_test("moto_hmac(moto-sha224)", "moto_hmac(moto-sha224)", 0, 0);
        printk (KERN_INFO "moto_hmac(moto-sha224) test result: %d\n", err);
    }
    if (!err) {
        err = moto_alg_test("moto_hmac(moto-sha256)", "moto_hmac(moto-sha256)", 0, 0);
        printk (KERN_INFO "moto_hmac(moto-sha256) test result: %d\n", err);
    }
    if (!err) {
        err = moto_alg_test("moto_hmac(moto-sha384)", "moto_hmac(moto-sha384)", 0, 0);
        printk (KERN_INFO "moto_hmac(moto-sha384) test result: %d\n", err);
    }
    if (!err) {
        err = moto_alg_test("moto_hmac(moto-sha512)", "moto_hmac(moto-sha512)", 0, 0);
        printk (KERN_INFO "moto_hmac(moto-sha512) test result: %d\n", err);
    }

    return err;
}

void moto_hmac_finish(void)
{
    printk (KERN_INFO "hmac finish\n");
}
