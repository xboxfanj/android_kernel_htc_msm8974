/*
 * Algorithm testing framework and tests.
 *
 * Copyright (c) 2002 James Morris <jmorris@intercode.com.au>
 * Copyright (c) 2002 Jean-Francois Dive <jef@linuxbe.org>
 * Copyright (c) 2007 Nokia Siemens Networks
 * Copyright (c) 2008 Herbert Xu <herbert@gondor.apana.org.au>
 *
 * Updated RFC4106 AES-GCM testing. Some test vectors were taken from
 * http://csrc.nist.gov/groups/ST/toolkit/BCM/documents/proposedmodes/
 * gcm/gcm-test-vectors.tar.gz
 *     Authors: Aidan O'Mahony (aidan.o.mahony@intel.com)
 *              Adrian Hoban <adrian.hoban@intel.com>
 *              Gabriele Paoloni <gabriele.paoloni@intel.com>
 *              Tadeusz Struk (tadeusz.struk@intel.com)
 *     Copyright (c) 2010, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 */
#ifndef _MOTO_CRYPTO_TESTMGR_H
#define _MOTO_CRYPTO_TESTMGR_H

#include <linux/netlink.h>

#define MAX_DIGEST_SIZE     64
#define MAX_TAP             8

#define MAX_KEYLEN          56
#define MAX_IVLEN           32

struct moto_hash_testvec {
    /* only used with keyed hash algorithms */
    char *key;
    char *plaintext;
    char *digest;
    unsigned char tap[MAX_TAP];
    unsigned char psize;
    unsigned char np;
    unsigned char ksize;
};

struct moto_cipher_testvec {
    char *key;
    char *iv;
    char *input;
    char *result;
    unsigned short tap[MAX_TAP];
    int np;
    unsigned char fail;
    unsigned char wk; /* weak key flag */
    unsigned char klen;
    unsigned short ilen;
    unsigned short rlen;
};

struct moto_cprng_testvec {
    char *key;
    char *dt;
    char *v;
    char *result;
    unsigned char klen;
    unsigned short dtlen;
    unsigned short vlen;
    unsigned short rlen;
    unsigned short loops;
};

int moto_alg_test(const char *driver, const char *alg, u32 type, u32 mask);

#endif  /* _MOTO_CRYPTO_TESTMGR_H */
