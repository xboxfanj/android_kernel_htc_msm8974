/*
 * Algorithm testing framework and tests.
 *
 * Copyright (c) 2002 James Morris <jmorris@intercode.com.au>
 * Copyright (c) 2002 Jean-Francois Dive <jef@linuxbe.org>
 * Copyright (c) 2007 Nokia Siemens Networks
 * Copyright (c) 2008 Herbert Xu <herbert@gondor.apana.org.au>
 *
 * Updated RFC4106 AES-GCM testing.
 *    Authors: Aidan O'Mahony (aidan.o.mahony@intel.com)
 *             Adrian Hoban <adrian.hoban@intel.com>
 *             Gabriele Paoloni <gabriele.paoloni@intel.com>
 *             Tadeusz Struk (tadeusz.struk@intel.com)
 *    Copyright (c) 2010, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 */

#include <crypto/hash.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <crypto/rng.h>

#include "moto_testmgr.h"
#include "moto_crypto_main.h"
#include "moto_crypto_util.h"

/*
 * Need slab memory for testing (size in number of pages).
 */
#define XBUFSIZE	8

/*
 * Indexes into the xbuf to simulate cross-page access.
 */
#define IDX1		32
#define IDX2		32400
#define IDX3		1
#define IDX4		8193
#define IDX5		22222
#define IDX6		17101
#define IDX7		27333
#define IDX8		3000

/*
 * Used by moto_test_cipher()
 */
#define ENCRYPT 1
#define DECRYPT 0

#define INJECT_FAULT_ALL_KEY_LENGHTS -1

/*
 * SHA1 test vectors  from from FIPS PUB 180-1
 * Long vector from CAVS 5.0
 */
#define SHA1_TEST_VECTORS	3

static struct moto_hash_testvec moto_sha1_tv_template[] = {
        {
                .plaintext = "abc",
                .psize	= 3,
                .digest	= "\xa9\x99\x3e\x36\x47\x06\x81\x6a\xba\x3e"
                        "\x25\x71\x78\x50\xc2\x6c\x9c\xd0\xd8\x9d",
        }, {
                .plaintext = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq",
                .psize	= 56,
                .digest	= "\x84\x98\x3e\x44\x1c\x3b\xd2\x6e\xba\xae"
                        "\x4a\xa1\xf9\x51\x29\xe5\xe5\x46\x70\xf1",
                        .np	= 2,
                        .tap	= { 28, 28 }
        }, {
                .plaintext = "\xec\x29\x56\x12\x44\xed\xe7\x06"
                        "\xb6\xeb\x30\xa1\xc3\x71\xd7\x44"
                        "\x50\xa1\x05\xc3\xf9\x73\x5f\x7f"
                        "\xa9\xfe\x38\xcf\x67\xf3\x04\xa5"
                        "\x73\x6a\x10\x6e\x92\xe1\x71\x39"
                        "\xa6\x81\x3b\x1c\x81\xa4\xf3\xd3"
                        "\xfb\x95\x46\xab\x42\x96\xfa\x9f"
                        "\x72\x28\x26\xc0\x66\x86\x9e\xda"
                        "\xcd\x73\xb2\x54\x80\x35\x18\x58"
                        "\x13\xe2\x26\x34\xa9\xda\x44\x00"
                        "\x0d\x95\xa2\x81\xff\x9f\x26\x4e"
                        "\xcc\xe0\xa9\x31\x22\x21\x62\xd0"
                        "\x21\xcc\xa2\x8d\xb5\xf3\xc2\xaa"
                        "\x24\x94\x5a\xb1\xe3\x1c\xb4\x13"
                        "\xae\x29\x81\x0f\xd7\x94\xca\xd5"
                        "\xdf\xaf\x29\xec\x43\xcb\x38\xd1"
                        "\x98\xfe\x4a\xe1\xda\x23\x59\x78"
                        "\x02\x21\x40\x5b\xd6\x71\x2a\x53"
                        "\x05\xda\x4b\x1b\x73\x7f\xce\x7c"
                        "\xd2\x1c\x0e\xb7\x72\x8d\x08\x23"
                        "\x5a\x90\x11",
                        .psize	= 163,
                        .digest	= "\x97\x01\x11\xc4\xe7\x7b\xcc\x88\xcc\x20"
                                "\x45\x9c\x02\xb6\x9b\x4a\xa8\xf5\x82\x17",
                                .np	= 4,
                                .tap	= { 63, 64, 31, 5 }
        }
};


/*
 * SHA224 test vectors from from FIPS PUB 180-2
 */
#define SHA224_TEST_VECTORS     2

static struct moto_hash_testvec moto_sha224_tv_template[] = {
        {
                .plaintext = "abc",
                .psize  = 3,
                .digest = "\x23\x09\x7D\x22\x34\x05\xD8\x22"
                        "\x86\x42\xA4\x77\xBD\xA2\x55\xB3"
                        "\x2A\xAD\xBC\xE4\xBD\xA0\xB3\xF7"
                        "\xE3\x6C\x9D\xA7",
        }, {
                .plaintext =
                        "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq",
                        .psize  = 56,
                        .digest = "\x75\x38\x8B\x16\x51\x27\x76\xCC"
                                "\x5D\xBA\x5D\xA1\xFD\x89\x01\x50"
                                "\xB0\xC6\x45\x5C\xB4\xF5\x8B\x19"
                                "\x52\x52\x25\x25",
                                .np     = 2,
                                .tap    = { 28, 28 }
        }
};

/*
 * SHA256 test vectors from from NIST
 */
#define SHA256_TEST_VECTORS	2

static struct moto_hash_testvec moto_sha256_tv_template[] = {
        {
                .plaintext = "abc",
                .psize	= 3,
                .digest	= "\xba\x78\x16\xbf\x8f\x01\xcf\xea"
                        "\x41\x41\x40\xde\x5d\xae\x22\x23"
                        "\xb0\x03\x61\xa3\x96\x17\x7a\x9c"
                        "\xb4\x10\xff\x61\xf2\x00\x15\xad",
        }, {
                .plaintext = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq",
                .psize	= 56,
                .digest	= "\x24\x8d\x6a\x61\xd2\x06\x38\xb8"
                        "\xe5\xc0\x26\x93\x0c\x3e\x60\x39"
                        "\xa3\x3c\xe4\x59\x64\xff\x21\x67"
                        "\xf6\xec\xed\xd4\x19\xdb\x06\xc1",
                        .np	= 2,
                        .tap	= { 28, 28 }
        },
};

/*
 * SHA384 test vectors from from NIST and kerneli
 */
#define SHA384_TEST_VECTORS	4

static struct moto_hash_testvec moto_sha384_tv_template[] = {
        {
                .plaintext= "abc",
                .psize	= 3,
                .digest	= "\xcb\x00\x75\x3f\x45\xa3\x5e\x8b"
                        "\xb5\xa0\x3d\x69\x9a\xc6\x50\x07"
                        "\x27\x2c\x32\xab\x0e\xde\xd1\x63"
                        "\x1a\x8b\x60\x5a\x43\xff\x5b\xed"
                        "\x80\x86\x07\x2b\xa1\xe7\xcc\x23"
                        "\x58\xba\xec\xa1\x34\xc8\x25\xa7",
        }, {
                .plaintext = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq",
                .psize	= 56,
                .digest	= "\x33\x91\xfd\xdd\xfc\x8d\xc7\x39"
                        "\x37\x07\xa6\x5b\x1b\x47\x09\x39"
                        "\x7c\xf8\xb1\xd1\x62\xaf\x05\xab"
                        "\xfe\x8f\x45\x0d\xe5\xf3\x6b\xc6"
                        "\xb0\x45\x5a\x85\x20\xbc\x4e\x6f"
                        "\x5f\xe9\x5b\x1f\xe3\xc8\x45\x2b",
        }, {
                .plaintext = "abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmn"
                        "hijklmnoijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu",
                        .psize	= 112,
                        .digest	= "\x09\x33\x0c\x33\xf7\x11\x47\xe8"
                                "\x3d\x19\x2f\xc7\x82\xcd\x1b\x47"
                                "\x53\x11\x1b\x17\x3b\x3b\x05\xd2"
                                "\x2f\xa0\x80\x86\xe3\xb0\xf7\x12"
                                "\xfc\xc7\xc7\x1a\x55\x7e\x2d\xb9"
                                "\x66\xc3\xe9\xfa\x91\x74\x60\x39",
        }, {
                .plaintext = "abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcd"
                        "efghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyz",
                        .psize	= 104,
                        .digest	= "\x3d\x20\x89\x73\xab\x35\x08\xdb"
                                "\xbd\x7e\x2c\x28\x62\xba\x29\x0a"
                                "\xd3\x01\x0e\x49\x78\xc1\x98\xdc"
                                "\x4d\x8f\xd0\x14\xe5\x82\x82\x3a"
                                "\x89\xe1\x6f\x9b\x2a\x7b\xbc\x1a"
                                "\xc9\x38\xe2\xd1\x99\xe8\xbe\xa4",
                                .np	= 4,
                                .tap	= { 26, 26, 26, 26 }
        },
};

/*
 * SHA512 test vectors from from NIST and kerneli
 */
#define SHA512_TEST_VECTORS	4

static struct moto_hash_testvec moto_sha512_tv_template[] = {
        {
                .plaintext = "abc",
                .psize	= 3,
                .digest	= "\xdd\xaf\x35\xa1\x93\x61\x7a\xba"
                        "\xcc\x41\x73\x49\xae\x20\x41\x31"
                        "\x12\xe6\xfa\x4e\x89\xa9\x7e\xa2"
                        "\x0a\x9e\xee\xe6\x4b\x55\xd3\x9a"
                        "\x21\x92\x99\x2a\x27\x4f\xc1\xa8"
                        "\x36\xba\x3c\x23\xa3\xfe\xeb\xbd"
                        "\x45\x4d\x44\x23\x64\x3c\xe8\x0e"
                        "\x2a\x9a\xc9\x4f\xa5\x4c\xa4\x9f",
        }, {
                .plaintext = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq",
                .psize	= 56,
                .digest	= "\x20\x4a\x8f\xc6\xdd\xa8\x2f\x0a"
                        "\x0c\xed\x7b\xeb\x8e\x08\xa4\x16"
                        "\x57\xc1\x6e\xf4\x68\xb2\x28\xa8"
                        "\x27\x9b\xe3\x31\xa7\x03\xc3\x35"
                        "\x96\xfd\x15\xc1\x3b\x1b\x07\xf9"
                        "\xaa\x1d\x3b\xea\x57\x78\x9c\xa0"
                        "\x31\xad\x85\xc7\xa7\x1d\xd7\x03"
                        "\x54\xec\x63\x12\x38\xca\x34\x45",
        }, {
                .plaintext = "abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmn"
                        "hijklmnoijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu",
                        .psize	= 112,
                        .digest	= "\x8e\x95\x9b\x75\xda\xe3\x13\xda"
                                "\x8c\xf4\xf7\x28\x14\xfc\x14\x3f"
                                "\x8f\x77\x79\xc6\xeb\x9f\x7f\xa1"
                                "\x72\x99\xae\xad\xb6\x88\x90\x18"
                                "\x50\x1d\x28\x9e\x49\x00\xf7\xe4"
                                "\x33\x1b\x99\xde\xc4\xb5\x43\x3a"
                                "\xc7\xd3\x29\xee\xb6\xdd\x26\x54"
                                "\x5e\x96\xe5\x5b\x87\x4b\xe9\x09",
        }, {
                .plaintext = "abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcd"
                        "efghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyz",
                        .psize	= 104,
                        .digest	= "\x93\x0d\x0c\xef\xcb\x30\xff\x11"
                                "\x33\xb6\x89\x81\x21\xf1\xcf\x3d"
                                "\x27\x57\x8a\xfc\xaf\xe8\x67\x7c"
                                "\x52\x57\xcf\x06\x99\x11\xf7\x5d"
                                "\x8f\x58\x31\xb5\x6e\xbf\xda\x67"
                                "\xb2\x78\xe6\x6d\xff\x8b\x84\xfe"
                                "\x2b\x28\x70\xf7\x42\xa5\x80\xd8"
                                "\xed\xb4\x19\x87\x23\x28\x50\xc9",
                                .np	= 4,
                                .tap	= { 26, 26, 26, 26 }
        },
};

/*
 * HMAC-SHA1 test vectors from RFC2202
 */
#define HMAC_SHA1_TEST_VECTORS	7

static struct moto_hash_testvec moto_hmac_sha1_tv_template[] = {
        {
                .key	= "\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b",
                .ksize	= 20,
                .plaintext = "Hi There",
                .psize	= 8,
                .digest	= "\xb6\x17\x31\x86\x55\x05\x72\x64"
                        "\xe2\x8b\xc0\xb6\xfb\x37\x8c\x8e\xf1"
                        "\x46\xbe",
        }, {
                .key	= "Jefe",
                .ksize	= 4,
                .plaintext = "what do ya want for nothing?",
                .psize	= 28,
                .digest	= "\xef\xfc\xdf\x6a\xe5\xeb\x2f\xa2\xd2\x74"
                        "\x16\xd5\xf1\x84\xdf\x9c\x25\x9a\x7c\x79",
                        .np	= 2,
                        .tap	= { 14, 14 }
        }, {
                .key	= "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa",
                .ksize	= 20,
                .plaintext = "\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd"
                        "\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd"
                        "\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd"
                        "\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd",
                        .psize	= 50,
                        .digest	= "\x12\x5d\x73\x42\xb9\xac\x11\xcd\x91\xa3"
                                "\x9a\xf4\x8a\xa1\x7b\x4f\x63\xf1\x75\xd3",
        }, {
                .key	= "\x01\x02\x03\x04\x05\x06\x07\x08"
                        "\x09\x0a\x0b\x0c\x0d\x0e\x0f\x10"
                        "\x11\x12\x13\x14\x15\x16\x17\x18\x19",
                        .ksize	= 25,
                        .plaintext = "\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd"
                                "\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd"
                                "\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd"
                                "\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd",
                                .psize	= 50,
                                .digest	= "\x4c\x90\x07\xf4\x02\x62\x50\xc6\xbc\x84"
                                        "\x14\xf9\xbf\x50\xc8\x6c\x2d\x72\x35\xda",
        }, {
                .key	= "\x0c\x0c\x0c\x0c\x0c\x0c\x0c\x0c\x0c\x0c\x0c\x0c\x0c\x0c\x0c\x0c\x0c\x0c\x0c\x0c",
                .ksize	= 20,
                .plaintext = "Test With Truncation",
                .psize	= 20,
                .digest	= "\x4c\x1a\x03\x42\x4b\x55\xe0\x7f\xe7\xf2"
                        "\x7b\xe1\xd5\x8b\xb9\x32\x4a\x9a\x5a\x04",
        }, {
                .key	= "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa",
                        .ksize	= 80,
                        .plaintext = "Test Using Larger Than Block-Size Key - Hash Key First",
                        .psize	= 54,
                        .digest	= "\xaa\x4a\xe5\xe1\x52\x72\xd0\x0e\x95\x70"
                                "\x56\x37\xce\x8a\x3b\x55\xed\x40\x21\x12",
        }, {
                .key	= "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa",
                        .ksize	= 80,
                        .plaintext = "Test Using Larger Than Block-Size Key and Larger Than One "
                                "Block-Size Data",
                                .psize	= 73,
                                .digest	= "\xe8\xe9\x9d\x0f\x45\x23\x7d\x78\x6d\x6b"
                                        "\xba\xa7\x96\x5c\x78\x08\xbb\xff\x1a\x91",
        },
};


/*
 * SHA224 HMAC test vectors from RFC4231
 */
#define HMAC_SHA224_TEST_VECTORS    4

static struct moto_hash_testvec moto_hmac_sha224_tv_template[] = {
        {
                .key    = "\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b"
                        "\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b"
                        "\x0b\x0b\x0b\x0b",
                        .ksize  = 20,
                        /*  ("Hi There") */
                        .plaintext = "\x48\x69\x20\x54\x68\x65\x72\x65",
                        .psize  = 8,
                        .digest = "\x89\x6f\xb1\x12\x8a\xbb\xdf\x19"
                                "\x68\x32\x10\x7c\xd4\x9d\xf3\x3f"
                                "\x47\xb4\xb1\x16\x99\x12\xba\x4f"
                                "\x53\x68\x4b\x22",
        }, {
                .key    = "Jefe",
                .ksize  = 4,
                /* ("what do ya want for nothing?") */
                .plaintext = "\x77\x68\x61\x74\x20\x64\x6f\x20"
                        "\x79\x61\x20\x77\x61\x6e\x74\x20"
                        "\x66\x6f\x72\x20\x6e\x6f\x74\x68"
                        "\x69\x6e\x67\x3f",
                        .psize  = 28,
                        .digest = "\xa3\x0e\x01\x09\x8b\xc6\xdb\xbf"
                                "\x45\x69\x0f\x3a\x7e\x9e\x6d\x0f"
                                "\x8b\xbe\xa2\xa3\x9e\x61\x48\x00"
                                "\x8f\xd0\x5e\x44",
                                .np = 4,
                                .tap    = { 7, 7, 7, 7 }
        }, {
                .key    = "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa",
                        .ksize  = 131,
                        /* ("Test Using Larger Than Block-Size Key - Hash Key First") */
                        .plaintext = "\x54\x65\x73\x74\x20\x55\x73\x69"
                                "\x6e\x67\x20\x4c\x61\x72\x67\x65"
                                "\x72\x20\x54\x68\x61\x6e\x20\x42"
                                "\x6c\x6f\x63\x6b\x2d\x53\x69\x7a"
                                "\x65\x20\x4b\x65\x79\x20\x2d\x20"
                                "\x48\x61\x73\x68\x20\x4b\x65\x79"
                                "\x20\x46\x69\x72\x73\x74",
                                .psize  = 54,
                                .digest = "\x95\xe9\xa0\xdb\x96\x20\x95\xad"
                                        "\xae\xbe\x9b\x2d\x6f\x0d\xbc\xe2"
                                        "\xd4\x99\xf1\x12\xf2\xd2\xb7\x27"
                                        "\x3f\xa6\x87\x0e",
        }, {
                .key    = "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa",
                        .ksize  = 131,
                        /* ("This is a test using a larger than block-size key and a")
		(" larger than block-size data. The key needs to be")
			(" hashed before being used by the HMAC algorithm.") */
                        .plaintext = "\x54\x68\x69\x73\x20\x69\x73\x20"
                                "\x61\x20\x74\x65\x73\x74\x20\x75"
                                "\x73\x69\x6e\x67\x20\x61\x20\x6c"
                                "\x61\x72\x67\x65\x72\x20\x74\x68"
                                "\x61\x6e\x20\x62\x6c\x6f\x63\x6b"
                                "\x2d\x73\x69\x7a\x65\x20\x6b\x65"
                                "\x79\x20\x61\x6e\x64\x20\x61\x20"
                                "\x6c\x61\x72\x67\x65\x72\x20\x74"
                                "\x68\x61\x6e\x20\x62\x6c\x6f\x63"
                                "\x6b\x2d\x73\x69\x7a\x65\x20\x64"
                                "\x61\x74\x61\x2e\x20\x54\x68\x65"
                                "\x20\x6b\x65\x79\x20\x6e\x65\x65"
                                "\x64\x73\x20\x74\x6f\x20\x62\x65"
                                "\x20\x68\x61\x73\x68\x65\x64\x20"
                                "\x62\x65\x66\x6f\x72\x65\x20\x62"
                                "\x65\x69\x6e\x67\x20\x75\x73\x65"
                                "\x64\x20\x62\x79\x20\x74\x68\x65"
                                "\x20\x48\x4d\x41\x43\x20\x61\x6c"
                                "\x67\x6f\x72\x69\x74\x68\x6d\x2e",
                                .psize  = 152,
                                .digest = "\x3a\x85\x41\x66\xac\x5d\x9f\x02"
                                        "\x3f\x54\xd5\x17\xd0\xb3\x9d\xbd"
                                        "\x94\x67\x70\xdb\x9c\x2b\x95\xc9"
                                        "\xf6\xf5\x65\xd1",
        },
};

/*
 * HMAC-SHA256 test vectors from
 * draft-ietf-ipsec-ciph-sha-256-01.txt
 */
#define HMAC_SHA256_TEST_VECTORS	10

static struct moto_hash_testvec moto_hmac_sha256_tv_template[] = {
        {
                .key	= "\x01\x02\x03\x04\x05\x06\x07\x08"
                        "\x09\x0a\x0b\x0c\x0d\x0e\x0f\x10"
                        "\x11\x12\x13\x14\x15\x16\x17\x18"
                        "\x19\x1a\x1b\x1c\x1d\x1e\x1f\x20",
                        .ksize	= 32,
                        .plaintext = "abc",
                        .psize	= 3,
                        .digest	= "\xa2\x1b\x1f\x5d\x4c\xf4\xf7\x3a"
                                "\x4d\xd9\x39\x75\x0f\x7a\x06\x6a"
                                "\x7f\x98\xcc\x13\x1c\xb1\x6a\x66"
                                "\x92\x75\x90\x21\xcf\xab\x81\x81",
        }, {
                .key	= "\x01\x02\x03\x04\x05\x06\x07\x08"
                        "\x09\x0a\x0b\x0c\x0d\x0e\x0f\x10"
                        "\x11\x12\x13\x14\x15\x16\x17\x18"
                        "\x19\x1a\x1b\x1c\x1d\x1e\x1f\x20",
                        .ksize	= 32,
                        .plaintext = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq",
                        .psize	= 56,
                        .digest	= "\x10\x4f\xdc\x12\x57\x32\x8f\x08"
                                "\x18\x4b\xa7\x31\x31\xc5\x3c\xae"
                                "\xe6\x98\xe3\x61\x19\x42\x11\x49"
                                "\xea\x8c\x71\x24\x56\x69\x7d\x30",
        }, {
                .key	= "\x01\x02\x03\x04\x05\x06\x07\x08"
                        "\x09\x0a\x0b\x0c\x0d\x0e\x0f\x10"
                        "\x11\x12\x13\x14\x15\x16\x17\x18"
                        "\x19\x1a\x1b\x1c\x1d\x1e\x1f\x20",
                        .ksize	= 32,
                        .plaintext = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"
                                "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq",
                                .psize	= 112,
                                .digest	= "\x47\x03\x05\xfc\x7e\x40\xfe\x34"
                                        "\xd3\xee\xb3\xe7\x73\xd9\x5a\xab"
                                        "\x73\xac\xf0\xfd\x06\x04\x47\xa5"
                                        "\xeb\x45\x95\xbf\x33\xa9\xd1\xa3",
        }, {
                .key	= "\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b"
                        "\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b"
                        "\x0b\x0b\x0b\x0b\x0b\x0b",
                        .ksize	= 32,
                        .plaintext = "Hi There",
                        .psize	= 8,
                        .digest	= "\x19\x8a\x60\x7e\xb4\x4b\xfb\xc6"
                                "\x99\x03\xa0\xf1\xcf\x2b\xbd\xc5"
                                "\xba\x0a\xa3\xf3\xd9\xae\x3c\x1c"
                                "\x7a\x3b\x16\x96\xa0\xb6\x8c\xf7",
        }, {
                .key	= "Jefe",
                .ksize	= 4,
                .plaintext = "what do ya want for nothing?",
                .psize	= 28,
                .digest	= "\x5b\xdc\xc1\x46\xbf\x60\x75\x4e"
                        "\x6a\x04\x24\x26\x08\x95\x75\xc7"
                        "\x5a\x00\x3f\x08\x9d\x27\x39\x83"
                        "\x9d\xec\x58\xb9\x64\xec\x38\x43",
                        .np	= 2,
                        .tap	= { 14, 14 }
        }, {
                .key	= "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa",
                        .ksize	= 32,
                        .plaintext = "\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd"
                                "\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd"
                                "\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd"
                                "\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd\xdd",
                                .psize	= 50,
                                .digest	= "\xcd\xcb\x12\x20\xd1\xec\xcc\xea"
                                        "\x91\xe5\x3a\xba\x30\x92\xf9\x62"
                                        "\xe5\x49\xfe\x6c\xe9\xed\x7f\xdc"
                                        "\x43\x19\x1f\xbd\xe4\x5c\x30\xb0",
        }, {
                .key	= "\x01\x02\x03\x04\x05\x06\x07\x08"
                        "\x09\x0a\x0b\x0c\x0d\x0e\x0f\x10"
                        "\x11\x12\x13\x14\x15\x16\x17\x18"
                        "\x19\x1a\x1b\x1c\x1d\x1e\x1f\x20"
                        "\x21\x22\x23\x24\x25",
                        .ksize	= 37,
                        .plaintext = "\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd"
                                "\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd"
                                "\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd"
                                "\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd\xcd",
                                .psize	= 50,
                                .digest	= "\xd4\x63\x3c\x17\xf6\xfb\x8d\x74"
                                        "\x4c\x66\xde\xe0\xf8\xf0\x74\x55"
                                        "\x6e\xc4\xaf\x55\xef\x07\x99\x85"
                                        "\x41\x46\x8e\xb4\x9b\xd2\xe9\x17",
        }, {
                .key	= "\x0c\x0c\x0c\x0c\x0c\x0c\x0c\x0c\x0c\x0c\x0c\x0c\x0c"
                        "\x0c\x0c\x0c\x0c\x0c\x0c\x0c\x0c\x0c\x0c\x0c\x0c\x0c"
                        "\x0c\x0c\x0c\x0c\x0c\x0c",
                        .ksize	= 32,
                        .plaintext = "Test With Truncation",
                        .psize	= 20,
                        .digest	= "\x75\x46\xaf\x01\x84\x1f\xc0\x9b"
                                "\x1a\xb9\xc3\x74\x9a\x5f\x1c\x17"
                                "\xd4\xf5\x89\x66\x8a\x58\x7b\x27"
                                "\x00\xa9\xc9\x7c\x11\x93\xcf\x42",
        }, {
                .key	= "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa",
                        .ksize	= 80,
                        .plaintext = "Test Using Larger Than Block-Size Key - Hash Key First",
                        .psize	= 54,
                        .digest	= "\x69\x53\x02\x5e\xd9\x6f\x0c\x09"
                                "\xf8\x0a\x96\xf7\x8e\x65\x38\xdb"
                                "\xe2\xe7\xb8\x20\xe3\xdd\x97\x0e"
                                "\x7d\xdd\x39\x09\x1b\x32\x35\x2f",
        }, {
                .key	= "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa",
                        .ksize	= 80,
                        .plaintext = "Test Using Larger Than Block-Size Key and Larger Than "
                                "One Block-Size Data",
                                .psize	= 73,
                                .digest	= "\x63\x55\xac\x22\xe8\x90\xd0\xa3"
                                        "\xc8\x48\x1a\x5c\xa4\x82\x5b\xc8"
                                        "\x84\xd3\xe7\xa1\xff\x98\xa2\xfc"
                                        "\x2a\xc7\xd8\xe0\x64\xc3\xb2\xe6",
        },
};

/*
 * SHA384 HMAC test vectors from RFC4231
 */

#define HMAC_SHA384_TEST_VECTORS	4

static struct moto_hash_testvec moto_hmac_sha384_tv_template[] = {
        {
                .key	= "\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b"
                        "\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b"
                        "\x0b\x0b\x0b\x0b",
                        .ksize	= 20,
                        .plaintext = "Hi There",
                        .psize	= 8,
                        .digest	= "\xaf\xd0\x39\x44\xd8\x48\x95\x62"
                                "\x6b\x08\x25\xf4\xab\x46\x90\x7f"
                                "\x15\xf9\xda\xdb\xe4\x10\x1e\xc6"
                                "\x82\xaa\x03\x4c\x7c\xeb\xc5\x9c"
                                "\xfa\xea\x9e\xa9\x07\x6e\xde\x7f"
                                "\x4a\xf1\x52\xe8\xb2\xfa\x9c\xb6",
        }, {
                .key	= "Jefe",
                .ksize	= 4,
                .plaintext = "what do ya want for nothing?",
                .psize	= 28,
                .digest	= "\xaf\x45\xd2\xe3\x76\x48\x40\x31"
                        "\x61\x7f\x78\xd2\xb5\x8a\x6b\x1b"
                        "\x9c\x7e\xf4\x64\xf5\xa0\x1b\x47"
                        "\xe4\x2e\xc3\x73\x63\x22\x44\x5e"
                        "\x8e\x22\x40\xca\x5e\x69\xe2\xc7"
                        "\x8b\x32\x39\xec\xfa\xb2\x16\x49",
                        .np	= 4,
                        .tap	= { 7, 7, 7, 7 }
        }, {
                .key	= "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa",
                        .ksize	= 131,
                        .plaintext = "Test Using Larger Than Block-Siz"
                                "e Key - Hash Key First",
                                .psize	= 54,
                                .digest	= "\x4e\xce\x08\x44\x85\x81\x3e\x90"
                                        "\x88\xd2\xc6\x3a\x04\x1b\xc5\xb4"
                                        "\x4f\x9e\xf1\x01\x2a\x2b\x58\x8f"
                                        "\x3c\xd1\x1f\x05\x03\x3a\xc4\xc6"
                                        "\x0c\x2e\xf6\xab\x40\x30\xfe\x82"
                                        "\x96\x24\x8d\xf1\x63\xf4\x49\x52",
        }, {
                .key	= "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa",
                        .ksize	= 131,
                        .plaintext = "This is a test u"
                                "sing a larger th"
                                "an block-size ke"
                                "y and a larger t"
                                "han block-size d"
                                "ata. The key nee"
                                "ds to be hashed "
                                "before being use"
                                "d by the HMAC al"
                                "gorithm.",
                                .psize	= 152,
                                .digest	= "\x66\x17\x17\x8e\x94\x1f\x02\x0d"
                                        "\x35\x1e\x2f\x25\x4e\x8f\xd3\x2c"
                                        "\x60\x24\x20\xfe\xb0\xb8\xfb\x9a"
                                        "\xdc\xce\xbb\x82\x46\x1e\x99\xc5"
                                        "\xa6\x78\xcc\x31\xe7\x99\x17\x6d"
                                        "\x38\x60\xe6\x11\x0c\x46\x52\x3e",
        },
};

/*
 * SHA512 HMAC test vectors from RFC4231
 */

#define HMAC_SHA512_TEST_VECTORS	4

static struct moto_hash_testvec moto_hmac_sha512_tv_template[] = {
        {
                .key	= "\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b"
                        "\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b"
                        "\x0b\x0b\x0b\x0b",
                        .ksize	= 20,
                        .plaintext = "Hi There",
                        .psize	= 8,
                        .digest	= "\x87\xaa\x7c\xde\xa5\xef\x61\x9d"
                                "\x4f\xf0\xb4\x24\x1a\x1d\x6c\xb0"
                                "\x23\x79\xf4\xe2\xce\x4e\xc2\x78"
                                "\x7a\xd0\xb3\x05\x45\xe1\x7c\xde"
                                "\xda\xa8\x33\xb7\xd6\xb8\xa7\x02"
                                "\x03\x8b\x27\x4e\xae\xa3\xf4\xe4"
                                "\xbe\x9d\x91\x4e\xeb\x61\xf1\x70"
                                "\x2e\x69\x6c\x20\x3a\x12\x68\x54",
        }, {
                .key	= "Jefe",
                .ksize	= 4,
                .plaintext = "what do ya want for nothing?",
                .psize	= 28,
                .digest	= "\x16\x4b\x7a\x7b\xfc\xf8\x19\xe2"
                        "\xe3\x95\xfb\xe7\x3b\x56\xe0\xa3"
                        "\x87\xbd\x64\x22\x2e\x83\x1f\xd6"
                        "\x10\x27\x0c\xd7\xea\x25\x05\x54"
                        "\x97\x58\xbf\x75\xc0\x5a\x99\x4a"
                        "\x6d\x03\x4f\x65\xf8\xf0\xe6\xfd"
                        "\xca\xea\xb1\xa3\x4d\x4a\x6b\x4b"
                        "\x63\x6e\x07\x0a\x38\xbc\xe7\x37",
                        .np	= 4,
                        .tap	= { 7, 7, 7, 7 }
        }, {
                .key	= "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa",
                        .ksize	= 131,
                        .plaintext = "Test Using Large"
                                "r Than Block-Siz"
                                "e Key - Hash Key"
                                " First",
                                .psize	= 54,
                                .digest	= "\x80\xb2\x42\x63\xc7\xc1\xa3\xeb"
                                        "\xb7\x14\x93\xc1\xdd\x7b\xe8\xb4"
                                        "\x9b\x46\xd1\xf4\x1b\x4a\xee\xc1"
                                        "\x12\x1b\x01\x37\x83\xf8\xf3\x52"
                                        "\x6b\x56\xd0\x37\xe0\x5f\x25\x98"
                                        "\xbd\x0f\xd2\x21\x5d\x6a\x1e\x52"
                                        "\x95\xe6\x4f\x73\xf6\x3f\x0a\xec"
                                        "\x8b\x91\x5a\x98\x5d\x78\x65\x98",
        }, {
                .key	= "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa"
                        "\xaa\xaa\xaa",
                        .ksize	= 131,
                        .plaintext =
                                "This is a test u"
                                "sing a larger th"
                                "an block-size ke"
                                "y and a larger t"
                                "han block-size d"
                                "ata. The key nee"
                                "ds to be hashed "
                                "before being use"
                                "d by the HMAC al"
                                "gorithm.",
                                .psize	= 152,
                                .digest	= "\xe3\x7b\x6a\x77\x5d\xc8\x7d\xba"
                                        "\xa4\xdf\xa9\xf9\x6e\x5e\x3f\xfd"
                                        "\xde\xbd\x71\xf8\x86\x72\x89\x86"
                                        "\x5d\xf5\xa3\x2d\x20\xcd\xc9\x44"
                                        "\xb6\x02\x2c\xac\x3c\x49\x82\xb1"
                                        "\x0d\x5e\xeb\x55\xc3\xe4\xde\x15"
                                        "\x13\x46\x76\xfb\x6d\xe0\x44\x60"
                                        "\x65\xc9\x74\x40\xfa\x8c\x6a\x58",
        },
};

/*
 * DES test vectors.
 */
#define DES3_EDE_ENC_TEST_VECTORS	3
#define DES3_EDE_DEC_TEST_VECTORS	3
#define DES3_EDE_CBC_ENC_TEST_VECTORS	1
#define DES3_EDE_CBC_DEC_TEST_VECTORS	1


static struct moto_cipher_testvec moto_des3_ede_enc_tv_template[] = {
        { /* These are from openssl */
                .key	= "\x01\x23\x45\x67\x89\xab\xcd\xef"
                        "\x55\x55\x55\x55\x55\x55\x55\x55"
                        "\xfe\xdc\xba\x98\x76\x54\x32\x10",
                        .klen	= 24,
                        .input	= "\x73\x6f\x6d\x65\x64\x61\x74\x61",
                        .ilen	= 8,
                        .result = "\x18\xd7\x48\xe5\x63\x62\x05\x72",
                        .rlen	= 8,
        }, {
                .key	= "\x03\x52\x02\x07\x67\x20\x82\x17"
                        "\x86\x02\x87\x66\x59\x08\x21\x98"
                        "\x64\x05\x6a\xbd\xfe\xa9\x34\x57",
                        .klen	= 24,
                        .input	= "\x73\x71\x75\x69\x67\x67\x6c\x65",
                        .ilen	= 8,
                        .result	= "\xc0\x7d\x2a\x0f\xa5\x66\xfa\x30",
                        .rlen	= 8,
        }, {
                .key	= "\x10\x46\x10\x34\x89\x98\x80\x20"
                        "\x91\x07\xd0\x15\x89\x19\x01\x01"
                        "\x19\x07\x92\x10\x98\x1a\x01\x01",
                        .klen	= 24,
                        .input	= "\x00\x00\x00\x00\x00\x00\x00\x00",
                        .ilen	= 8,
                        .result	= "\xe1\xef\x62\xc3\x32\xfe\x82\x5b",
                        .rlen	= 8,
        },
};

static struct moto_cipher_testvec moto_des3_ede_dec_tv_template[] = {
        { /* These are from openssl */
                .key	= "\x01\x23\x45\x67\x89\xab\xcd\xef"
                        "\x55\x55\x55\x55\x55\x55\x55\x55"
                        "\xfe\xdc\xba\x98\x76\x54\x32\x10",
                        .klen	= 24,
                        .input	= "\x18\xd7\x48\xe5\x63\x62\x05\x72",
                        .ilen	= 8,
                        .result	= "\x73\x6f\x6d\x65\x64\x61\x74\x61",
                        .rlen	= 8,
        }, {
                .key	= "\x03\x52\x02\x07\x67\x20\x82\x17"
                        "\x86\x02\x87\x66\x59\x08\x21\x98"
                        "\x64\x05\x6a\xbd\xfe\xa9\x34\x57",
                        .klen	= 24,
                        .input	= "\xc0\x7d\x2a\x0f\xa5\x66\xfa\x30",
                        .ilen	= 8,
                        .result	= "\x73\x71\x75\x69\x67\x67\x6c\x65",
                        .rlen	= 8,
        }, {
                .key	= "\x10\x46\x10\x34\x89\x98\x80\x20"
                        "\x91\x07\xd0\x15\x89\x19\x01\x01"
                        "\x19\x07\x92\x10\x98\x1a\x01\x01",
                        .klen	= 24,
                        .input	= "\xe1\xef\x62\xc3\x32\xfe\x82\x5b",
                        .ilen	= 8,
                        .result	= "\x00\x00\x00\x00\x00\x00\x00\x00",
                        .rlen	= 8,
        },
};

static struct moto_cipher_testvec moto_des3_ede_cbc_enc_tv_template[] = {
        { /* Generated from openssl */
                .key	= "\xE9\xC0\xFF\x2E\x76\x0B\x64\x24"
                        "\x44\x4D\x99\x5A\x12\xD6\x40\xC0"
                        "\xEA\xC2\x84\xE8\x14\x95\xDB\xE8",
                        .klen	= 24,
                        .iv	= "\x7D\x33\x88\x93\x0F\x93\xB2\x42",
                        .input	= "\x6f\x54\x20\x6f\x61\x4d\x79\x6e"
                                "\x53\x20\x63\x65\x65\x72\x73\x74"
                                "\x54\x20\x6f\x6f\x4d\x20\x6e\x61"
                                "\x20\x79\x65\x53\x72\x63\x74\x65"
                                "\x20\x73\x6f\x54\x20\x6f\x61\x4d"
                                "\x79\x6e\x53\x20\x63\x65\x65\x72"
                                "\x73\x74\x54\x20\x6f\x6f\x4d\x20"
                                "\x6e\x61\x20\x79\x65\x53\x72\x63"
                                "\x74\x65\x20\x73\x6f\x54\x20\x6f"
                                "\x61\x4d\x79\x6e\x53\x20\x63\x65"
                                "\x65\x72\x73\x74\x54\x20\x6f\x6f"
                                "\x4d\x20\x6e\x61\x20\x79\x65\x53"
                                "\x72\x63\x74\x65\x20\x73\x6f\x54"
                                "\x20\x6f\x61\x4d\x79\x6e\x53\x20"
                                "\x63\x65\x65\x72\x73\x74\x54\x20"
                                "\x6f\x6f\x4d\x20\x6e\x61\x0a\x79",
                                .ilen	= 128,
                                .result	= "\x0e\x2d\xb6\x97\x3c\x56\x33\xf4"
                                        "\x67\x17\x21\xc7\x6e\x8a\xd5\x49"
                                        "\x74\xb3\x49\x05\xc5\x1c\xd0\xed"
                                        "\x12\x56\x5c\x53\x96\xb6\x00\x7d"
                                        "\x90\x48\xfc\xf5\x8d\x29\x39\xcc"
                                        "\x8a\xd5\x35\x18\x36\x23\x4e\xd7"
                                        "\x76\xd1\xda\x0c\x94\x67\xbb\x04"
                                        "\x8b\xf2\x03\x6c\xa8\xcf\xb6\xea"
                                        "\x22\x64\x47\xaa\x8f\x75\x13\xbf"
                                        "\x9f\xc2\xc3\xf0\xc9\x56\xc5\x7a"
                                        "\x71\x63\x2e\x89\x7b\x1e\x12\xca"
                                        "\xe2\x5f\xaf\xd8\xa4\xf8\xc9\x7a"
                                        "\xd6\xf9\x21\x31\x62\x44\x45\xa6"
                                        "\xd6\xbc\x5a\xd3\x2d\x54\x43\xcc"
                                        "\x9d\xde\xa5\x70\xe9\x42\x45\x8a"
                                        "\x6b\xfa\xb1\x91\x13\xb0\xd9\x19",
                                        .rlen	= 128,
        },
};

static struct moto_cipher_testvec moto_des3_ede_cbc_dec_tv_template[] = {
        { /* Generated from openssl */
                .key	= "\xE9\xC0\xFF\x2E\x76\x0B\x64\x24"
                        "\x44\x4D\x99\x5A\x12\xD6\x40\xC0"
                        "\xEA\xC2\x84\xE8\x14\x95\xDB\xE8",
                        .klen	= 24,
                        .iv	= "\x7D\x33\x88\x93\x0F\x93\xB2\x42",
                        .input	= "\x0e\x2d\xb6\x97\x3c\x56\x33\xf4"
                                "\x67\x17\x21\xc7\x6e\x8a\xd5\x49"
                                "\x74\xb3\x49\x05\xc5\x1c\xd0\xed"
                                "\x12\x56\x5c\x53\x96\xb6\x00\x7d"
                                "\x90\x48\xfc\xf5\x8d\x29\x39\xcc"
                                "\x8a\xd5\x35\x18\x36\x23\x4e\xd7"
                                "\x76\xd1\xda\x0c\x94\x67\xbb\x04"
                                "\x8b\xf2\x03\x6c\xa8\xcf\xb6\xea"
                                "\x22\x64\x47\xaa\x8f\x75\x13\xbf"
                                "\x9f\xc2\xc3\xf0\xc9\x56\xc5\x7a"
                                "\x71\x63\x2e\x89\x7b\x1e\x12\xca"
                                "\xe2\x5f\xaf\xd8\xa4\xf8\xc9\x7a"
                                "\xd6\xf9\x21\x31\x62\x44\x45\xa6"
                                "\xd6\xbc\x5a\xd3\x2d\x54\x43\xcc"
                                "\x9d\xde\xa5\x70\xe9\x42\x45\x8a"
                                "\x6b\xfa\xb1\x91\x13\xb0\xd9\x19",
                                .ilen	= 128,
                                .result	= "\x6f\x54\x20\x6f\x61\x4d\x79\x6e"
                                        "\x53\x20\x63\x65\x65\x72\x73\x74"
                                        "\x54\x20\x6f\x6f\x4d\x20\x6e\x61"
                                        "\x20\x79\x65\x53\x72\x63\x74\x65"
                                        "\x20\x73\x6f\x54\x20\x6f\x61\x4d"
                                        "\x79\x6e\x53\x20\x63\x65\x65\x72"
                                        "\x73\x74\x54\x20\x6f\x6f\x4d\x20"
                                        "\x6e\x61\x20\x79\x65\x53\x72\x63"
                                        "\x74\x65\x20\x73\x6f\x54\x20\x6f"
                                        "\x61\x4d\x79\x6e\x53\x20\x63\x65"
                                        "\x65\x72\x73\x74\x54\x20\x6f\x6f"
                                        "\x4d\x20\x6e\x61\x20\x79\x65\x53"
                                        "\x72\x63\x74\x65\x20\x73\x6f\x54"
                                        "\x20\x6f\x61\x4d\x79\x6e\x53\x20"
                                        "\x63\x65\x65\x72\x73\x74\x54\x20"
                                        "\x6f\x6f\x4d\x20\x6e\x61\x0a\x79",
                                        .rlen	= 128,
        },
};

/*
 * AES test vectors.
 */
#define AES_ENC_TEST_VECTORS 3
#define AES_DEC_TEST_VECTORS 3
#define AES_CBC_ENC_TEST_VECTORS 4
#define AES_CBC_DEC_TEST_VECTORS 4
#define AES_CTR_ENC_TEST_VECTORS 3
#define AES_CTR_DEC_TEST_VECTORS 3

static struct moto_cipher_testvec moto_aes_enc_tv_template[] = {
        { /* From FIPS-197 */
                .key	= "\x00\x01\x02\x03\x04\x05\x06\x07"
                        "\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f",
                        .klen	= 16,
                        .input	= "\x00\x11\x22\x33\x44\x55\x66\x77"
                                "\x88\x99\xaa\xbb\xcc\xdd\xee\xff",
                                .ilen	= 16,
                                .result	= "\x69\xc4\xe0\xd8\x6a\x7b\x04\x30"
                                        "\xd8\xcd\xb7\x80\x70\xb4\xc5\x5a",
                                        .rlen	= 16,
        }, {
                .key	= "\x00\x01\x02\x03\x04\x05\x06\x07"
                        "\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f"
                        "\x10\x11\x12\x13\x14\x15\x16\x17",
                        .klen	= 24,
                        .input	= "\x00\x11\x22\x33\x44\x55\x66\x77"
                                "\x88\x99\xaa\xbb\xcc\xdd\xee\xff",
                                .ilen	= 16,
                                .result	= "\xdd\xa9\x7c\xa4\x86\x4c\xdf\xe0"
                                        "\x6e\xaf\x70\xa0\xec\x0d\x71\x91",
                                        .rlen	= 16,
        }, {
                .key	= "\x00\x01\x02\x03\x04\x05\x06\x07"
                        "\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f"
                        "\x10\x11\x12\x13\x14\x15\x16\x17"
                        "\x18\x19\x1a\x1b\x1c\x1d\x1e\x1f",
                        .klen	= 32,
                        .input	= "\x00\x11\x22\x33\x44\x55\x66\x77"
                                "\x88\x99\xaa\xbb\xcc\xdd\xee\xff",
                                .ilen	= 16,
                                .result	= "\x8e\xa2\xb7\xca\x51\x67\x45\xbf"
                                        "\xea\xfc\x49\x90\x4b\x49\x60\x89",
                                        .rlen	= 16,
        },
};

static struct moto_cipher_testvec moto_aes_dec_tv_template[] = {
        { /* From FIPS-197 */
                .key	= "\x00\x01\x02\x03\x04\x05\x06\x07"
                        "\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f",
                        .klen	= 16,
                        .input	= "\x69\xc4\xe0\xd8\x6a\x7b\x04\x30"
                                "\xd8\xcd\xb7\x80\x70\xb4\xc5\x5a",
                                .ilen	= 16,
                                .result	= "\x00\x11\x22\x33\x44\x55\x66\x77"
                                        "\x88\x99\xaa\xbb\xcc\xdd\xee\xff",
                                        .rlen	= 16,
        }, {
                .key	= "\x00\x01\x02\x03\x04\x05\x06\x07"
                        "\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f"
                        "\x10\x11\x12\x13\x14\x15\x16\x17",
                        .klen	= 24,
                        .input	= "\xdd\xa9\x7c\xa4\x86\x4c\xdf\xe0"
                                "\x6e\xaf\x70\xa0\xec\x0d\x71\x91",
                                .ilen	= 16,
                                .result	= "\x00\x11\x22\x33\x44\x55\x66\x77"
                                        "\x88\x99\xaa\xbb\xcc\xdd\xee\xff",
                                        .rlen	= 16,
        }, {
                .key	= "\x00\x01\x02\x03\x04\x05\x06\x07"
                        "\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f"
                        "\x10\x11\x12\x13\x14\x15\x16\x17"
                        "\x18\x19\x1a\x1b\x1c\x1d\x1e\x1f",
                        .klen	= 32,
                        .input	= "\x8e\xa2\xb7\xca\x51\x67\x45\xbf"
                                "\xea\xfc\x49\x90\x4b\x49\x60\x89",
                                .ilen	= 16,
                                .result	= "\x00\x11\x22\x33\x44\x55\x66\x77"
                                        "\x88\x99\xaa\xbb\xcc\xdd\xee\xff",
                                        .rlen	= 16,
        },
};

static struct moto_cipher_testvec moto_aes_cbc_enc_tv_template[] = {
        { /* From RFC 3602 */
                .key    = "\x06\xa9\x21\x40\x36\xb8\xa1\x5b"
                        "\x51\x2e\x03\xd5\x34\x12\x00\x06",
                        .klen   = 16,
                        .iv	= "\x3d\xaf\xba\x42\x9d\x9e\xb4\x30"
                                "\xb4\x22\xda\x80\x2c\x9f\xac\x41",
                                .input	= "Single block msg",
                                .ilen   = 16,
                                .result = "\xe3\x53\x77\x9c\x10\x79\xae\xb8"
                                        "\x27\x08\x94\x2d\xbe\x77\x18\x1a",
                                        .rlen   = 16,
        }, {
                .key    = "\xc2\x86\x69\x6d\x88\x7c\x9a\xa0"
                        "\x61\x1b\xbb\x3e\x20\x25\xa4\x5a",
                        .klen   = 16,
                        .iv     = "\x56\x2e\x17\x99\x6d\x09\x3d\x28"
                                "\xdd\xb3\xba\x69\x5a\x2e\x6f\x58",
                                .input  = "\x00\x01\x02\x03\x04\x05\x06\x07"
                                        "\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f"
                                        "\x10\x11\x12\x13\x14\x15\x16\x17"
                                        "\x18\x19\x1a\x1b\x1c\x1d\x1e\x1f",
                                        .ilen   = 32,
                                        .result = "\xd2\x96\xcd\x94\xc2\xcc\xcf\x8a"
                                                "\x3a\x86\x30\x28\xb5\xe1\xdc\x0a"
                                                "\x75\x86\x60\x2d\x25\x3c\xff\xf9"
                                                "\x1b\x82\x66\xbe\xa6\xd6\x1a\xb1",
                                                .rlen   = 32,
        }, { /* From NIST SP800-38A */
                .key	= "\x8e\x73\xb0\xf7\xda\x0e\x64\x52"
                        "\xc8\x10\xf3\x2b\x80\x90\x79\xe5"
                        "\x62\xf8\xea\xd2\x52\x2c\x6b\x7b",
                        .klen	= 24,
                        .iv	= "\x00\x01\x02\x03\x04\x05\x06\x07"
                                "\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f",
                                .input	= "\x6b\xc1\xbe\xe2\x2e\x40\x9f\x96"
                                        "\xe9\x3d\x7e\x11\x73\x93\x17\x2a"
                                        "\xae\x2d\x8a\x57\x1e\x03\xac\x9c"
                                        "\x9e\xb7\x6f\xac\x45\xaf\x8e\x51"
                                        "\x30\xc8\x1c\x46\xa3\x5c\xe4\x11"
                                        "\xe5\xfb\xc1\x19\x1a\x0a\x52\xef"
                                        "\xf6\x9f\x24\x45\xdf\x4f\x9b\x17"
                                        "\xad\x2b\x41\x7b\xe6\x6c\x37\x10",
                                        .ilen	= 64,
                                        .result	= "\x4f\x02\x1d\xb2\x43\xbc\x63\x3d"
                                                "\x71\x78\x18\x3a\x9f\xa0\x71\xe8"
                                                "\xb4\xd9\xad\xa9\xad\x7d\xed\xf4"
                                                "\xe5\xe7\x38\x76\x3f\x69\x14\x5a"
                                                "\x57\x1b\x24\x20\x12\xfb\x7a\xe0"
                                                "\x7f\xa9\xba\xac\x3d\xf1\x02\xe0"
                                                "\x08\xb0\xe2\x79\x88\x59\x88\x81"
                                                "\xd9\x20\xa9\xe6\x4f\x56\x15\xcd",
                                                .rlen	= 64,
        }, {
                .key	= "\x60\x3d\xeb\x10\x15\xca\x71\xbe"
                        "\x2b\x73\xae\xf0\x85\x7d\x77\x81"
                        "\x1f\x35\x2c\x07\x3b\x61\x08\xd7"
                        "\x2d\x98\x10\xa3\x09\x14\xdf\xf4",
                        .klen	= 32,
                        .iv	= "\x00\x01\x02\x03\x04\x05\x06\x07"
                                "\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f",
                                .input	= "\x6b\xc1\xbe\xe2\x2e\x40\x9f\x96"
                                        "\xe9\x3d\x7e\x11\x73\x93\x17\x2a"
                                        "\xae\x2d\x8a\x57\x1e\x03\xac\x9c"
                                        "\x9e\xb7\x6f\xac\x45\xaf\x8e\x51"
                                        "\x30\xc8\x1c\x46\xa3\x5c\xe4\x11"
                                        "\xe5\xfb\xc1\x19\x1a\x0a\x52\xef"
                                        "\xf6\x9f\x24\x45\xdf\x4f\x9b\x17"
                                        "\xad\x2b\x41\x7b\xe6\x6c\x37\x10",
                                        .ilen	= 64,
                                        .result	= "\xf5\x8c\x4c\x04\xd6\xe5\xf1\xba"
                                                "\x77\x9e\xab\xfb\x5f\x7b\xfb\xd6"
                                                "\x9c\xfc\x4e\x96\x7e\xdb\x80\x8d"
                                                "\x67\x9f\x77\x7b\xc6\x70\x2c\x7d"
                                                "\x39\xf2\x33\x69\xa9\xd9\xba\xcf"
                                                "\xa5\x30\xe2\x63\x04\x23\x14\x61"
                                                "\xb2\xeb\x05\xe2\xc3\x9b\xe9\xfc"
                                                "\xda\x6c\x19\x07\x8c\x6a\x9d\x1b",
                                                .rlen	= 64,
        },
};

static struct moto_cipher_testvec moto_aes_cbc_dec_tv_template[] = {
        { /* From RFC 3602 */
                .key    = "\x06\xa9\x21\x40\x36\xb8\xa1\x5b"
                        "\x51\x2e\x03\xd5\x34\x12\x00\x06",
                        .klen   = 16,
                        .iv     = "\x3d\xaf\xba\x42\x9d\x9e\xb4\x30"
                                "\xb4\x22\xda\x80\x2c\x9f\xac\x41",
                                .input  = "\xe3\x53\x77\x9c\x10\x79\xae\xb8"
                                        "\x27\x08\x94\x2d\xbe\x77\x18\x1a",
                                        .ilen   = 16,
                                        .result = "Single block msg",
                                        .rlen   = 16,
        }, {
                .key    = "\xc2\x86\x69\x6d\x88\x7c\x9a\xa0"
                        "\x61\x1b\xbb\x3e\x20\x25\xa4\x5a",
                        .klen   = 16,
                        .iv     = "\x56\x2e\x17\x99\x6d\x09\x3d\x28"
                                "\xdd\xb3\xba\x69\x5a\x2e\x6f\x58",
                                .input  = "\xd2\x96\xcd\x94\xc2\xcc\xcf\x8a"
                                        "\x3a\x86\x30\x28\xb5\xe1\xdc\x0a"
                                        "\x75\x86\x60\x2d\x25\x3c\xff\xf9"
                                        "\x1b\x82\x66\xbe\xa6\xd6\x1a\xb1",
                                        .ilen   = 32,
                                        .result = "\x00\x01\x02\x03\x04\x05\x06\x07"
                                                "\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f"
                                                "\x10\x11\x12\x13\x14\x15\x16\x17"
                                                "\x18\x19\x1a\x1b\x1c\x1d\x1e\x1f",
                                                .rlen   = 32,
        }, { /* From NIST SP800-38A */
                .key	= "\x8e\x73\xb0\xf7\xda\x0e\x64\x52"
                        "\xc8\x10\xf3\x2b\x80\x90\x79\xe5"
                        "\x62\xf8\xea\xd2\x52\x2c\x6b\x7b",
                        .klen	= 24,
                        .iv	= "\x00\x01\x02\x03\x04\x05\x06\x07"
                                "\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f",
                                .input	= "\x4f\x02\x1d\xb2\x43\xbc\x63\x3d"
                                        "\x71\x78\x18\x3a\x9f\xa0\x71\xe8"
                                        "\xb4\xd9\xad\xa9\xad\x7d\xed\xf4"
                                        "\xe5\xe7\x38\x76\x3f\x69\x14\x5a"
                                        "\x57\x1b\x24\x20\x12\xfb\x7a\xe0"
                                        "\x7f\xa9\xba\xac\x3d\xf1\x02\xe0"
                                        "\x08\xb0\xe2\x79\x88\x59\x88\x81"
                                        "\xd9\x20\xa9\xe6\x4f\x56\x15\xcd",
                                        .ilen	= 64,
                                        .result	= "\x6b\xc1\xbe\xe2\x2e\x40\x9f\x96"
                                                "\xe9\x3d\x7e\x11\x73\x93\x17\x2a"
                                                "\xae\x2d\x8a\x57\x1e\x03\xac\x9c"
                                                "\x9e\xb7\x6f\xac\x45\xaf\x8e\x51"
                                                "\x30\xc8\x1c\x46\xa3\x5c\xe4\x11"
                                                "\xe5\xfb\xc1\x19\x1a\x0a\x52\xef"
                                                "\xf6\x9f\x24\x45\xdf\x4f\x9b\x17"
                                                "\xad\x2b\x41\x7b\xe6\x6c\x37\x10",
                                                .rlen	= 64,
        }, {
                .key	= "\x60\x3d\xeb\x10\x15\xca\x71\xbe"
                        "\x2b\x73\xae\xf0\x85\x7d\x77\x81"
                        "\x1f\x35\x2c\x07\x3b\x61\x08\xd7"
                        "\x2d\x98\x10\xa3\x09\x14\xdf\xf4",
                        .klen	= 32,
                        .iv	= "\x00\x01\x02\x03\x04\x05\x06\x07"
                                "\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f",
                                .input	= "\xf5\x8c\x4c\x04\xd6\xe5\xf1\xba"
                                        "\x77\x9e\xab\xfb\x5f\x7b\xfb\xd6"
                                        "\x9c\xfc\x4e\x96\x7e\xdb\x80\x8d"
                                        "\x67\x9f\x77\x7b\xc6\x70\x2c\x7d"
                                        "\x39\xf2\x33\x69\xa9\xd9\xba\xcf"
                                        "\xa5\x30\xe2\x63\x04\x23\x14\x61"
                                        "\xb2\xeb\x05\xe2\xc3\x9b\xe9\xfc"
                                        "\xda\x6c\x19\x07\x8c\x6a\x9d\x1b",
                                        .ilen	= 64,
                                        .result	= "\x6b\xc1\xbe\xe2\x2e\x40\x9f\x96"
                                                "\xe9\x3d\x7e\x11\x73\x93\x17\x2a"
                                                "\xae\x2d\x8a\x57\x1e\x03\xac\x9c"
                                                "\x9e\xb7\x6f\xac\x45\xaf\x8e\x51"
                                                "\x30\xc8\x1c\x46\xa3\x5c\xe4\x11"
                                                "\xe5\xfb\xc1\x19\x1a\x0a\x52\xef"
                                                "\xf6\x9f\x24\x45\xdf\x4f\x9b\x17"
                                                "\xad\x2b\x41\x7b\xe6\x6c\x37\x10",
                                                .rlen	= 64,
        },
};

static struct moto_cipher_testvec moto_aes_ctr_enc_tv_template[] = {
        { /* From NIST Special Publication 800-38A, Appendix F.5 */
                .key	= "\x2b\x7e\x15\x16\x28\xae\xd2\xa6"
                        "\xab\xf7\x15\x88\x09\xcf\x4f\x3c",
                        .klen	= 16,
                        .iv	= "\xf0\xf1\xf2\xf3\xf4\xf5\xf6\xf7"
                                "\xf8\xf9\xfa\xfb\xfc\xfd\xfe\xff",
                                .input	= "\x6b\xc1\xbe\xe2\x2e\x40\x9f\x96"
                                        "\xe9\x3d\x7e\x11\x73\x93\x17\x2a"
                                        "\xae\x2d\x8a\x57\x1e\x03\xac\x9c"
                                        "\x9e\xb7\x6f\xac\x45\xaf\x8e\x51"
                                        "\x30\xc8\x1c\x46\xa3\x5c\xe4\x11"
                                        "\xe5\xfb\xc1\x19\x1a\x0a\x52\xef"
                                        "\xf6\x9f\x24\x45\xdf\x4f\x9b\x17"
                                        "\xad\x2b\x41\x7b\xe6\x6c\x37\x10",
                                        .ilen	= 64,
                                        .result	= "\x87\x4d\x61\x91\xb6\x20\xe3\x26"
                                                "\x1b\xef\x68\x64\x99\x0d\xb6\xce"
                                                "\x98\x06\xf6\x6b\x79\x70\xfd\xff"
                                                "\x86\x17\x18\x7b\xb9\xff\xfd\xff"
                                                "\x5a\xe4\xdf\x3e\xdb\xd5\xd3\x5e"
                                                "\x5b\x4f\x09\x02\x0d\xb0\x3e\xab"
                                                "\x1e\x03\x1d\xda\x2f\xbe\x03\xd1"
                                                "\x79\x21\x70\xa0\xf3\x00\x9c\xee",
                                                .rlen	= 64,
        }, {
                .key	= "\x8e\x73\xb0\xf7\xda\x0e\x64\x52"
                        "\xc8\x10\xf3\x2b\x80\x90\x79\xe5"
                        "\x62\xf8\xea\xd2\x52\x2c\x6b\x7b",
                        .klen	= 24,
                        .iv	= "\xf0\xf1\xf2\xf3\xf4\xf5\xf6\xf7"
                                "\xf8\xf9\xfa\xfb\xfc\xfd\xfe\xff",
                                .input	= "\x6b\xc1\xbe\xe2\x2e\x40\x9f\x96"
                                        "\xe9\x3d\x7e\x11\x73\x93\x17\x2a"
                                        "\xae\x2d\x8a\x57\x1e\x03\xac\x9c"
                                        "\x9e\xb7\x6f\xac\x45\xaf\x8e\x51"
                                        "\x30\xc8\x1c\x46\xa3\x5c\xe4\x11"
                                        "\xe5\xfb\xc1\x19\x1a\x0a\x52\xef"
                                        "\xf6\x9f\x24\x45\xdf\x4f\x9b\x17"
                                        "\xad\x2b\x41\x7b\xe6\x6c\x37\x10",
                                        .ilen	= 64,
                                        .result	= "\x1a\xbc\x93\x24\x17\x52\x1c\xa2"
                                                "\x4f\x2b\x04\x59\xfe\x7e\x6e\x0b"
                                                "\x09\x03\x39\xec\x0a\xa6\xfa\xef"
                                                "\xd5\xcc\xc2\xc6\xf4\xce\x8e\x94"
                                                "\x1e\x36\xb2\x6b\xd1\xeb\xc6\x70"
                                                "\xd1\xbd\x1d\x66\x56\x20\xab\xf7"
                                                "\x4f\x78\xa7\xf6\xd2\x98\x09\x58"
                                                "\x5a\x97\xda\xec\x58\xc6\xb0\x50",
                                                .rlen	= 64,
        }, {
                .key	= "\x60\x3d\xeb\x10\x15\xca\x71\xbe"
                        "\x2b\x73\xae\xf0\x85\x7d\x77\x81"
                        "\x1f\x35\x2c\x07\x3b\x61\x08\xd7"
                        "\x2d\x98\x10\xa3\x09\x14\xdf\xf4",
                        .klen	= 32,
                        .iv	= "\xf0\xf1\xf2\xf3\xf4\xf5\xf6\xf7"
                                "\xf8\xf9\xfa\xfb\xfc\xfd\xfe\xff",
                                .input	= "\x6b\xc1\xbe\xe2\x2e\x40\x9f\x96"
                                        "\xe9\x3d\x7e\x11\x73\x93\x17\x2a"
                                        "\xae\x2d\x8a\x57\x1e\x03\xac\x9c"
                                        "\x9e\xb7\x6f\xac\x45\xaf\x8e\x51"
                                        "\x30\xc8\x1c\x46\xa3\x5c\xe4\x11"
                                        "\xe5\xfb\xc1\x19\x1a\x0a\x52\xef"
                                        "\xf6\x9f\x24\x45\xdf\x4f\x9b\x17"
                                        "\xad\x2b\x41\x7b\xe6\x6c\x37\x10",
                                        .ilen	= 64,
                                        .result	= "\x60\x1e\xc3\x13\x77\x57\x89\xa5"
                                                "\xb7\xa7\xf5\x04\xbb\xf3\xd2\x28"
                                                "\xf4\x43\xe3\xca\x4d\x62\xb5\x9a"
                                                "\xca\x84\xe9\x90\xca\xca\xf5\xc5"
                                                "\x2b\x09\x30\xda\xa2\x3d\xe9\x4c"
                                                "\xe8\x70\x17\xba\x2d\x84\x98\x8d"
                                                "\xdf\xc9\xc5\x8d\xb6\x7a\xad\xa6"
                                                "\x13\xc2\xdd\x08\x45\x79\x41\xa6",
                                                .rlen	= 64,
        }
};

static struct moto_cipher_testvec moto_aes_ctr_dec_tv_template[] = {
        { /* From NIST Special Publication 800-38A, Appendix F.5 */
                .key	= "\x2b\x7e\x15\x16\x28\xae\xd2\xa6"
                        "\xab\xf7\x15\x88\x09\xcf\x4f\x3c",
                        .klen	= 16,
                        .iv	= "\xf0\xf1\xf2\xf3\xf4\xf5\xf6\xf7"
                                "\xf8\xf9\xfa\xfb\xfc\xfd\xfe\xff",
                                .input	= "\x87\x4d\x61\x91\xb6\x20\xe3\x26"
                                        "\x1b\xef\x68\x64\x99\x0d\xb6\xce"
                                        "\x98\x06\xf6\x6b\x79\x70\xfd\xff"
                                        "\x86\x17\x18\x7b\xb9\xff\xfd\xff"
                                        "\x5a\xe4\xdf\x3e\xdb\xd5\xd3\x5e"
                                        "\x5b\x4f\x09\x02\x0d\xb0\x3e\xab"
                                        "\x1e\x03\x1d\xda\x2f\xbe\x03\xd1"
                                        "\x79\x21\x70\xa0\xf3\x00\x9c\xee",
                                        .ilen	= 64,
                                        .result	= "\x6b\xc1\xbe\xe2\x2e\x40\x9f\x96"
                                                "\xe9\x3d\x7e\x11\x73\x93\x17\x2a"
                                                "\xae\x2d\x8a\x57\x1e\x03\xac\x9c"
                                                "\x9e\xb7\x6f\xac\x45\xaf\x8e\x51"
                                                "\x30\xc8\x1c\x46\xa3\x5c\xe4\x11"
                                                "\xe5\xfb\xc1\x19\x1a\x0a\x52\xef"
                                                "\xf6\x9f\x24\x45\xdf\x4f\x9b\x17"
                                                "\xad\x2b\x41\x7b\xe6\x6c\x37\x10",
                                                .rlen	= 64,
        }, {
                .key	= "\x8e\x73\xb0\xf7\xda\x0e\x64\x52"
                        "\xc8\x10\xf3\x2b\x80\x90\x79\xe5"
                        "\x62\xf8\xea\xd2\x52\x2c\x6b\x7b",
                        .klen	= 24,
                        .iv	= "\xf0\xf1\xf2\xf3\xf4\xf5\xf6\xf7"
                                "\xf8\xf9\xfa\xfb\xfc\xfd\xfe\xff",
                                .input	= "\x1a\xbc\x93\x24\x17\x52\x1c\xa2"
                                        "\x4f\x2b\x04\x59\xfe\x7e\x6e\x0b"
                                        "\x09\x03\x39\xec\x0a\xa6\xfa\xef"
                                        "\xd5\xcc\xc2\xc6\xf4\xce\x8e\x94"
                                        "\x1e\x36\xb2\x6b\xd1\xeb\xc6\x70"
                                        "\xd1\xbd\x1d\x66\x56\x20\xab\xf7"
                                        "\x4f\x78\xa7\xf6\xd2\x98\x09\x58"
                                        "\x5a\x97\xda\xec\x58\xc6\xb0\x50",
                                        .ilen	= 64,
                                        .result	= "\x6b\xc1\xbe\xe2\x2e\x40\x9f\x96"
                                                "\xe9\x3d\x7e\x11\x73\x93\x17\x2a"
                                                "\xae\x2d\x8a\x57\x1e\x03\xac\x9c"
                                                "\x9e\xb7\x6f\xac\x45\xaf\x8e\x51"
                                                "\x30\xc8\x1c\x46\xa3\x5c\xe4\x11"
                                                "\xe5\xfb\xc1\x19\x1a\x0a\x52\xef"
                                                "\xf6\x9f\x24\x45\xdf\x4f\x9b\x17"
                                                "\xad\x2b\x41\x7b\xe6\x6c\x37\x10",
                                                .rlen	= 64,
        }, {
                .key	= "\x60\x3d\xeb\x10\x15\xca\x71\xbe"
                        "\x2b\x73\xae\xf0\x85\x7d\x77\x81"
                        "\x1f\x35\x2c\x07\x3b\x61\x08\xd7"
                        "\x2d\x98\x10\xa3\x09\x14\xdf\xf4",
                        .klen	= 32,
                        .iv	= "\xf0\xf1\xf2\xf3\xf4\xf5\xf6\xf7"
                                "\xf8\xf9\xfa\xfb\xfc\xfd\xfe\xff",
                                .input	= "\x60\x1e\xc3\x13\x77\x57\x89\xa5"
                                        "\xb7\xa7\xf5\x04\xbb\xf3\xd2\x28"
                                        "\xf4\x43\xe3\xca\x4d\x62\xb5\x9a"
                                        "\xca\x84\xe9\x90\xca\xca\xf5\xc5"
                                        "\x2b\x09\x30\xda\xa2\x3d\xe9\x4c"
                                        "\xe8\x70\x17\xba\x2d\x84\x98\x8d"
                                        "\xdf\xc9\xc5\x8d\xb6\x7a\xad\xa6"
                                        "\x13\xc2\xdd\x08\x45\x79\x41\xa6",
                                        .ilen	= 64,
                                        .result	= "\x6b\xc1\xbe\xe2\x2e\x40\x9f\x96"
                                                "\xe9\x3d\x7e\x11\x73\x93\x17\x2a"
                                                "\xae\x2d\x8a\x57\x1e\x03\xac\x9c"
                                                "\x9e\xb7\x6f\xac\x45\xaf\x8e\x51"
                                                "\x30\xc8\x1c\x46\xa3\x5c\xe4\x11"
                                                "\xe5\xfb\xc1\x19\x1a\x0a\x52\xef"
                                                "\xf6\x9f\x24\x45\xdf\x4f\x9b\x17"
                                                "\xad\x2b\x41\x7b\xe6\x6c\x37\x10",
                                                .rlen	= 64,
        }
};


/*
 * ANSI X9.31 Continuous Pseudo-Random Number Generator (AES mode)
 * test vectors, taken from Appendix B.2.9 and B.2.10:
 *     http://csrc.nist.gov/groups/STM/cavp/documents/rng/RNGVS.pdf
 * Only AES-128 is supported at this time.
 */
#define ANSI_CPRNG_AES_TEST_VECTORS	6

static struct moto_cprng_testvec moto_ansi_cprng_aes_tv_template[] = {
        {
                .key	= "\xf3\xb1\x66\x6d\x13\x60\x72\x42"
                        "\xed\x06\x1c\xab\xb8\xd4\x62\x02",
                        .klen	= 16,
                        .dt	= "\xe6\xb3\xbe\x78\x2a\x23\xfa\x62"
                                "\xd7\x1d\x4a\xfb\xb0\xe9\x22\xf9",
                                .dtlen	= 16,
                                .v	= "\x80\x00\x00\x00\x00\x00\x00\x00"
                                        "\x00\x00\x00\x00\x00\x00\x00\x00",
                                        .vlen	= 16,
                                        .result	= "\x59\x53\x1e\xd1\x3b\xb0\xc0\x55"
                                                "\x84\x79\x66\x85\xc1\x2f\x76\x41",
                                                .rlen	= 16,
                                                .loops	= 1,
        }, {
                .key	= "\xf3\xb1\x66\x6d\x13\x60\x72\x42"
                        "\xed\x06\x1c\xab\xb8\xd4\x62\x02",
                        .klen	= 16,
                        .dt	= "\xe6\xb3\xbe\x78\x2a\x23\xfa\x62"
                                "\xd7\x1d\x4a\xfb\xb0\xe9\x22\xfa",
                                .dtlen	= 16,
                                .v	= "\xc0\x00\x00\x00\x00\x00\x00\x00"
                                        "\x00\x00\x00\x00\x00\x00\x00\x00",
                                        .vlen	= 16,
                                        .result	= "\x7c\x22\x2c\xf4\xca\x8f\xa2\x4c"
                                                "\x1c\x9c\xb6\x41\xa9\xf3\x22\x0d",
                                                .rlen	= 16,
                                                .loops	= 1,
        }, {
                .key	= "\xf3\xb1\x66\x6d\x13\x60\x72\x42"
                        "\xed\x06\x1c\xab\xb8\xd4\x62\x02",
                        .klen	= 16,
                        .dt	= "\xe6\xb3\xbe\x78\x2a\x23\xfa\x62"
                                "\xd7\x1d\x4a\xfb\xb0\xe9\x22\xfb",
                                .dtlen	= 16,
                                .v	= "\xe0\x00\x00\x00\x00\x00\x00\x00"
                                        "\x00\x00\x00\x00\x00\x00\x00\x00",
                                        .vlen	= 16,
                                        .result	= "\x8a\xaa\x00\x39\x66\x67\x5b\xe5"
                                                "\x29\x14\x28\x81\xa9\x4d\x4e\xc7",
                                                .rlen	= 16,
                                                .loops	= 1,
        }, {
                .key	= "\xf3\xb1\x66\x6d\x13\x60\x72\x42"
                        "\xed\x06\x1c\xab\xb8\xd4\x62\x02",
                        .klen	= 16,
                        .dt	= "\xe6\xb3\xbe\x78\x2a\x23\xfa\x62"
                                "\xd7\x1d\x4a\xfb\xb0\xe9\x22\xfc",
                                .dtlen	= 16,
                                .v	= "\xf0\x00\x00\x00\x00\x00\x00\x00"
                                        "\x00\x00\x00\x00\x00\x00\x00\x00",
                                        .vlen	= 16,
                                        .result	= "\x88\xdd\xa4\x56\x30\x24\x23\xe5"
                                                "\xf6\x9d\xa5\x7e\x7b\x95\xc7\x3a",
                                                .rlen	= 16,
                                                .loops	= 1,
        }, {
                .key	= "\xf3\xb1\x66\x6d\x13\x60\x72\x42"
                        "\xed\x06\x1c\xab\xb8\xd4\x62\x02",
                        .klen	= 16,
                        .dt	= "\xe6\xb3\xbe\x78\x2a\x23\xfa\x62"
                                "\xd7\x1d\x4a\xfb\xb0\xe9\x22\xfd",
                                .dtlen	= 16,
                                .v	= "\xf8\x00\x00\x00\x00\x00\x00\x00"
                                        "\x00\x00\x00\x00\x00\x00\x00\x00",
                                        .vlen	= 16,
                                        .result	= "\x05\x25\x92\x46\x61\x79\xd2\xcb"
                                                "\x78\xc4\x0b\x14\x0a\x5a\x9a\xc8",
                                                .rlen	= 16,
                                                .loops	= 1,
        }, {	/* Monte Carlo Test */
                .key	= "\x9f\x5b\x51\x20\x0b\xf3\x34\xb5"
                        "\xd8\x2b\xe8\xc3\x72\x55\xc8\x48",
                        .klen	= 16,
                        .dt	= "\x63\x76\xbb\xe5\x29\x02\xba\x3b"
                                "\x67\xc9\x25\xfa\x70\x1f\x11\xac",
                                .dtlen	= 16,
                                .v	= "\x57\x2c\x8e\x76\x87\x26\x47\x97"
                                        "\x7e\x74\xfb\xdd\xc4\x95\x01\xd1",
                                        .vlen	= 16,
                                        .result	= "\x48\xe9\xbd\x0d\x06\xee\x18\xfb"
                                                "\xe4\x57\x90\xd5\xc3\xfc\x9b\x73",
                                                .rlen	= 16,
                                                .loops	= 10000,
        },
};

struct moto_tcrypt_result {
    struct completion completion;
    int err;
};

struct moto_cipher_test_suite {
    struct {
        struct moto_cipher_testvec *vecs;
        unsigned int count;
    } enc, dec;
};

struct moto_hash_test_suite {
    struct moto_hash_testvec *vecs;
    unsigned int count;
};

struct moto_cprng_test_suite {
    struct moto_cprng_testvec *vecs;
    unsigned int count;
};

struct moto_alg_test_desc {
    const char *alg;
    int (*test)(const struct moto_alg_test_desc *desc, const char *driver,
            u32 type, u32 mask);
    unsigned alg_id;
    union {
        struct moto_cipher_test_suite cipher;
        struct moto_hash_test_suite hash;
        struct moto_cprng_test_suite cprng;
    } suite;
};

static unsigned int 
MOTO_IDX[8] = { IDX1, IDX2, IDX3, IDX4, IDX5, IDX6, IDX7, IDX8 };

static void moto_tcrypt_complete(struct crypto_async_request *req, int err)
{
    struct moto_tcrypt_result *res = req->data;

    if (err == -EINPROGRESS)
        return;

    res->err = err;
    complete(&res->completion);
}

static int moto_testmgr_alloc_buf(char *buf[XBUFSIZE])
{
    int i;

    for (i = 0; i < XBUFSIZE; i++) {
        buf[i] = (void *)__get_free_page(GFP_KERNEL);
        if (!buf[i])
            goto err_free_buf;
    }

    return 0;

    err_free_buf:
    while (i-- > 0)
        free_page((unsigned long)buf[i]);

    return -ENOMEM;
}

static void moto_testmgr_buf(char *buf[XBUFSIZE])
{
    int i;

    for (i = 0; i < XBUFSIZE; i++)
        free_page((unsigned long)buf[i]);
}

static int moto_do_one_async_hash_op(struct ahash_request *req,
        struct moto_tcrypt_result *tr,
        int ret)
{
    if (ret == -EINPROGRESS || ret == -EBUSY) {
        ret = wait_for_completion_interruptible(&tr->completion);
        if (!ret)
            ret = tr->err;
        INIT_COMPLETION(tr->completion);
    }
    return ret;
}

static int moto_test_hash(struct crypto_ahash *tfm, 
        struct moto_hash_testvec *template,
        unsigned int tcount, bool use_digest, 
        int inject_fault)
{
    const char *algo = crypto_tfm_alg_driver_name(crypto_ahash_tfm(tfm));
    unsigned int i, j, k, temp;
    struct scatterlist sg[8];
    char result[64];
    struct ahash_request *req;
    struct moto_tcrypt_result tresult;
    void *hash_buff;
    char *xbuf[XBUFSIZE];
    int ret = -ENOMEM;

    if (moto_testmgr_alloc_buf(xbuf))
        goto out_nobuf;

    init_completion(&tresult.completion);

    req = ahash_request_alloc(tfm, GFP_KERNEL);
    if (!req) {
        printk(KERN_ERR 
                "moto_crypto: hash: Failed to allocate request for "
                "%s\n", algo);
        goto out_noreq;
    }
    ahash_request_set_callback(req, CRYPTO_TFM_REQ_MAY_BACKLOG,
            moto_tcrypt_complete, &tresult);

    j = 0;
    for (i = 0; i < tcount; i++) {
        if (template[i].np)
            continue;

        j++;
        memset(result, 0, 64);

        hash_buff = xbuf[0];

        memcpy(hash_buff, template[i].plaintext, template[i].psize);
        sg_init_one(&sg[0], hash_buff, template[i].psize);

        if (template[i].ksize) {
            crypto_ahash_clear_flags(tfm, ~0);
            ret = crypto_ahash_setkey(tfm, template[i].key,
                    template[i].ksize);
            if (ret) {
                printk(KERN_ERR 
                        "moto_crypto hash: setkey failed on "
                        "test %d for %s: ret=%d\n", j, algo,
                        -ret);
                goto out;
            }
        }

        ahash_request_set_crypt(req, sg, result, template[i].psize);
        if (use_digest) {
            ret = moto_do_one_async_hash_op(req, &tresult,
                    crypto_ahash_digest(req));
            if (ret) {
                printk(KERN_ERR 
                        "moto_crypto: hash: digest failed on "
                        "test %d for %s: ret=%d\n", 
                        j, algo, -ret);
                goto out;
            }
        } else {
            ret = moto_do_one_async_hash_op(req, &tresult,
                    crypto_ahash_init(req));
            if (ret) {
                printk(KERN_ERR 
                        "moto_crypto hash: init failed on "
                        "test %d for %s: ret=%d\n", 
                        j, algo, -ret);
                goto out;
            }
            ret = moto_do_one_async_hash_op(req, &tresult,
                    crypto_ahash_update(req));
            if (ret) {
                printk(KERN_ERR 
                        "moto_crypto hash: update failed on "
                        "test %d for %s: ret=%d\n", 
                        j, algo, -ret);
                goto out;
            }
            ret = moto_do_one_async_hash_op(req, &tresult,
                    crypto_ahash_final(req));
            if (ret) {
                printk(KERN_ERR 
                        "moto_crypto hash: final failed on "
                        "test %d for %s: ret=%d\n", 
                        j, algo, -ret);
                goto out;
            }
        }

#ifdef CONFIG_CRYPTO_MOTOROLA_FAULT_INJECTION
        if (inject_fault) {
            result[0] ^= 0xff;
        }
#endif

        if (memcmp(result, template[i].digest,
                crypto_ahash_digestsize(tfm))) {
            printk(KERN_ERR 
                    "moto_crypto: hash: Test %d failed for %s\n",
                    j, algo);
            moto_hexdump(result, crypto_ahash_digestsize(tfm));
            ret = -EINVAL;
            goto out;
        }
    }

    j = 0;
    for (i = 0; i < tcount; i++) {
        if (template[i].np) {
            j++;
            memset(result, 0, 64);

            temp = 0;
            sg_init_table(sg, template[i].np);
            ret = -EINVAL;
            for (k = 0; k < template[i].np; k++) {
                if (WARN_ON(offset_in_page(MOTO_IDX[k]) +
                        template[i].tap[k] > PAGE_SIZE))
                    goto out;
                sg_set_buf(&sg[k],
                        memcpy(xbuf[MOTO_IDX[k] >> PAGE_SHIFT] +
                                offset_in_page(MOTO_IDX[k]),
                                template[i].plaintext + temp,
                                template[i].tap[k]),
                                template[i].tap[k]);
                temp += template[i].tap[k];
            }

            if (template[i].ksize) {
                crypto_ahash_clear_flags(tfm, ~0);
                ret = crypto_ahash_setkey(tfm, template[i].key,
                        template[i].ksize);

                if (ret) {
                    printk(KERN_ERR 
                            "moto_crypto: hash: setkey "
                            "failed on chunking test %d "
                            "for %s: ret=%d\n", j, algo,
                            -ret);
                    goto out;
                }
            }

            ahash_request_set_crypt(req, sg, result,
                    template[i].psize);
            ret = crypto_ahash_digest(req);
            switch (ret) {
            case 0:
                break;
            case -EINPROGRESS:
            case -EBUSY:
                ret = wait_for_completion_interruptible(
                        &tresult.completion);
                if (!ret && !(ret = tresult.err)) {
                    INIT_COMPLETION(tresult.completion);
                    break;
                }
                /* fall through */
            default:
                printk(KERN_ERR
                        "moto_crypto: hash: digest failed "
                        "on chunking test %d for %s: "
                        "ret=%d\n", j, algo, -ret);
                goto out;
            }

            if (memcmp(result, template[i].digest,
                    crypto_ahash_digestsize(tfm))) {
                printk(KERN_ERR 
                        "moto_crypto: hash: Chunking test %d "
                        "failed for %s\n", j, algo);
                moto_hexdump(result, crypto_ahash_digestsize(tfm));
                ret = -EINVAL;
                goto out;
            }
        }
    }

    ret = 0;

    out:
    ahash_request_free(req);
    out_noreq:
    moto_testmgr_buf(xbuf);
    out_nobuf:
    return ret;
}

static int moto_test_skcipher(struct crypto_ablkcipher *tfm, int enc,
        struct moto_cipher_testvec *template, 
        unsigned int tcount, int inject_fault)
{
    const char *algo =
            crypto_tfm_alg_driver_name(crypto_ablkcipher_tfm(tfm));
    unsigned int i, j, k, n, temp;
    char *q;
    struct ablkcipher_request *req;
    struct scatterlist sg[8];
    const char *e;
    struct moto_tcrypt_result result;
    void *data;
    char iv[MAX_IVLEN];
    char *xbuf[XBUFSIZE];
    int ret = -ENOMEM;

    if (moto_testmgr_alloc_buf(xbuf))
        goto out_nobuf;

    if (enc == ENCRYPT)
        e = "encryption";
    else
        e = "decryption";

    init_completion(&result.completion);

    req = ablkcipher_request_alloc(tfm, GFP_KERNEL);
    if (!req) {
        printk(KERN_ERR 
                "moto_crypto: skcipher: Failed to allocate request "
                "for %s\n", algo);
        goto out;
    }

    ablkcipher_request_set_callback(req, CRYPTO_TFM_REQ_MAY_BACKLOG,
            moto_tcrypt_complete, &result);

    j = 0;
    for (i = 0; i < tcount; i++) {
        if (template[i].iv)
            memcpy(iv, template[i].iv, MAX_IVLEN);
        else
            memset(iv, 0, MAX_IVLEN);

        if (!(template[i].np)) {
            j++;

            ret = -EINVAL;
            if (WARN_ON(template[i].ilen > PAGE_SIZE))
                goto out;

            data = xbuf[0];
            memcpy(data, template[i].input, template[i].ilen);

            crypto_ablkcipher_clear_flags(tfm, ~0);
            if (template[i].wk)
                crypto_ablkcipher_set_flags(
                        tfm, CRYPTO_TFM_REQ_WEAK_KEY);

            ret = crypto_ablkcipher_setkey(tfm, template[i].key,
                    template[i].klen);
            if (!ret == template[i].fail) {
                printk(KERN_ERR 
                        "moto_crypto: skcipher: setkey failed "
                        "on test %d for %s: flags=%x\n", j,
                        algo, crypto_ablkcipher_get_flags(tfm));
                goto out;
            } else if (ret)
                continue;

            sg_init_one(&sg[0], data, template[i].ilen);

            ablkcipher_request_set_crypt(req, sg, sg,
                    template[i].ilen, iv);
            ret = enc ?
                    crypto_ablkcipher_encrypt(req) :
                    crypto_ablkcipher_decrypt(req);

            switch (ret) {
            case 0:
                break;
            case -EINPROGRESS:
            case -EBUSY:
                ret = wait_for_completion_interruptible(
                        &result.completion);
                if (!ret && !((ret = result.err))) {
                    INIT_COMPLETION(result.completion);
                    break;
                }
                /* fall through */
            default:
                printk(KERN_ERR 
                        "moto_crypto: skcipher: %s failed on "
                        "test %d for %s: ret=%d\n", e, j, algo,
                        -ret);
                goto out;
            }

            q = data;

#ifdef CONFIG_CRYPTO_MOTOROLA_FAULT_INJECTION
            if (inject_fault == INJECT_FAULT_ALL_KEY_LENGHTS ||
                    inject_fault == (template[i].klen * 8)) {
                q[0] ^= 0xff;
            }
#endif
            if (memcmp(q, template[i].result, template[i].rlen)) {
                printk(KERN_ERR 
                        "moto_crypto: skcipher: Test %d "
                        "failed on %s for %s\n", j, e, algo);
                moto_hexdump(q, template[i].rlen);
                ret = -EINVAL;
                goto out;
            }
        }
    }

    j = 0;
    for (i = 0; i < tcount; i++) {

        if (template[i].iv)
            memcpy(iv, template[i].iv, MAX_IVLEN);
        else
            memset(iv, 0, MAX_IVLEN);

        if (template[i].np) {
            j++;

            crypto_ablkcipher_clear_flags(tfm, ~0);
            if (template[i].wk)
                crypto_ablkcipher_set_flags(
                        tfm, CRYPTO_TFM_REQ_WEAK_KEY);

            ret = crypto_ablkcipher_setkey(tfm, template[i].key,
                    template[i].klen);
            if (!ret == template[i].fail) {
                printk(KERN_ERR 
                        "moto_crypto: skcipher: setkey failed "
                        "on chunk test %d for %s: flags=%x\n",
                        j, algo,
                        crypto_ablkcipher_get_flags(tfm));
                goto out;
            } else if (ret)
                continue;

            temp = 0;
            ret = -EINVAL;
            sg_init_table(sg, template[i].np);
            for (k = 0; k < template[i].np; k++) {
                if (WARN_ON(offset_in_page(MOTO_IDX[k]) +
                        template[i].tap[k] > PAGE_SIZE))
                    goto out;

                q = xbuf[MOTO_IDX[k] >> PAGE_SHIFT] +
                        offset_in_page(MOTO_IDX[k]);

                memcpy(q, template[i].input + temp,
                        template[i].tap[k]);

                if (offset_in_page(q) + template[i].tap[k] <
                        PAGE_SIZE)
                    q[template[i].tap[k]] = 0;

                sg_set_buf(&sg[k], q, template[i].tap[k]);

                temp += template[i].tap[k];
            }

            ablkcipher_request_set_crypt(req, sg, sg,
                    template[i].ilen, iv);

            ret = enc ?
                    crypto_ablkcipher_encrypt(req) :
                    crypto_ablkcipher_decrypt(req);

            switch (ret) {
            case 0:
                break;
            case -EINPROGRESS:
            case -EBUSY:
                ret = wait_for_completion_interruptible(
                        &result.completion);
                if (!ret && !((ret = result.err))) {
                    INIT_COMPLETION(result.completion);
                    break;
                }
                /* fall through */
            default:
                printk(KERN_ERR 
                        "moto_crypto: skcipher: %s failed on "
                        "chunk test %d for %s: ret=%d\n", e, j,
                        algo, -ret);
                goto out;
            }

            temp = 0;
            ret = -EINVAL;
            for (k = 0; k < template[i].np; k++) {
                q = xbuf[MOTO_IDX[k] >> PAGE_SHIFT] +
                        offset_in_page(MOTO_IDX[k]);

                if (memcmp(q, template[i].result + temp,
                        template[i].tap[k])) {
                    printk(KERN_ERR 
                            "moto_crypto: skcipher: Chunk "
                            "test %d failed on %s at page "
                            "%u for %s\n", j, e, k, algo);
                    moto_hexdump(q, template[i].tap[k]);
                    goto out;
                }

                q += template[i].tap[k];
                for (n = 0; offset_in_page(q + n) && q[n]; n++)
                    ;
                if (n) {
                    printk(KERN_ERR 
                            "moto_crypto: skcipher: "
                            "Result buffer corruption in "
                            "chunk test %d on %s at page "
                            "%u for %s: %u bytes:\n", j, e,
                            k, algo, n);
                    moto_hexdump(q, n);
                    goto out;
                }
                temp += template[i].tap[k];
            }
        }
    }

    ret = 0;

    out:
    ablkcipher_request_free(req);
    moto_testmgr_buf(xbuf);
    out_nobuf:
    return ret;
}


static int moto_test_cprng(struct crypto_rng *tfm, 
        struct moto_cprng_testvec *template,
        unsigned int tcount, int inject_fault)
{
    const char *algo = crypto_tfm_alg_driver_name(crypto_rng_tfm(tfm));
    int err = 0, i, j, seedsize;
    u8 *seed;
    char result[32];

    seedsize = crypto_rng_seedsize(tfm);

    seed = kmalloc(seedsize, GFP_KERNEL);
    if (!seed) {
        printk(KERN_ERR 
                "moto_crypto: cprng: Failed to allocate seed space "
                "for %s\n", algo);
        return -ENOMEM;
    }

    for (i = 0; i < tcount; i++) {
        memset(result, 0, 32);

        memcpy(seed, template[i].v, template[i].vlen);
        memcpy(seed + template[i].vlen, template[i].key,
                template[i].klen);
        memcpy(seed + template[i].vlen + template[i].klen,
                template[i].dt, template[i].dtlen);

        err = crypto_rng_reset(tfm, seed, seedsize);
        if (err) {
            printk(KERN_ERR 
                    "moto_crypto: cprng: Failed to reset rng "
                    "for %s\n", algo);
            goto out;
        }

        for (j = 0; j < template[i].loops; j++) {
            err = crypto_rng_get_bytes(tfm, result,
                    template[i].rlen);
            if (err != template[i].rlen) {
                printk(KERN_ERR 
                        "moto_crypto: cprng: Failed to obtain "
                        "the correct amount of random data for "
                        "%s (requested %d, got %d)\n", algo,
                        template[i].rlen, err);
                goto out;
            }
        }

#ifdef CONFIG_CRYPTO_MOTOROLA_FAULT_INJECTION
        if (inject_fault) {
            printk(KERN_WARNING 
                    "Moto crypto: injecting fault in RNG\n");
            result[0] ^= 0xff;
        }
#endif
        err = memcmp(result, template[i].result,
                template[i].rlen);
        if (err) {
            printk(KERN_ERR 
                    "moto_crypto: cprng: Test %d failed for %s\n",
                    i, algo);
            moto_hexdump(result, template[i].rlen);
            err = -EINVAL;
            goto out;
        }
    }

    out:
    kfree(seed);
    return err;
}

/* Tests for symmetric key ciphers */
static int moto_alg_test_skcipher(const struct moto_alg_test_desc *desc,
        const char *driver, u32 type, u32 mask)
{
    struct crypto_ablkcipher *tfm;
    int err = 0;
    int inject_fault = 0;

    printk(KERN_ERR 
            "moto_alg_test_skcipher driver=%s type=%d mask=%d\n", 
            driver, type, mask);
    tfm = crypto_alloc_ablkcipher(driver, type, mask);
    if (IS_ERR(tfm)) {
        printk(KERN_ERR 
                "moto_crypto: skcipher: Failed to load transform for "
                "%s: %ld\n", driver, PTR_ERR(tfm));
        return PTR_ERR(tfm);
    }

#ifdef CONFIG_CRYPTO_MOTOROLA_FAULT_INJECTION
    if (fault_injection_mask & desc->alg_id) {
        printk(KERN_WARNING 
                "Moto crypto: injecting fault in block cipher %s\n", 
                driver);
        inject_fault = INJECT_FAULT_ALL_KEY_LENGHTS;
    }
    if (!strcmp(driver, "moto-aes-ecb")) {
        if (fault_injection_mask & MOTO_CRYPTO_ALG_AES_ECB_128) {
            inject_fault = 128;
        }
        if (fault_injection_mask & MOTO_CRYPTO_ALG_AES_ECB_192) {
            inject_fault = 192;
        }
        if (fault_injection_mask & MOTO_CRYPTO_ALG_AES_ECB_256) {
            inject_fault = 256;
        }
    }
    if (!strcmp(driver, "moto-aes-cbc")) {
        if (fault_injection_mask & MOTO_CRYPTO_ALG_AES_CBC_128) {
            inject_fault = 128;
        }
        if (fault_injection_mask & MOTO_CRYPTO_ALG_AES_CBC_192) {
            inject_fault = 192;
        }
        if (fault_injection_mask & MOTO_CRYPTO_ALG_AES_CBC_256) {
            inject_fault = 256;
        }
    }
    if (!strcmp(driver, "moto-aes-ctr")) {
        if (fault_injection_mask & MOTO_CRYPTO_ALG_AES_CTR_128) {
            inject_fault = 128;
        }
        if (fault_injection_mask & MOTO_CRYPTO_ALG_AES_CTR_192) {
            inject_fault = 192;
        }
        if (fault_injection_mask & MOTO_CRYPTO_ALG_AES_CTR_256) {
            inject_fault = 256;
        }
    }
#endif

    if (desc->suite.cipher.enc.vecs) {
        err = moto_test_skcipher(tfm, ENCRYPT, 
                desc->suite.cipher.enc.vecs,
                desc->suite.cipher.enc.count, 
                inject_fault);
        if (err)
            goto out;
    }

    if (desc->suite.cipher.dec.vecs)
        err = moto_test_skcipher(tfm, DECRYPT, 
                desc->suite.cipher.dec.vecs,
                desc->suite.cipher.dec.count, 
                inject_fault);

    out:
    crypto_free_ablkcipher(tfm);
    return err;
}

/* Test for hash functions */
static int moto_alg_test_hash(const struct moto_alg_test_desc *desc, 
        const char *driver, u32 type, u32 mask)
{
    struct crypto_ahash *tfm;
    int err;
    int inject_fault = 0;

    tfm = crypto_alloc_ahash(driver, type, mask);
    if (IS_ERR(tfm)) {
        printk(KERN_ERR 
                "moto_crypto: hash: Failed to load transform for %s: "
                "%ld\n", driver, PTR_ERR(tfm));
        return PTR_ERR(tfm);
    }

#ifdef CONFIG_CRYPTO_MOTOROLA_FAULT_INJECTION
    if (fault_injection_mask & desc->alg_id) {
        printk(KERN_WARNING 
                "Moto crypto: injecting fault in hash %s\n", driver);
        inject_fault = 1;
    }
#endif

    err = moto_test_hash(tfm, desc->suite.hash.vecs,
            desc->suite.hash.count, true, inject_fault);
    if (!err)
        err = moto_test_hash(tfm, desc->suite.hash.vecs,
                desc->suite.hash.count, false, 
                inject_fault);

    crypto_free_ahash(tfm);
    return err;
}

/* Test for RNG */
static int moto_alg_test_cprng(const struct moto_alg_test_desc *desc, 
        const char *driver, u32 type, u32 mask)
{
    struct crypto_rng *rng;
    int err;
    int inject_fault = 0;

    rng = crypto_alloc_rng(driver, type, mask);
    if (IS_ERR(rng)) {
        printk(KERN_ERR 
                "moto_crypto: cprng: Failed to load transform for %s: "
                "%ld\n", driver, PTR_ERR(rng));
        return PTR_ERR(rng);
    }

#ifdef CONFIG_CRYPTO_MOTOROLA_FAULT_INJECTION
    if (fault_injection_mask & desc->alg_id) {
        inject_fault = 1;
    }
#endif

    err = moto_test_cprng(rng, desc->suite.cprng.vecs, 
            desc->suite.cprng.count, inject_fault);

    crypto_free_rng(rng);

    return err;
}

/* Please keep this list sorted by algorithm name. */
static const struct moto_alg_test_desc moto_alg_test_descs[] = {
        {
                .alg = "ansi_cprng",
                .test = moto_alg_test_cprng,
                .alg_id = MOTO_CRYPTO_ALG_CPRNG,
                .suite = {
                        .cprng = {
                                .vecs = moto_ansi_cprng_aes_tv_template,
                                .count = ANSI_CPRNG_AES_TEST_VECTORS
                        }
                }
        }, {
                .alg = "cbc(aes)",
                .test = moto_alg_test_skcipher,
                .suite = {
                        .cipher = {
                                .enc = {
                                        .vecs = moto_aes_cbc_enc_tv_template,
                                        .count = AES_CBC_ENC_TEST_VECTORS
                                },
                                .dec = {
                                        .vecs = moto_aes_cbc_dec_tv_template,
                                        .count = AES_CBC_DEC_TEST_VECTORS
                                }
                        }
                }
        }, {
                .alg = "cbc(des3_ede)",
                .test = moto_alg_test_skcipher,
                .alg_id = MOTO_CRYPTO_ALG_TDES_CBC,
                .suite = {
                        .cipher = {
                                .enc = {
                                        .vecs = moto_des3_ede_cbc_enc_tv_template,
                                        .count = DES3_EDE_CBC_ENC_TEST_VECTORS
                                },
                                .dec = {
                                        .vecs = moto_des3_ede_cbc_dec_tv_template,
                                        .count = DES3_EDE_CBC_DEC_TEST_VECTORS
                                }
                        }
                }
        }, {
                .alg = "ctr(aes)",
                .test = moto_alg_test_skcipher,
                .suite = {
                        .cipher = {
                                .enc = {
                                        .vecs = moto_aes_ctr_enc_tv_template,
                                        .count = AES_CTR_ENC_TEST_VECTORS
                                },
                                .dec = {
                                        .vecs = moto_aes_ctr_dec_tv_template,
                                        .count = AES_CTR_DEC_TEST_VECTORS
                                }
                        }
                }
        }, {
                .alg = "ecb(aes)",
                .test = moto_alg_test_skcipher,
                .suite = {
                        .cipher = {
                                .enc = {
                                        .vecs = moto_aes_enc_tv_template,
                                        .count = AES_ENC_TEST_VECTORS
                                },
                                .dec = {
                                        .vecs = moto_aes_dec_tv_template,
                                        .count = AES_DEC_TEST_VECTORS
                                }
                        }
                }
        }, {
                .alg = "ecb(des3_ede)",
                .test = moto_alg_test_skcipher,
                .alg_id = MOTO_CRYPTO_ALG_TDES_ECB,
                .suite = {
                        .cipher = {
                                .enc = {
                                        .vecs = moto_des3_ede_enc_tv_template,
                                        .count = DES3_EDE_ENC_TEST_VECTORS
                                },
                                .dec = {
                                        .vecs = moto_des3_ede_dec_tv_template,
                                        .count = DES3_EDE_DEC_TEST_VECTORS
                                }
                        }
                }
        }, {
                .alg = "moto_hmac(moto-sha1)",
                .test = moto_alg_test_hash,
                .alg_id = MOTO_CRYPTO_ALG_HMAC_SHA1,
                .suite = {
                        .hash = {
                                .vecs = moto_hmac_sha1_tv_template,
                                .count = HMAC_SHA1_TEST_VECTORS
                        }
                }
        }, {
                .alg = "moto_hmac(moto-sha224)",
                .test = moto_alg_test_hash,
                .alg_id = MOTO_CRYPTO_ALG_HMAC_SHA224,
                .suite = {
                        .hash = {
                                .vecs = moto_hmac_sha224_tv_template,
                                .count = HMAC_SHA224_TEST_VECTORS
                        }
                }
        }, {
                .alg = "moto_hmac(moto-sha256)",
                .test = moto_alg_test_hash,
                .alg_id = MOTO_CRYPTO_ALG_HMAC_SHA256,
                .suite = {
                        .hash = {
                                .vecs = moto_hmac_sha256_tv_template,
                                .count = HMAC_SHA256_TEST_VECTORS
                        }
                }
        }, {
                .alg = "moto_hmac(moto-sha384)",
                .test = moto_alg_test_hash,
                .alg_id = MOTO_CRYPTO_ALG_HMAC_SHA384,
                .suite = {
                        .hash = {
                                .vecs = moto_hmac_sha384_tv_template,
                                .count = HMAC_SHA384_TEST_VECTORS
                        }
                }
        }, {
                .alg = "moto_hmac(moto-sha512)",
                .test = moto_alg_test_hash,
                .alg_id = MOTO_CRYPTO_ALG_HMAC_SHA512,
                .suite = {
                        .hash = {
                                .vecs = moto_hmac_sha512_tv_template,
                                .count = HMAC_SHA512_TEST_VECTORS
                        }
                }
        }, {
                .alg = "sha1",
                .test = moto_alg_test_hash,
                .alg_id = MOTO_CRYPTO_ALG_SHA1,
                .suite = {
                        .hash = {
                                .vecs = moto_sha1_tv_template,
                                .count = SHA1_TEST_VECTORS
                        }
                }
        }, {
                .alg = "sha224",
                .test = moto_alg_test_hash,
                .alg_id = MOTO_CRYPTO_ALG_SHA224,
                .suite = {
                        .hash = {
                                .vecs = moto_sha224_tv_template,
                                .count = SHA224_TEST_VECTORS
                        }
                }
        }, {
                .alg = "sha256",
                .test = moto_alg_test_hash,
                .alg_id = MOTO_CRYPTO_ALG_SHA256,
                .suite = {
                        .hash = {
                                .vecs = moto_sha256_tv_template,
                                .count = SHA256_TEST_VECTORS
                        }
                }
        }, {
                .alg = "sha384",
                .test = moto_alg_test_hash,
                .alg_id = MOTO_CRYPTO_ALG_SHA384,
                .suite = {
                        .hash = {
                                .vecs = moto_sha384_tv_template,
                                .count = SHA384_TEST_VECTORS
                        }
                }
        }, {
                .alg = "sha512",
                .test = moto_alg_test_hash,
                .alg_id = MOTO_CRYPTO_ALG_SHA512,
                .suite = {
                        .hash = {
                                .vecs = moto_sha512_tv_template,
                                .count = SHA512_TEST_VECTORS
                        }
                }
        }
};

/* Finds the position in the array of tests description based on the */
/* algorithm name                                                    */
static int moto_alg_find_test(const char *alg)
{
    int start = 0;
    int end = ARRAY_SIZE(moto_alg_test_descs);

    while (start < end) {
        int i = (start + end) / 2;
        int diff = strcmp(moto_alg_test_descs[i].alg, alg);

        if (diff > 0) {
            end = i;
            continue;
        }

        if (diff < 0) {
            start = i + 1;
            continue;
        }

        return i;
    }

    return -1;
}

/* Entry point for algorithm tests */
int moto_alg_test(const char *driver, const char *alg, u32 type, u32 mask)
{
    int i;
    int j;
    int rc;

    i = moto_alg_find_test(alg);
    j = moto_alg_find_test(driver);
    if (i < 0 && j < 0)
        goto notest;

    rc = 0;
    if (i >= 0)
        rc |= moto_alg_test_descs[i].test(moto_alg_test_descs + i, 
                driver, type, mask);
    else if (j >= 0)
        rc |= moto_alg_test_descs[j].test(moto_alg_test_descs + j, 
                driver, type, mask);

    if (!rc)
        printk(KERN_INFO 
                "moto_crypto: self-tests for %s (%s) passed\n",
                driver, alg);
    else
        printk(KERN_ERR  
                "moto_crypto: self-tests for %s (%s) NOT passed\n",
                driver, alg);

    return rc;

    notest:
    printk(KERN_INFO "moto_crypto: No test for %s (%s)\n", alg, driver);
    return 0;
}
