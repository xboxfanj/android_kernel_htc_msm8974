#ifndef _MOTO_CRYPTO_TEST_H
#define _MOTO_CRYPTO_TEST_H

#define MAX_TAP			8

#define MAX_IVLEN		32

struct moto_test_hash_testvec {
    /* only used with keyed hash algorithms */
    int count;
    char *key;
    char *plaintext;
    char *test_file_name;
    int psize;
    unsigned short ksize;
};

struct moto_test_cipher_testvec {
    char *test_file_name;
    int count;
    char *key;
    char *iv;
    char *input;
    unsigned short tap[MAX_TAP];
    unsigned char fail;
    unsigned char wk; /* weak key flag */
    unsigned char klen;
    unsigned short ilen;
};

struct moto_test_cprng_testvec {
    char *test_file_name;
    int count;
    char *key;
    char *dt;
    char *v;
    unsigned char klen;
    unsigned short dtlen;
    unsigned short vlen;
    unsigned short rlen;
    unsigned short loops;
};

#define MOTO_CRYPTO_ALG_SHA1        0x00000001
#define MOTO_CRYPTO_ALG_SHA224      0x00000002
#define MOTO_CRYPTO_ALG_SHA256      0x00000004
#define MOTO_CRYPTO_ALG_SHA384      0x00000008
#define MOTO_CRYPTO_ALG_SHA512      0x00000010
#define MOTO_CRYPTO_ALG_HMAC_SHA1   0x00000020
#define MOTO_CRYPTO_ALG_HMAC_SHA224 0x00000040
#define MOTO_CRYPTO_ALG_HMAC_SHA256 0x00000080
#define MOTO_CRYPTO_ALG_HMAC_SHA384 0x00000100
#define MOTO_CRYPTO_ALG_HMAC_SHA512 0x00000200
#define MOTO_CRYPTO_ALG_AES_ECB_128 0x00000400
#define MOTO_CRYPTO_ALG_AES_ECB_192 0x00000800
#define MOTO_CRYPTO_ALG_AES_ECB_256 0x00001000
#define MOTO_CRYPTO_ALG_AES_CBC_128 0x00002000
#define MOTO_CRYPTO_ALG_AES_CBC_192 0x00004000
#define MOTO_CRYPTO_ALG_AES_CBC_256 0x00008000
#define MOTO_CRYPTO_ALG_AES_CTR_128 0x00010000
#define MOTO_CRYPTO_ALG_AES_CTR_192 0x00020000
#define MOTO_CRYPTO_ALG_AES_CTR_256 0x00040000
#define MOTO_CRYPTO_ALG_TDES_ECB    0x00080000
#define MOTO_CRYPTO_ALG_TDES_CBC    0x00100000
#define MOTO_CRYPTO_ALG_CPRNG       0x00200000

#define MOTO_CRYPTO_MODULE_INTEGRITY 0x00400000

#endif

