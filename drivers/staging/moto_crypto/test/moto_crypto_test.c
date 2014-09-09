#include <crypto/hash.h>
#include <linux/err.h>
#include <linux/string.h>
#include <crypto/rng.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>

#include "moto_crypto_test.h"

#define TEST_AES
#define TEST_SHA1
#define TEST_SHA224
#define TEST_SHA256
#define TEST_SHA384
#define TEST_SHA512
#define TEST_TDES
#define TEST_RNG
#define TEST_HMAC

#define XBUFSIZE 8

#define ENCRYPT 1
#define DECRYPT 0

static const char nbits[256] =
{
        0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
        1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8,
};

static struct file *fp;
static unsigned long long file_offset;

struct moto_tcrypt_result {
    struct completion completion;
    int err;
};

struct moto_cipher_test_suite {
    struct {
        struct moto_test_cipher_testvec *vecs;
        unsigned int count;
    } enc, dec;
};

struct moto_hash_test_suite {
    struct moto_test_hash_testvec *vecs;
    unsigned int count;
};

struct moto_cprng_test_suite {
    struct moto_test_cprng_testvec *vecs;
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

static int dummy_param = 0;

static int moto_alg_test_skcipher(const struct moto_alg_test_desc *desc,
        const char *driver, u32 type, u32 mask);

static int moto_alg_test_hash(const struct moto_alg_test_desc *desc, 
        const char *driver, u32 type, u32 mask);

static int moto_alg_test_cprng(const struct moto_alg_test_desc *desc, 
        const char *driver, u32 type, u32 mask);

static void moto_xor_byte(u8 *a, const u8 *b, unsigned int size)
{
    for (; size; size--)
        *a++ ^= *b++;
}

static void moto_xor(u8 *dst, const u8 *src, unsigned int size)
{
    u32 *a = (u32 *)dst;
    u32 *b = (u32 *)src;

    for (; size >= 4; size -= 4)
        *a++ ^= *b++;

    moto_xor_byte((u8 *)a, (u8 *)b, size);
}

static void buffer_to_hex(char* hex, unsigned char *buf, unsigned int len) 
{
    int i;
    char ch;

    if (len <= 0 || buf == NULL || hex == NULL) {
        return;
    }
    hex[0] = '!';
    for (i = 0; i < len; i++) {
        ch = buf[i];
        hex[i*3+1] = hex_asc_hi(ch);
        hex[i*3+2] = hex_asc_lo(ch);
        hex[i*3+3] = ' ';
    }
    hex[3*len] = '!';
}

static struct file* file_open(const char* path, int flags, int rights) 
{
    struct file* filp = NULL;
    mm_segment_t oldfs;
    int err = 0;

    oldfs = get_fs();
    set_fs(get_ds());
    filp = filp_open(path, flags, rights);
    set_fs(oldfs);
    if(IS_ERR(filp)) {
        err = PTR_ERR(filp);
        printk(KERN_ERR "Error %d opening file\n", err);
        return NULL;
    }
    file_offset = 0;
    return filp;
}

static void file_close(struct file* file) {
    filp_close(file, NULL);
}

static int file_write(struct file* file, unsigned long long offset, unsigned char* data, unsigned int size) {
    mm_segment_t oldfs;
    int ret;

    oldfs = get_fs();
    set_fs(get_ds());

    ret = vfs_write(file, data, size, &offset);

    set_fs(oldfs);
    return ret;
}

static void write_result(unsigned char *buf, unsigned int len, const char* prefix, ...)
{
    va_list args;
    char* hex = NULL;
    char* prefix_buf = NULL;
    int count = 0;
    int wrote = 0;

    if (!fp) {
        return;
    }
    prefix_buf = kmalloc(256, GFP_KERNEL);
    if (prefix_buf == NULL) {
        printk(KERN_ERR "Error allocating prefix_buf\n");
        return;
    }
    va_start(args, prefix);
    count = vscnprintf(prefix_buf, 256, prefix, args);
    va_end(args);
    hex = kmalloc(3*len + 3 + count, GFP_KERNEL);
    if (hex == NULL) {
        printk(KERN_ERR "Error allocating hex\n");
        kfree(prefix_buf);
        return;
    }
    memcpy(hex, prefix_buf, count);
    kfree(prefix_buf);
    buffer_to_hex(hex + count, buf, len);
    hex[3*len+1+count] = '\n';
    hex[3*len+2+count] = 0;
    wrote = file_write(fp, file_offset, hex, 3*len + 2 + count);
    file_offset += wrote;

    kfree(hex);
}

static void write_quadruple_result(unsigned char *key, unsigned int key_len, unsigned char *iv, unsigned int iv_len, unsigned char *pt, unsigned int pt_len, unsigned char *buf, unsigned int len, const char* prefix, ...)
{
    va_list args;
    char* hex = NULL;
    char* prefix_buf = NULL;
    int count = 0;
    int wrote = 0;
    int total = 0;

    if (!fp) {
        return;
    }
    prefix_buf = kmalloc(256, GFP_KERNEL);
    if (prefix_buf == NULL) {
        printk(KERN_ERR "Error allocating prefix_buf\n");
        return;
    }
    va_start(args, prefix);
    count = vscnprintf(prefix_buf, 256, prefix, args);
    va_end(args);
    
    hex = kmalloc(3*len + 3*key_len + 3*iv_len + 3*pt_len + 6 + count, GFP_KERNEL);
    if (hex == NULL) {
        printk(KERN_ERR "Error allocating hex\n");
        kfree(prefix_buf);
        return;
    }
    memcpy(hex, prefix_buf, count);
    kfree(prefix_buf);
    total = count;
    buffer_to_hex(hex + total, buf, len);
    total += 3*len + 1;
    buffer_to_hex(hex + total, key, key_len);
    total += 3*key_len + 1;
    buffer_to_hex(hex + total, pt, pt_len);
    total += 3*pt_len + 1;
    if (iv_len > 0) {
        buffer_to_hex(hex + total, iv, iv_len);
        total += 3*iv_len + 1;
    }
    hex[total] = '\n';
    hex[total + 1] = 0;
    
    wrote = file_write(fp, file_offset, hex, total + 1);
    file_offset += wrote;

    kfree(hex);
}

/* Allocate a scatterlist for a vmalloc block. The scatterlist is allocated
   with kmalloc. Buffers of arbitrary alignment are supported.
   This function is derived from other vmalloc_to_sg functions in the kernel
   tree, but note that its second argument is a size in bytes, not in pages.
 */
static struct scatterlist *vmalloc_to_sg(unsigned char *const buf,
        size_t const bytes)
{
    struct scatterlist *sg_array = NULL;
    struct page *pg;
    /* Allow non-page-aligned pointers, so the first and last page may
	   both be partial. */
    unsigned const page_count = bytes / PAGE_SIZE + 2;
    unsigned char *ptr;
    unsigned i;

    sg_array = kcalloc(page_count, sizeof(*sg_array), GFP_KERNEL);
    if (sg_array == NULL) {
        printk(KERN_ERR "Error allocation sg_array page_count=%d\n", page_count);
        goto abort;
    }
    sg_init_table(sg_array, page_count);
    for (i = 0, ptr = (void *)((unsigned long)buf & PAGE_MASK);
            ptr < buf + bytes;
            i++, ptr += PAGE_SIZE) {
        pg = vmalloc_to_page(ptr);
        if (pg == NULL) {
            printk(KERN_ERR "vmalloc_to_page failed i=%d\n", i);
            goto abort;
        }
        sg_set_page(&sg_array[i], pg, PAGE_SIZE, 0);
    }
    /* Rectify the first page which may be partial. The last page may
	   also be partial but its offset is correct so it doesn't matter. */
    sg_array[0].offset = offset_in_page(buf);
    sg_array[0].length = PAGE_SIZE - offset_in_page(buf);
    return sg_array;
    abort:
    if (sg_array != NULL)
        kfree(sg_array);
    return NULL;
}

#ifdef TEST_AES
#include "alg_test_aes.c" 
#endif

#ifdef TEST_SHA1
#include "alg_test_sha1.c" 
#endif

#ifdef TEST_SHA224
#include "alg_test_sha224.c" 
#endif

#ifdef TEST_SHA256
#include "alg_test_sha256.c" 
#endif

#ifdef TEST_SHA384
#include "alg_test_sha384.c" 
#endif

#ifdef TEST_SHA512
#include "alg_test_sha512.c" 
#endif

#ifdef TEST_TDES
#include "alg_test_tdes.c" 
#endif

#ifdef TEST_RNG
#include "alg_test_rng.c" 
#endif

#ifdef TEST_HMAC
#include "alg_test_hmac.c" 
#endif

#include "alg_test.c" 

static void moto_tcrypt_complete(struct crypto_async_request *req, int err)
{
    struct moto_tcrypt_result *res = req->data;

    if (err == -EINPROGRESS)
        return;

    res->err = err;
    complete(&res->completion);
}

static int moto_testmgr_alloc_buf(char **buf)
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

static void moto_testmgr_buf(char **buf)
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
        struct moto_test_hash_testvec *template,
        unsigned int tcount)
{
    const char *algo = crypto_tfm_alg_driver_name(crypto_ahash_tfm(tfm));
    unsigned int i;
    struct scatterlist *sg = NULL;
    char result[64];
    struct ahash_request *req;
    struct moto_tcrypt_result tresult;
    int ret = -ENOMEM;
    int dsize = 0;
    int psize = 0;
    
    init_completion(&tresult.completion);

    req = ahash_request_alloc(tfm, GFP_KERNEL);
    if (!req) {
        printk(KERN_ERR 
                "moto_crypto: hash: Failed to allocate request for "
                "%s\n", algo);
        goto out;
    }
    ahash_request_set_callback(req, CRYPTO_TFM_REQ_MAY_BACKLOG,
            moto_tcrypt_complete, &tresult);

    for (i = 0; i < tcount; i++) {
        /* For HMAC tests, the size is in bytes, for SHA, the size is in bits */
        psize = (template[i].ksize) ? template[i].psize : template[i].psize/8;

        printk(KERN_INFO "Running test number %d psize on struct=%d psize in bytes=%d file=%s\n", i, template[i].psize, psize, template[i].test_file_name);

        memset(result, 0, 64);

        sg = vmalloc_to_sg(template[i].plaintext, psize);

        if (template[i].ksize) {
            crypto_ahash_clear_flags(tfm, ~0);
            ret = crypto_ahash_setkey(tfm, template[i].key,
                    template[i].ksize);
            if (ret) {
                printk(KERN_ERR 
                        "moto_crypto hash: setkey failed on "
                        "test %d for %s: ret=%d\n", i, algo,
                        -ret);
                goto out;
            }
        }	

        ahash_request_set_crypt(req, sg, result, psize);
        ret = moto_do_one_async_hash_op(req, &tresult,
                crypto_ahash_init(req));
        if (ret) {
            printk(KERN_ERR 
                    "moto_crypto hash: init failed on "
                    "test %d for %s: ret=%d\n", 
                    i, algo, -ret);
            goto out;
        }
        ret = moto_do_one_async_hash_op(req, &tresult,
                crypto_ahash_update(req));
        if (ret) {
            printk(KERN_ERR 
                    "moto_crypto hash: update failed on "
                    "test %d for %s: ret=%d\n", 
                    i, algo, -ret);
            goto out;
        }
        ret = moto_do_one_async_hash_op(req, &tresult,
                crypto_ahash_final(req));
        if (ret) {
            printk(KERN_ERR 
                    "moto_crypto hash: final failed on "
                    "test %d for %s: ret=%d\n", 
                    i, algo, -ret);
            goto out;
        }
        if (template[i].ksize) {
            /* HMAC */
            dsize = crypto_ahash_digestsize(tfm);
            write_result(result, dsize, "file:%s len:%d count:%d hash_size:%d ", template[i].test_file_name, template[i].psize, template[i].count, dsize);
        }
        else {
            write_result(result, crypto_ahash_digestsize(tfm), "file:%s len:%d ", template[i].test_file_name, template[i].psize);
        }
        kfree(sg);
        sg = NULL;
    }

    ret = 0;

    out:
    if (!IS_ERR_OR_NULL(req)) {
        ahash_request_free(req);
    }
    if (!IS_ERR_OR_NULL(sg)) {
        kfree(sg);
    }
    return ret;
}

static int moto_test_skcipher(struct crypto_ablkcipher *tfm, int enc,
        struct moto_test_cipher_testvec *template, 
        unsigned int tcount)
{
    const char *algo =
            crypto_tfm_alg_driver_name(crypto_ablkcipher_tfm(tfm));
    unsigned int i, j;
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

        printk(KERN_INFO "Running test number %d\n", i);
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

        write_result(q, template[i].ilen, "file:%s count:%d enc:%d ", template[i].test_file_name, template[i].count, enc);
    }

    ret = 0;

    out:
    ablkcipher_request_free(req);
    moto_testmgr_buf(xbuf);
    out_nobuf:
    return ret;
}


static int moto_test_cprng(struct crypto_rng *tfm, 
        struct moto_test_cprng_testvec *template,
        unsigned int tcount)
{
    const char *algo = crypto_tfm_alg_driver_name(crypto_rng_tfm(tfm));
    int err = 0, i, j, seedsize;
    u8 *seed;
    char result[16];

    seedsize = crypto_rng_seedsize(tfm);

    seed = kmalloc(seedsize, GFP_KERNEL);
    if (!seed) {
        printk(KERN_ERR 
                "moto_crypto: cprng: Failed to allocate seed space "
                "for %s\n", algo);
        return -ENOMEM;
    }

    for (i = 0; i < tcount; i++) {
        printk(KERN_INFO "Running test number %d\n", i);

        memset(result, 0, 16);

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
        write_result(result, template[i].rlen, "file:%s count:%d ", template[i].test_file_name, template[i].count);
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

    if (desc->suite.cipher.enc.vecs) {
        printk(KERN_INFO "Running encryption tests\n");
        err = moto_test_skcipher(tfm, ENCRYPT, 
                desc->suite.cipher.enc.vecs,
                desc->suite.cipher.enc.count);
        if (err)
            goto out;
    }

    if (desc->suite.cipher.dec.vecs)
        printk(KERN_INFO "Running decryption tests\n");
    err = moto_test_skcipher(tfm, DECRYPT, 
            desc->suite.cipher.dec.vecs,
            desc->suite.cipher.dec.count);

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

    tfm = crypto_alloc_ahash(driver, type, mask);
    if (IS_ERR(tfm)) {
        printk(KERN_ERR 
                "moto_crypto: hash: Failed to load transform for %s: "
                "%ld\n", driver, PTR_ERR(tfm));
        return PTR_ERR(tfm);
    }

    err = moto_test_hash(tfm, desc->suite.hash.vecs,
            desc->suite.hash.count);

    crypto_free_ahash(tfm);
    return err;
}


/* Test for RNG */
static int moto_alg_test_cprng(const struct moto_alg_test_desc *desc, 
        const char *driver, u32 type, u32 mask)
{
    struct crypto_rng *rng;
    int err;

    rng = crypto_alloc_rng(driver, type, mask);
    if (IS_ERR(rng)) {
        printk(KERN_ERR 
                "moto_crypto: cprng: Failed to load transform for %s: "
                "%ld\n", driver, PTR_ERR(rng));
        return PTR_ERR(rng);
    }

    err = moto_test_cprng(rng, desc->suite.cprng.vecs, 
            desc->suite.cprng.count);

    crypto_free_rng(rng);

    return err;
}


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
int moto_test_alg_test(const char *driver, const char *alg, u32 type, u32 mask)
{
    int i = -1;
    int j = -1;
    int rc;

    if (alg != NULL)
        i = moto_alg_find_test(alg);
    if (driver != NULL)
        j = moto_alg_find_test(driver);
    if (i < 0 && j < 0)
        goto notest;

    rc = 0;
    if (i >= 0)
        rc |= moto_alg_test_descs[i].test(moto_alg_test_descs + i, 
                driver, type, mask);
    if (j >= 0)
        rc |= moto_alg_test_descs[j].test(moto_alg_test_descs + j, 
                driver, type, mask);


    return rc;

    notest:
    printk(KERN_INFO "crypto_test: No test for %s (%s)\n", alg, driver);
    return 0;
}

static void adjust_tdes_key_parity(char* key)
{
    int i;
    int count;
    
    for (i = 0; i < 24; i++)
    {
        count = nbits[(unsigned int)key[i]];
        if (count % 2 == 0) 
        {
            key[i] ^= 0x01;
        }
    }
}

static void moto_test_tdes_ecb_monte(struct moto_test_cipher_testvec* test_data, int enc) 
{
    char buf[8];
    char dstbuf[8];
    char round_key[24];
    char but_last[8];
    char but_but_last[8];
    char pt[8];
    struct scatterlist sg[1];
    struct scatterlist dst[1];
    struct blkcipher_desc desc = {NULL, 0};
    int error, i, j;
    int k1k2, k1k3, k2k3;

    if (test_data->ilen != 8) {
        printk(KERN_ERR "Unsupported input size: %d\n", test_data->ilen);
        goto abort;
    }
    desc.flags = 0;
    desc.tfm = crypto_alloc_blkcipher("ecb(des3_ede)", 0, 0);
    if (IS_ERR_OR_NULL(desc.tfm)) {
        printk(KERN_ERR "crypto_alloc_cipher failed\n");
        error = (desc.tfm == NULL ? -ENOMEM : (int)desc.tfm);
        goto abort;
    }

    sg_init_one(sg, buf, 8);

    sg_init_one(dst, dstbuf, 8);

    memcpy(buf, test_data->input, test_data->ilen);
    memcpy(round_key, test_data->key, test_data->klen);
    memset(but_last, 0, 8);
    
    for (i = 0; i < 400; i++) {
        memcpy(pt, buf, 8);
        error = crypto_blkcipher_setkey(desc.tfm, round_key, test_data->klen);
        if (error) {
            printk(KERN_ERR "setkey failed\n");
            goto abort;
        }
        for (j = 0; j < 10000; j++) {
            if (enc)
                error = crypto_blkcipher_encrypt(&desc, dst, sg, 8);
            else
                error = crypto_blkcipher_decrypt(&desc, dst, sg, 8);
            if (error) {
                printk(KERN_ERR "crypto_blkcipher_encrypt/decrypt iteration %d,%d failed: %d\n",
                        i, j, error);
                goto abort;
            }
            memcpy(but_but_last, but_last, 8);
            memcpy(but_last, buf, 8);
            memcpy(buf, dstbuf, 8);
        }
        write_quadruple_result(round_key, 24, NULL, 0, pt, 8, dstbuf, 8, "TDESMonte file:%s enc:%d count:%d ", test_data->test_file_name, enc, i);
        k1k2 = memcmp(round_key, round_key + 8, 8);
        k1k3 = memcmp(round_key, round_key + 16, 8);
        k2k3 = memcmp(round_key + 8, round_key + 16, 8);
        moto_xor(round_key, dstbuf, 8);
        if ((k1k2 && !k1k3) || (k1k2 && k2k3 && k1k3)) {
            moto_xor(round_key + 8, but_last, 8);
        }
        else {
            moto_xor(round_key + 8, dstbuf, 8);
        }
        if ((!k1k2 && !k1k3 && !k2k3) || (k1k2 && !k1k3)) {
            moto_xor(round_key + 16, dstbuf, 8);
        }
        else {
            moto_xor(round_key + 16, but_but_last, 8);
        }
        adjust_tdes_key_parity(round_key);
    }

    abort:
    if (!IS_ERR_OR_NULL(desc.tfm))
        crypto_free_blkcipher(desc.tfm);
}

static void moto_test_tdes_cbc_dec_monte(struct moto_test_cipher_testvec* test_data) 
{
    char buf[8];
    char dstbuf[8];
    char round_key[24];
    char round_iv[8];
    char but_last[8];
    char but_but_last[8];
    char pt[8];
    char iv_zero[8];
    struct scatterlist sg[1];
    struct scatterlist dst[1];
    struct blkcipher_desc desc = {NULL, 0};
    int error, i, j;
    int k1k2, k1k3, k2k3;

    if (test_data->ilen != 8) {
        printk(KERN_ERR "Unsupported input size: %d\n", test_data->ilen);
        goto abort;
    }
    desc.flags = 0;
    desc.tfm = crypto_alloc_blkcipher("cbc(des3_ede)", 0, 0);
    if (IS_ERR_OR_NULL(desc.tfm)) {
        printk(KERN_ERR "crypto_alloc_cipher failed\n");
        error = (desc.tfm == NULL ? -ENOMEM : (int)desc.tfm);
        goto abort;
    }

    sg_init_one(sg, buf, 8);

    sg_init_one(dst, dstbuf, 8);

    memcpy(buf, test_data->input, test_data->ilen);
    memcpy(round_key, test_data->key, test_data->klen);
    memset(but_last, 0, 8);
    
    memcpy(round_iv, test_data->iv, 8);
    
    for (i = 0; i < 400; i++) {
        memcpy(pt, buf, 8);
        error = crypto_blkcipher_setkey(desc.tfm, round_key, test_data->klen);
        if (error) {
            printk(KERN_ERR "setkey failed\n");
            goto abort;
        }
        memcpy(iv_zero, round_iv, 8);

        for (j = 0; j < 10000; j++) {
            crypto_blkcipher_set_iv(desc.tfm, round_iv, 8);
            error = crypto_blkcipher_decrypt(&desc, dst, sg, 8);
            if (error) {
                printk(KERN_ERR "crypto_blkcipher_encrypt/decrypt iteration %d,%d failed: %d\n",
                        i, j, error);
                goto abort;
            }
            memcpy(round_iv, buf, 8);
            memcpy(but_but_last, but_last, 8);
            memcpy(but_last, buf, 8);
            memcpy(buf, dstbuf, 8);
        }
        write_quadruple_result(round_key, 24, iv_zero, 8, pt, 8, dstbuf, 8, "TDESMonte file:%s enc:0 count:%d ", test_data->test_file_name, i);
        k1k2 = memcmp(round_key, round_key + 8, 8);
        k1k3 = memcmp(round_key, round_key + 16, 8);
        k2k3 = memcmp(round_key + 8, round_key + 16, 8);
        moto_xor(round_key, dstbuf, 8);
        if ((k1k2 && !k1k3) || (k1k2 && k2k3 && k1k3)) {
            moto_xor(round_key + 8, but_last, 8);
        }
        else {
            moto_xor(round_key + 8, dstbuf, 8);
        }
        if ((!k1k2 && !k1k3 && !k2k3) || (k1k2 && !k1k3)) {
            moto_xor(round_key + 16, dstbuf, 8);
        }
        else {
            moto_xor(round_key + 16, but_but_last, 8);
        }
        adjust_tdes_key_parity(round_key);
    }

    abort:
    if (!IS_ERR_OR_NULL(desc.tfm))
        crypto_free_blkcipher(desc.tfm);
}


static void moto_test_tdes_cbc_enc_monte(struct moto_test_cipher_testvec* test_data) 
{
    char buf[8];
    char dstbuf[8];
    char round_key[24];
    char round_iv[8];
    char last[8];
    char but_last[8];
    char but_but_last[8];
    char pt[8];
    char iv_zero[8];
    struct scatterlist sg[1];
    struct scatterlist dst[1];
    struct blkcipher_desc desc = {NULL, 0};
    int error, i, j;
    int k1k2, k1k3, k2k3;

    if (test_data->ilen != 8) {
        printk(KERN_ERR "Unsupported input size: %d\n", test_data->ilen);
        goto abort;
    }
    desc.flags = 0;
    desc.tfm = crypto_alloc_blkcipher("cbc(des3_ede)", 0, 0);
    if (IS_ERR_OR_NULL(desc.tfm)) {
        printk(KERN_ERR "crypto_alloc_cipher failed\n");
        error = (desc.tfm == NULL ? -ENOMEM : (int)desc.tfm);
        goto abort;
    }

    sg_init_one(sg, buf, 8);

    sg_init_one(dst, dstbuf, 8);

    memcpy(buf, test_data->input, test_data->ilen);
    memcpy(round_key, test_data->key, test_data->klen);
    memset(but_last, 0, 8);
    memset(last, 0, 8);
    
    memcpy(round_iv, test_data->iv, 8);
    
    for (i = 0; i < 400; i++) {
        memcpy(pt, buf, 8);
        error = crypto_blkcipher_setkey(desc.tfm, round_key, test_data->klen);
        if (error) {
            printk(KERN_ERR "setkey failed\n");
            goto abort;
        }
        memcpy(iv_zero, round_iv, 8);

        for (j = 0; j < 10000; j++) {
            crypto_blkcipher_set_iv(desc.tfm, round_iv, 8);
            error = crypto_blkcipher_encrypt(&desc, dst, sg, 8);
            if (error) {
                printk(KERN_ERR "crypto_blkcipher_encrypt/decrypt iteration %d,%d failed: %d\n",
                        i, j, error);
                goto abort;
            }
            if (j == 0) {
                memcpy(buf, round_iv, 8);
            }
            else {
                memcpy(buf, last, 8);
            }
            memcpy(but_but_last, but_last, 8);
            memcpy(but_last, last, 8);
            memcpy(last, dstbuf, 8);
            
            memcpy(round_iv, dstbuf, 8);
        }
        write_quadruple_result(round_key, 24, iv_zero, 8, pt, 8, dstbuf, 8, "TDESMonte file:%s enc:1 count:%d ", test_data->test_file_name, i);
        k1k2 = memcmp(round_key, round_key + 8, 8);
        k1k3 = memcmp(round_key, round_key + 16, 8);
        k2k3 = memcmp(round_key + 8, round_key + 16, 8);
        moto_xor(round_key, dstbuf, 8);
        if ((k1k2 && !k1k3) || (k1k2 && k2k3 && k1k3)) {
            moto_xor(round_key + 8, but_last, 8);
        }
        else {
            moto_xor(round_key + 8, dstbuf, 8);
        }
        if ((!k1k2 && !k1k3 && !k2k3) || (k1k2 && !k1k3)) {
            moto_xor(round_key + 16, dstbuf, 8);
        }
        else {
            moto_xor(round_key + 16, but_but_last, 8);
        }
        adjust_tdes_key_parity(round_key);
    }

    abort:
    if (!IS_ERR_OR_NULL(desc.tfm))
        crypto_free_blkcipher(desc.tfm);
}

static void moto_test_aes_ecb_monte(struct moto_test_cipher_testvec* test_data, int enc) 
{
    char *buf = NULL;
    char *dstbuf = NULL;
    char* round_key = NULL;
    char* xor_key = NULL;
    char* but_last = NULL;
    char* pt = NULL;
    struct scatterlist sg[1];
    struct scatterlist dst[1];
    struct blkcipher_desc desc = {NULL, 0};
    int error, i, j;

    if (test_data->ilen != 16) {
        printk(KERN_ERR "Unsupported input size: %d\n", test_data->ilen);
        goto abort;
    }
    desc.flags = 0;
    desc.tfm = crypto_alloc_blkcipher("ecb(aes)", 0, 0);
    if (IS_ERR_OR_NULL(desc.tfm)) {
        printk(KERN_ERR "crypto_alloc_cipher failed\n");
        error = (desc.tfm == NULL ? -ENOMEM : (int)desc.tfm);
        goto abort;
    }

    buf = kmalloc(16, GFP_KERNEL);
    if (IS_ERR_OR_NULL(buf)) {
        printk(KERN_ERR "kmalloc for buf failed\n");
        error = (buf == NULL ? -ENOMEM : (int)buf);
        goto abort;
    }

    sg_init_one(sg, buf, 16);

    dstbuf = kmalloc(16, GFP_KERNEL);
    if (IS_ERR_OR_NULL(dstbuf)) {
        printk(KERN_ERR "kmalloc for dstbuf failed\n");
        error = (dstbuf == NULL ? -ENOMEM : (int)dstbuf);
        goto abort;
    }

    sg_init_one(dst, dstbuf, 16);

    round_key = kmalloc(test_data->klen, GFP_KERNEL);
    if (IS_ERR_OR_NULL(round_key)) {
        printk(KERN_ERR "kmalloc for round_key failed\n");
        error = (round_key == NULL ? -ENOMEM : (int)round_key);
        goto abort;
    }

    xor_key = kmalloc(test_data->klen, GFP_KERNEL);
    if (IS_ERR_OR_NULL(xor_key)) {
        printk(KERN_ERR "kmalloc for xor_key failed\n");
        error = (xor_key == NULL ? -ENOMEM : (int)xor_key);
        goto abort;
    }

    but_last = kmalloc(16, GFP_KERNEL);
    if (IS_ERR_OR_NULL(but_last)) {
        printk(KERN_ERR "kmalloc for but_last failed\n");
        error = (but_last == NULL ? -ENOMEM : (int)but_last);
        goto abort;
    }

    pt = kmalloc(16, GFP_KERNEL);
    if (IS_ERR_OR_NULL(pt)) {
        printk(KERN_ERR "kmalloc for pt failed\n");
        error = (pt == NULL ? -ENOMEM : (int)pt);
        goto abort;
    }

    memcpy(round_key, test_data->key, test_data->klen);
    memcpy(buf, test_data->input, 16);

    for (i = 0; i < 100; i++) {
        memcpy(pt, buf, 16);
        for (j = 0; j < 1000; j++) {
            error = crypto_blkcipher_setkey(desc.tfm, round_key, test_data->klen);
            if (error) {
                printk(KERN_ERR "setkey failed round (%d,%d)\n", i, j);
                goto abort;
            }
            if (enc)
                error = crypto_blkcipher_encrypt(&desc, dst, sg, 16);
            else
                error = crypto_blkcipher_decrypt(&desc, dst, sg, 16);
            memcpy(but_last, buf, 16);
            memcpy(buf, dstbuf, 16);
        }
        write_quadruple_result(round_key, test_data->klen, NULL, 0, pt, 16, dstbuf, 16, "AESMonte file:%s enc:%d count:%d ", test_data->test_file_name, enc, i);
        switch (test_data->klen) {
        case 16:
            memcpy(xor_key, dstbuf, 16);
            break;
        case 24:
            memcpy(xor_key, but_last + 8, 8);
            memcpy(xor_key + 8, dstbuf, 16);
            break;
        case 32:
            memcpy(xor_key, but_last, 16);
            memcpy(xor_key + 16, dstbuf, 16);
            break;
        }
        moto_xor(round_key, xor_key, test_data->klen);
    }

    abort:
    if (!IS_ERR_OR_NULL(buf))
        kfree(buf);
    if (!IS_ERR_OR_NULL(dstbuf))
        kfree(dstbuf);
    if (!IS_ERR_OR_NULL(round_key))
        kfree(round_key);
    if (!IS_ERR_OR_NULL(xor_key))
        kfree(xor_key);
    if (!IS_ERR_OR_NULL(but_last))
        kfree(but_last);
    if (!IS_ERR_OR_NULL(pt))
        kfree(pt);
    if (!IS_ERR_OR_NULL(desc.tfm))
        crypto_free_blkcipher(desc.tfm);
}

static void moto_test_aes_cbc_monte(struct moto_test_cipher_testvec* test_data, int enc) 
{
    char *buf = NULL;
    char *dstbuf = NULL;
    char* round_key = NULL;
    char *round_iv = NULL;
    char* xor_key = NULL;
    char* but_last = NULL;
    char *pt = NULL;
    struct scatterlist sg[1];
    struct scatterlist dst[1];
    struct blkcipher_desc desc = {NULL, 0};
    int error, i, j;

    if (test_data->ilen != 16) {
        printk(KERN_ERR "Unsupported input size: %d\n", test_data->ilen);
        goto abort;
    }
    desc.flags = 0;
    desc.tfm = crypto_alloc_blkcipher("cbc(aes)", 0, 0);
    if (IS_ERR_OR_NULL(desc.tfm)) {
        printk(KERN_ERR "crypto_alloc_cipher failed\n");
        error = (desc.tfm == NULL ? -ENOMEM : (int)desc.tfm);
        goto abort;
    }

    buf = kmalloc(16, GFP_KERNEL);
    if (IS_ERR_OR_NULL(buf)) {
        printk(KERN_ERR "kmalloc for buf failed\n");
        error = (buf == NULL ? -ENOMEM : (int)buf);
        goto abort;
    }

    sg_init_one(sg, buf, 16);

    dstbuf = kmalloc(16, GFP_KERNEL);
    if (IS_ERR_OR_NULL(dstbuf)) {
        printk(KERN_ERR "kmalloc for dstbuf failed\n");
        error = (dstbuf == NULL ? -ENOMEM : (int)dstbuf);
        goto abort;
    }

    sg_init_one(dst, dstbuf, 16);

    round_key = kmalloc(test_data->klen, GFP_KERNEL);
    if (IS_ERR_OR_NULL(round_key)) {
        printk(KERN_ERR "kmalloc for round_key failed\n");
        error = (round_key == NULL ? -ENOMEM : (int)round_key);
        goto abort;
    }

    round_iv = kmalloc(16, GFP_KERNEL);
    if (IS_ERR_OR_NULL(round_iv)) {
        printk(KERN_ERR "kmalloc for round_iv failed\n");
        error = (round_iv == NULL ? -ENOMEM : (int)round_iv);
        goto abort;
    }

    xor_key = kmalloc(test_data->klen, GFP_KERNEL);
    if (IS_ERR_OR_NULL(xor_key)) {
        printk(KERN_ERR "kmalloc for xor_key failed\n");
        error = (xor_key == NULL ? -ENOMEM : (int)xor_key);
        goto abort;
    }

    but_last = kmalloc(16, GFP_KERNEL);
    if (IS_ERR_OR_NULL(but_last)) {
        printk(KERN_ERR "kmalloc for but_last failed\n");
        error = (but_last == NULL ? -ENOMEM : (int)but_last);
        goto abort;
    }

    pt = kmalloc(16, GFP_KERNEL);
    if (IS_ERR_OR_NULL(pt)) {
        printk(KERN_ERR "kmalloc for pt failed\n");
        error = (pt == NULL ? -ENOMEM : (int)pt);
        goto abort;
    }

    memcpy(round_key, test_data->key, test_data->klen);
    memcpy(buf, test_data->input, 16);
    memcpy(round_iv, test_data->iv, 16);
    
    for (i = 0; i < 100; i++) {
        memcpy(pt, buf, 16);
        for (j = 0; j < 1000; j++) {
            error = crypto_blkcipher_setkey(desc.tfm, round_key, test_data->klen);
            if (error) {
                printk(KERN_ERR "setkey failed round (%d,%d)\n", i, j);
                goto abort;
            }
            if (j == 0) {
                crypto_blkcipher_set_iv(desc.tfm, round_iv, 16);
            }
            if (enc)
                error = crypto_blkcipher_encrypt(&desc, dst, sg, 16);
            else
                error = crypto_blkcipher_decrypt(&desc, dst, sg, 16);
            if (j == 0) {
                memcpy(buf, round_iv, 16);
            }
            else {
                memcpy(buf, but_last, 16);
            }
            memcpy(but_last, dstbuf, 16);
        }
        write_quadruple_result(round_key, test_data->klen, round_iv, 16, pt, 16, dstbuf, 16, "AESMonte file:%s enc:%d count:%d ", test_data->test_file_name, enc, i);
        switch (test_data->klen) {
        case 16:
            memcpy(xor_key, dstbuf, 16);
            break;
        case 24:
            memcpy(xor_key, buf + 8, 8);
            memcpy(xor_key + 8, dstbuf, 16);
            break;
        case 32:
            memcpy(xor_key, buf, 16);
            memcpy(xor_key + 16, dstbuf, 16);
            break;
        }
        moto_xor(round_key, xor_key, test_data->klen);
        memcpy(round_iv, dstbuf, 16);
    }

    abort:
    if (!IS_ERR_OR_NULL(buf))
        kfree(buf);
    if (!IS_ERR_OR_NULL(dstbuf))
        kfree(dstbuf);
    if (!IS_ERR_OR_NULL(round_key))
        kfree(round_key);
    if (!IS_ERR_OR_NULL(xor_key))
        kfree(xor_key);
    if (!IS_ERR_OR_NULL(round_iv))
        kfree(round_iv);
    if (!IS_ERR_OR_NULL(but_last))
        kfree(but_last);
    if (!IS_ERR_OR_NULL(pt))
        kfree(pt);
    if (!IS_ERR_OR_NULL(desc.tfm))
        crypto_free_blkcipher(desc.tfm);
}

static void moto_test_hash_monte(const char* alg_name, char* seed) {
    char *buf = NULL;
    char *out = NULL;
    char* round_seed = NULL;
    struct scatterlist sg[1];
    struct hash_desc desc = {NULL, 0};
    size_t digest_length;
    int error, i, j;

    desc.tfm = crypto_alloc_hash(alg_name, 0, 0);
    if (IS_ERR_OR_NULL(desc.tfm)) {
        printk(KERN_ERR "crypto_alloc_hash(%s) failed\n", alg_name);
        error = (desc.tfm == NULL ? -ENOMEM : (int)desc.tfm);
        goto abort;
    }
    digest_length = crypto_hash_digestsize(desc.tfm);
    printk(KERN_INFO "Monte test alg_name=%s driver_name=%s digest_length=%u\n",
            alg_name,
            crypto_tfm_alg_driver_name(crypto_hash_tfm(desc.tfm)),
            digest_length);

    out = (char *)kmalloc(digest_length, GFP_KERNEL);
    if (IS_ERR_OR_NULL(out)) {
        printk(KERN_ERR "allocating out memory failed\n");
        error = (out == NULL ? -ENOMEM : (int)out);
        goto abort;
    }

    round_seed = (char *)kmalloc(digest_length, GFP_KERNEL);
    if (IS_ERR_OR_NULL(round_seed)) {
        printk(KERN_ERR "allocating round_seed memory failed\n");
        error = (round_seed == NULL ? -ENOMEM : (int)round_seed);
        goto abort;
    }

    buf = (char *)kmalloc(3 * digest_length, GFP_KERNEL);
    if (IS_ERR_OR_NULL(buf)) {
        printk(KERN_ERR "allocating buffer memory failed\n");
        error = (buf == NULL ? -ENOMEM : (int)buf);
        goto abort;
    }

    memcpy(round_seed, seed, digest_length);
    
    sg_init_one(sg, buf, 3 * digest_length);

    for (j = 0; j < 100; j++) {
        memcpy(buf, round_seed, digest_length);
        memcpy(buf + digest_length, round_seed, digest_length);
        memcpy(buf + 2 * digest_length, round_seed, digest_length);
        for (i = 3; i < 1003; i++) {
            error = crypto_hash_digest(&desc, sg, 3 * digest_length, out);
            if (error) {
                printk(KERN_ERR "crypto_hash_digest(%s) iteration (%d, %d) failed: %d\n",
                        alg_name, j, i, error);
                goto abort;
            }
            memcpy(buf, buf + digest_length, digest_length);
            memcpy(buf + digest_length, buf + 2 * digest_length, digest_length);
            memcpy(buf + 2 * digest_length, out, digest_length);
        }
        write_result(out, digest_length, "Monte alg:%s count:%d ", alg_name, j);
        memcpy(round_seed, out, digest_length);
    }

abort:
    if (!IS_ERR_OR_NULL(buf))
        kfree(buf);
    if (!IS_ERR_OR_NULL(out))
        kfree(out);
    if (!IS_ERR_OR_NULL(round_seed))
        kfree(round_seed);
    if (!IS_ERR_OR_NULL(desc.tfm))
        crypto_free_hash(desc.tfm);
}

/* Module entry point */
static int __init moto_crypto_test_init(void)
{
    unsigned long start_jiffies;
    long diff;
    int rc;
#ifdef TEST_AES
    int i;
#endif
    
    printk(KERN_INFO "moto_crypto_test_init\n");

    start_jiffies = jiffies;
    fp = file_open("/sdcard/algtest.txt", O_WRONLY | O_TRUNC | O_CREAT | O_LARGEFILE, 0);
#ifdef TEST_AES
    printk(KERN_INFO "Starting ECB AES tests\n");
    for (i = 0; i < ARRAY_SIZE(moto_aes_ecb_monte_enc); i++) {
        moto_test_aes_ecb_monte(&moto_aes_ecb_monte_enc[i], ENCRYPT);
    }
    for (i = 0; i < ARRAY_SIZE(moto_aes_ecb_monte_dec); i++) {
        moto_test_aes_ecb_monte(&moto_aes_ecb_monte_dec[i], DECRYPT);
    }
    rc = moto_test_alg_test("moto-aes-ecb", "ecb(aes)", 0, 0); 
    printk(KERN_INFO "Starting CBC AES tests\n");
    for (i = 0; i < ARRAY_SIZE(moto_aes_cbc_monte_enc); i++) {
        moto_test_aes_cbc_monte(&moto_aes_cbc_monte_enc[i], ENCRYPT);
    }
    for (i = 0; i < ARRAY_SIZE(moto_aes_cbc_monte_dec); i++) {
        moto_test_aes_cbc_monte(&moto_aes_cbc_monte_dec[i], DECRYPT);
    }
    rc = moto_test_alg_test("moto-aes-cbc", "cbc(aes)", 0, 0);
#endif
#ifdef TEST_SHA1
    printk(KERN_INFO "Starting SHA-1 tests\n");
    rc = moto_test_alg_test("moto-sha1", "sha1", 0, 0);
    moto_test_hash_monte("moto-sha1", moto_test_monte_SHA1);
#endif
#ifdef TEST_SHA224
    printk(KERN_INFO "Starting SHA-224 tests\n");
    rc = moto_test_alg_test("moto-sha224", "sha224", 0, 0);
    moto_test_hash_monte("moto-sha224", moto_test_monte_SHA224);
#endif
#ifdef TEST_SHA256
    printk(KERN_INFO "Starting SHA-256 tests\n");
    rc = moto_test_alg_test("moto-sha256", "sha256", 0, 0);
    moto_test_hash_monte("moto-sha256", moto_test_monte_SHA256);
#endif
#ifdef TEST_SHA384
    printk(KERN_INFO "Starting SHA-384 tests\n");
    rc = moto_test_alg_test("moto-sha384", "sha384", 0, 0);
    moto_test_hash_monte("moto-sha384", moto_test_monte_SHA384);
#endif
#ifdef TEST_SHA512
    printk(KERN_INFO "Starting SHA-512 tests\n");
    rc = moto_test_alg_test("moto-sha512", "sha512", 0, 0);
    moto_test_hash_monte("moto-sha512", moto_test_monte_SHA512);
#endif
#ifdef TEST_HMAC
    printk(KERN_INFO "Starting HMAC(SHA-1) tests\n");
    rc = moto_test_alg_test("moto_hmac(moto-sha1)", NULL, 0, 0);
    printk(KERN_INFO "Starting HMAC(SHA-224) tests\n");
    rc = moto_test_alg_test("moto_hmac(moto-sha224)", NULL, 0, 0);
    printk(KERN_INFO "Starting HMAC(SHA-256) tests\n");
    rc = moto_test_alg_test("moto_hmac(moto-sha256)", NULL, 0, 0);
    printk(KERN_INFO "Starting HMAC(SHA-384) tests\n");
    rc = moto_test_alg_test("moto_hmac(moto-sha384)", NULL, 0, 0);
    printk(KERN_INFO "Starting HMAC(SHA-512) tests\n");
    rc = moto_test_alg_test("moto_hmac(moto-sha512)", NULL, 0, 0);
#endif
#ifdef TEST_RNG
    printk(KERN_INFO "Starting RNG tests\n");
    rc = moto_test_alg_test("moto_fips_ansi_cprng", "ansi_cprng", 0, 0);
#endif
#ifdef TEST_TDES
    printk(KERN_INFO "Starting ECB TDES tests\n");
    rc = moto_test_alg_test("moto-des3-ecb", "ecb(des3_ede)", 0, 0); 
    printk(KERN_INFO "Starting ECB TDES Monte tests\n");
    moto_test_tdes_ecb_monte(&moto_TECBMonte1_enc, ENCRYPT);
    moto_test_tdes_ecb_monte(&moto_TECBMonte2_enc, ENCRYPT);
    moto_test_tdes_ecb_monte(&moto_TECBMonte3_enc, ENCRYPT);
    moto_test_tdes_ecb_monte(&moto_TECBMonte1_dec, DECRYPT);
    moto_test_tdes_ecb_monte(&moto_TECBMonte2_dec, DECRYPT);
    moto_test_tdes_ecb_monte(&moto_TECBMonte3_dec, DECRYPT);
    printk(KERN_INFO "Starting CBC TDES tests\n");
    rc = moto_test_alg_test("moto-des3-cbc", "cbc(des3_ede)", 0, 0);
    printk(KERN_INFO "Starting CBC TDES Monte tests\n");
    moto_test_tdes_cbc_enc_monte(&moto_TCBCMonte1_enc);
    moto_test_tdes_cbc_enc_monte(&moto_TCBCMonte2_enc);
    moto_test_tdes_cbc_enc_monte(&moto_TCBCMonte3_enc);
    moto_test_tdes_cbc_dec_monte(&moto_TCBCMonte1_dec);
    moto_test_tdes_cbc_dec_monte(&moto_TCBCMonte2_dec);
    moto_test_tdes_cbc_dec_monte(&moto_TCBCMonte3_dec);
#endif

    if (fp) {
        file_close(fp);
    }
    fp = NULL;
    diff = (long)jiffies - (long)start_jiffies;
    printk(KERN_INFO "moto_crypto_test: Time to test: %ld msec\n", 
            diff * 1000 / HZ);
    return 0;
}

/* Module finalization function */
static void __exit moto_crypto_test_fini(void)
{
    printk(KERN_INFO "moto_crypto_test_fini\n");
    /* The calls below are just to avoid compiler warning */
    if (dummy_param) {
        printk(KERN_ERR "dummy_param not 0\n");
        moto_test_alg_test(NULL, NULL, 0, 0);
        moto_test_tdes_ecb_monte(NULL, 0);
        moto_test_tdes_cbc_enc_monte(NULL);
        moto_test_tdes_cbc_dec_monte(NULL);
        moto_test_aes_ecb_monte(NULL, 0);
        moto_test_aes_cbc_monte(NULL, 0);
        moto_test_hash(NULL, NULL, 0);
        moto_test_cprng(NULL, NULL, 0);
        moto_test_skcipher(NULL, 0, NULL, 0);
        moto_alg_test_skcipher(NULL, NULL, 0, 0);
        moto_alg_test_cprng(NULL, NULL, 0, 0);
        moto_alg_test_hash(NULL, NULL, 0, 0);
        moto_test_hash_monte(NULL, NULL);
        memset(&moto_TECBMonte1_enc, 0, 10);
        memset(&moto_TECBMonte2_enc, 0, 10);
        memset(&moto_TECBMonte3_enc, 0, 10);
        memset(&moto_TECBMonte1_dec, 0, 10);
        memset(&moto_TECBMonte2_dec, 0, 10);
        memset(&moto_TECBMonte3_dec, 0, 10);
        memset(&moto_TCBCMonte1_enc, 0, 10);
        memset(&moto_TCBCMonte2_enc, 0, 10);
        memset(&moto_TCBCMonte3_enc, 0, 10);
        memset(&moto_TCBCMonte1_dec, 0, 10);
        memset(&moto_TCBCMonte2_dec, 0, 10);
        memset(&moto_TCBCMonte3_dec, 0, 10);
    }
}


module_init(moto_crypto_test_init);
module_exit(moto_crypto_test_fini);

module_param_named(dummy, dummy_param, int, 0);
MODULE_PARM_DESC(dummy, "Dummy param, always 0");

MODULE_DESCRIPTION("Motorola cryptographic module test");
MODULE_LICENSE("GPL");
MODULE_ALIAS("crypto_test");
