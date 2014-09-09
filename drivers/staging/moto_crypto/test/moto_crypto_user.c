#include <crypto/hash.h>
#include <crypto/rng.h>
#include <linux/jiffies.h>
#include <linux/scatterlist.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/module.h>

static void moto_hexdump(unsigned char *buf, unsigned int len)
{
    print_hex_dump(KERN_CONT, "", DUMP_PREFIX_OFFSET,
            16, 1,
            buf, len, false);
}

static void moto_crypto_hmac_test(void) 
{
    unsigned char actual[32];
    struct scatterlist sg;
    struct hash_desc desc = {NULL, 0};
    size_t digest_length;
    unsigned char const key[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    size_t const key_length = 16;
    int error;
    unsigned char* data = NULL;
    size_t data_length = 0;

    printk(KERN_INFO "Running HMAC\n");

    desc.tfm = crypto_alloc_hash("moto_hmac(moto-sha256)", 0, 0);
    if (IS_ERR_OR_NULL(desc.tfm)) {
        printk(KERN_ERR "crypto_alloc_hash failed\n");
        error = (desc.tfm == NULL ? -ENOMEM : (int)desc.tfm);
        goto abort;
    }
    digest_length = crypto_hash_digestsize(desc.tfm);
    printk(KERN_INFO "driver_name=%s digest_length=%u\n",
            crypto_tfm_alg_driver_name(crypto_hash_tfm(desc.tfm)),
            digest_length);

    error = crypto_hash_setkey(desc.tfm, key, key_length);
    if (error) {
        printk(KERN_ERR "crypto_hash_setkey failed: %d\n", error);
        goto abort;
    }

    data_length = 32;
    data = kcalloc(data_length, sizeof(unsigned char), GFP_KERNEL);
    memset(data, 65, data_length);
    sg_init_one(&sg, data, data_length);

    error = crypto_hash_digest(&desc, &sg, data_length, actual);
    if (error) {
        printk(KERN_ERR "crypto_hash_digest failed: %d\n", error);
        goto abort;
    }

    printk(KERN_INFO "About to free tfm\n");
    crypto_free_hash(desc.tfm);

    return;

    abort:
    if (!IS_ERR_OR_NULL(data))
        kfree(data);
    if (!IS_ERR_OR_NULL(desc.tfm))
        crypto_free_hash(desc.tfm);
    return;
}

static void moto_crypto_rng_test(void) 
{
    struct crypto_rng *rng = NULL;
    int err = 0, seedsize;
    char key[] = "\xf3\xb1\x66\x6d\x13\x60\x72\x42\xed\x06\x1c\xab\xb8\xd4\x62\x02";
    char v_seed[] = "\xf3\xb1\x66\x6d\x13\x60\x72\x42\xed\x06\x1c\xab\xb8\xd4\x62\x02";
    char dt_seed[] = "\xe6\xb3\xbe\x78\x2a\x23\xfa\x62\xd7\x1d\x4a\xfb\xb0\xe9\x22\xf9";
    u8 *seed = NULL;
    char result[32];
    const char *algo;

    rng = crypto_alloc_rng("ansi_cprng", 0, 0);
    if (IS_ERR(rng)) {
        printk(KERN_ERR 
                "moto_crypto_rng_test: cprng: Failed to load transform for rng: "
                "%ld\n", PTR_ERR(rng));
        goto out;
    }
    algo = crypto_tfm_alg_driver_name(crypto_rng_tfm(rng));
    printk(KERN_INFO "Allocated %s\n", algo);

    seedsize = crypto_rng_seedsize(rng);
    printk(KERN_INFO "Seedsize: %d\n", seedsize);

    seed = kmalloc(seedsize, GFP_KERNEL);
    if (!seed) {
        printk(KERN_ERR 
                "moto_crypto: cprng: Failed to allocate seed space "
                "for %s\n", algo);
        goto out;
    }

    memset(result, 0, 32);

    memcpy(seed, v_seed, 16);
    memcpy(seed + 16, key, 16);
    memcpy(seed + 32, dt_seed, 16);

    err = crypto_rng_reset(rng, seed, seedsize);
    if (err) {
        printk(KERN_ERR 
                "moto_crypto: cprng: Failed to reset rng "
                "for %s\n", algo);
        goto out;
    }

    err = crypto_rng_get_bytes(rng, result, 16);
    printk(KERN_INFO "moto_crypto: cprng: Got %d bytes\n", err);

    moto_hexdump(result, 16);

    out:
    if (!IS_ERR_OR_NULL(rng))
        crypto_free_rng(rng);
    if (!IS_ERR_OR_NULL(seed))
        kfree(seed);
}

/* Module entry point */
static int __init moto_crypto_user_init(void)
{
    unsigned long start_jiffies;
    long diff;

    printk(KERN_INFO "moto_crypto_user_init\n");

    moto_crypto_hmac_test();

    moto_crypto_rng_test();

    start_jiffies = jiffies;

    diff = (long)jiffies - (long)start_jiffies;
    printk(KERN_INFO "moto_crypto_user: Time to run: %ld msec\n", 
            diff * 1000 / HZ);
    return 0;
}

/* Module finalization function */
static void __exit moto_crypto_user_fini(void)
{
    printk(KERN_INFO "moto_crypto_user_fini\n");
}


module_init(moto_crypto_user_init);
module_exit(moto_crypto_user_fini);

MODULE_DESCRIPTION("Motorola cryptographic module user");
MODULE_LICENSE("GPL");
MODULE_ALIAS("crypto_user");

