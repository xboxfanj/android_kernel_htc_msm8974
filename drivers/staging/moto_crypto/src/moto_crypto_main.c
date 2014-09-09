#include <moto_aes.h>
#include <moto_tdes.h>
#include <moto_sha.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/scatterlist.h>
#include <linux/err.h>
#include <linux/sort.h>
#include <linux/elf.h>

#include "moto_crypto_main.h"

#define MOTO_CRYPTO_CLASS "moto_crypto"
#define MOTO_CRYPTO_FIPS_VERSION "1.0"
#define MOTO_CRYPTO_ATTR_FIPS_ENABLED     "fips_enabled"
#define MOTO_CRYPTO_ATTR_FIPS_VERSION     "fips_version"
#define MOTO_CRYPTO_ATTR_FIPS_POST_RESULT "fips_post_result"

extern int moto_prng_init(void);
extern void moto_prng_finish(void);

static char *moto_integrity_hmac_sha256_expected_value;

struct section_header_data
{
    unsigned char* name;
    unsigned long address;
    unsigned int size;
};

struct elf_section_headers
{
    struct section_header_data* sect_hdrs;
    unsigned int nsects;
};

/**
 * Holds FIPS crypto POST result:
 * 0: passed
 * other values: POST failed 
 */
static unsigned failures = 0;

unsigned char const moto_integrity_hmac_sha256_key[] = {
        0x3c, 0x09, 0x1d, 0x83, 0x74, 0x5f, 0x3e, 0xd3,
        0x2c, 0xab, 0x47, 0x45, 0x89, 0x50, 0xbc, 0xa6,
        0x48, 0x56, 0x1b, 0xc5, 0x4d, 0x73, 0x8f, 0xe5,
        0xee, 0x34, 0x23, 0x5f, 0xf1, 0x10, 0x0d, 0x4a
};

/**
 * Get the elf sections that are eligible for hash calculation
 * based solely on flags. Considered ones are sections that have
 * SHF_ALLOC and are not SHT_NOBITS.
 * The eligible sections have their information stored in the
 * elf_section_headers struct, that shall be previously allocated
 * by the caller.
 */
static int parse_elf_sections(Elf_Ehdr* elf_ehdr,
        unsigned long elf_ehdr_len,
        struct elf_section_headers* parsed_sections)
{
    int error = 0;
    Elf_Shdr *sechdrs;
    char *secstrings;
    struct section_header_data *curr_sec_hdr;
    struct section_header_data *sec_hdrs;
    int num_valid_hdrs = 0;
    int i;

    /* Set up the convenience variables */
    sechdrs = (void*)elf_ehdr + elf_ehdr->e_shoff;
    secstrings = (void*)elf_ehdr + sechdrs[elf_ehdr->e_shstrndx].sh_offset;

    /* This should always be true, but let's be sure. */
    sechdrs[0].sh_addr = 0;

    for (i = 1; i < elf_ehdr->e_shnum; i++) {
        Elf_Shdr *shdr = &sechdrs[i];

        /* Mark all sections sh_addr with their address in the
		   temporary image. */
        shdr->sh_addr = (size_t)elf_ehdr + shdr->sh_offset;
    }

    sec_hdrs = kzalloc(sizeof(struct section_header_data) * elf_ehdr->e_shnum, GFP_KERNEL);
    if (IS_ERR_OR_NULL(sec_hdrs)) {
        error = -ENOMEM;
        goto abort;
    }

    curr_sec_hdr = sec_hdrs;

    /* Go through all sections and select those that are SHF_ALLOC and not
     * SHT_NOBITS. */
    for (i = 0; i < elf_ehdr->e_shnum; i++) {
        Elf_Shdr *shdr = &sechdrs[i];

        if ((!(shdr->sh_flags & SHF_ALLOC)) ||
                (shdr->sh_type == SHT_NOBITS))
            continue;

        curr_sec_hdr->name = secstrings + shdr->sh_name;
        curr_sec_hdr->address = shdr->sh_addr;
        curr_sec_hdr->size = shdr->sh_size;
        curr_sec_hdr++;
        num_valid_hdrs++;
    }

    parsed_sections->sect_hdrs = sec_hdrs;
    parsed_sections->nsects = num_valid_hdrs;

    abort:

    return error;
}

static int section_header_data_name_cmp(const void* a, const void* b)
{
    const struct section_header_data* section_a = a;
    const struct section_header_data* section_b = b;

    return strcmp(section_a->name, section_b->name);
}

/**
 * Organize the module sections based on their names, so
 * we are able to calculate the hash always using the same
 * data order.
 */
static int moto_crypto_canonicalize(struct module *mod,
        struct elf_section_headers* elf_sections)
{
    void *canonicalized_buffer;
    char *data;
    struct section_header_data *curr_sec_hdr;

    unsigned long canonicalized_buffer_size = 0;
    unsigned int loop;
    int error = 0;

    /* 1 - Allocate a buffer to store the canonicalized section data */
    canonicalized_buffer = kmalloc(mod->raw_binary_size, GFP_KERNEL);
    if (IS_ERR_OR_NULL(canonicalized_buffer)) {
        error = -ENOMEM;
        goto abort;
    }

    /* 2 - Sort the sections in alphabetical order */
    sort(elf_sections->sect_hdrs,
            elf_sections->nsects,
            sizeof(struct section_header_data),
            section_header_data_name_cmp,
            NULL);

    /* 3 - Move the sections to the canonicalized buffer */

    /* At this stage we remove sections that don't add any value to the hash
     * security or that are modified due to kernel changes.
     * Below is a description of each one of the removed sections:
     *
     *  .symtab - main symbol table used in compile-time linking or
     *            runtime debugging.
     *  .strtab - NULL-terminated strings of names of symbols in
     *            .symtab section.
     *  Both .symtab and strtab can be stripped from the ELF file
     *  without causing any issues in code execution.
     *
     *  .modinfo - module info section. It contains the kernel release
     *             number for which the module was built and it
     *             describes the form of the module's parameters.
     *             Mainly used by insmod. Since it varies depending
     *             on the kernel release number, we remove it from
     *             the valid sections group.
     *
     *  .gnu.linkonce.this_module - stores the struct module. This is
     *             used by the sys_init_module() during module
     *             initialization. Since this is used just for
     *             initialization purposes, we remove it from the
     *             valid sections group.
     *
     *  .note.gnu.build-id - section used to store a unique build id
     *             for the kernel and its modules. Since this changes
     *             for every different kernel built, we remove it from
     *             the valid sections group.
     */
    canonicalized_buffer_size = 0;
    curr_sec_hdr = elf_sections->sect_hdrs;
    data = canonicalized_buffer;
    for (loop = 0; loop < elf_sections->nsects; loop++) {
        if (curr_sec_hdr->size > 0 &&
                (strcmp(curr_sec_hdr->name,".strtab") != 0 &&
                        strcmp(curr_sec_hdr->name,".symtab") != 0 &&
                        strcmp(curr_sec_hdr->name,".modinfo") != 0 &&
                        strcmp(curr_sec_hdr->name,"__versions") != 0 &&
                        strcmp(curr_sec_hdr->name,".gnu.linkonce.this_module") != 0 &&
                        strcmp(curr_sec_hdr->name,".note.gnu.build-id") != 0)) {
            memcpy(data, (void *)curr_sec_hdr->address, curr_sec_hdr->size);
            data += curr_sec_hdr->size;
            canonicalized_buffer_size += curr_sec_hdr->size;
        }
        curr_sec_hdr++;
    }

    /* 4 - Due to memory constraints, we will reuse the original buffer to
     * copy the canonicalized one.
     */
    mod->raw_binary_size = canonicalized_buffer_size;
    memcpy(mod->raw_binary_ptr, canonicalized_buffer, canonicalized_buffer_size);

    abort:
    if (canonicalized_buffer != NULL)
        kfree(canonicalized_buffer);
    return error;
} /* end moto_crypto_canonicalize() */

/**
 * Show handler for moto_crypto_class attributes
 */
static ssize_t moto_crypto_attr_show(struct class *class,
        struct class_attribute *attribute,
        char *buf)
{
    int n = 0;
    const char *attr_name = attribute->attr.name;
    if (!strcmp(MOTO_CRYPTO_ATTR_FIPS_ENABLED, attr_name)) {
        n = snprintf(buf, PAGE_SIZE, "1\n");
    }
    else if (!strcmp(MOTO_CRYPTO_ATTR_FIPS_VERSION, attr_name)) {
        n = snprintf(buf, PAGE_SIZE, "%s\n", MOTO_CRYPTO_FIPS_VERSION);
    }
    else if (!strcmp(MOTO_CRYPTO_ATTR_FIPS_POST_RESULT, attr_name)) {
        n = ((failures == 0) ?
                snprintf(buf, PAGE_SIZE, "0\n") :
                snprintf(buf, PAGE_SIZE, "0x%08x\n", failures));
    }
    printk(KERN_DEBUG "moto_crypto_attr_show: %s=%s",
            attr_name,
            n > 0 ? buf : "\n");
    return n;
}

/**
 * Attributes of moto_crypto_class
 */
static struct class_attribute moto_crypto_class_attrs[] = {
        {
                .attr = { .name = MOTO_CRYPTO_ATTR_FIPS_ENABLED, .mode = S_IRUGO },
                .show = moto_crypto_attr_show
        },
        {
                .attr = { .name = MOTO_CRYPTO_ATTR_FIPS_VERSION, .mode = S_IRUGO },
                .show = moto_crypto_attr_show
        },
        {
                .attr = { .name = MOTO_CRYPTO_ATTR_FIPS_POST_RESULT, .mode = S_IRUGO },
                .show = moto_crypto_attr_show
        },
        __ATTR_NULL
};

/**
 * sysfs class to expose Motorola FIPS crypto module status into user space
 * as text files in /sys/class/moto_crypto/
 */
static struct class moto_crypto_class = {
        .name        = MOTO_CRYPTO_CLASS,
        .class_attrs = moto_crypto_class_attrs,
};

static ssize_t scan_hex(unsigned char *const buf,
        size_t buf_size,
        const char *const hex)
{
    size_t bi = 0;
    size_t hi;
    unsigned prev = -1, cur;
    for (hi = 0; hex[hi] != 0; hi++) {
        if (hex[hi] >= '0' && hex[hi] <= '9')
            cur = hex[hi] - '0';
        else if (hex[hi] >= 'a' && hex[hi] <= 'f')
            cur = hex[hi] - 'a' + 10;
        else if (hex[hi] >= 'A' && hex[hi] <= 'f')
            cur = hex[hi] - 'F' + 10;
        else if (hex[hi] == '-' || hex[hi] == ' ')
            continue;
        else {
            printk(KERN_ERR
                    "invalid character at %zu (%u)", hi, hex[hi]);
            return -EINVAL;
        }
        if (prev == -1)
            prev = cur;
        else {
            if (bi >= buf_size) {
                printk(KERN_ERR "buffer too large at %zu", hi);
                return -ENOSPC;
            }
            buf[bi++] = prev << 4 | cur;
            prev = -1;
        }
    }
    return bi;
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
    if (sg_array == NULL)
        goto abort;

    sg_init_table(sg_array, page_count);
    for (i = 0, ptr = (void *)((unsigned long)buf & PAGE_MASK);
            ptr < buf + bytes;
            i++, ptr += PAGE_SIZE) {
        pg = vmalloc_to_page(ptr);
        if (pg == NULL)
            goto abort;

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

static int self_test_integrity(const char *alg_name, struct module *mod)
{
    unsigned char expected[32];
    unsigned char actual[32];
    struct scatterlist *sg = NULL;
    struct hash_desc desc = {NULL, 0};
    size_t digest_length;
    size_t const key_length = sizeof(moto_integrity_hmac_sha256_key);
    int error;
    struct elf_section_headers elf_sections;
    elf_sections.sect_hdrs = NULL;

    if (mod->raw_binary_ptr == NULL)
        return -ENXIO;
    if (moto_integrity_hmac_sha256_expected_value == NULL)
        return -ENOENT;
    error = scan_hex(expected, sizeof(expected),
            moto_integrity_hmac_sha256_expected_value);
    if (error < 0) {
        printk(KERN_ERR
                "FIPS module: Badly formatted hmac_sha256 parameter "
                "(should be a hex string)\n");
        return -EIO;
    };

    desc.tfm = crypto_alloc_hash(alg_name, 0, 0);
    if (IS_ERR_OR_NULL(desc.tfm)) {
        printk(KERN_ERR "crypto_alloc_hash(%s) failed\n", alg_name);
        error = (desc.tfm == NULL ? -ENOMEM : (int)desc.tfm);
        goto abort;
    }
    digest_length = crypto_hash_digestsize(desc.tfm);
    printk(KERN_INFO "alg_name=%s driver_name=%s digest_length=%u\n",
            alg_name,
            crypto_tfm_alg_driver_name(crypto_hash_tfm(desc.tfm)),
            digest_length);

    error = crypto_hash_setkey(desc.tfm, moto_integrity_hmac_sha256_key, 
            key_length);
    if (error) {
        printk(KERN_ERR "crypto_hash_setkey(%s) failed: %d\n",
                alg_name, error);
        goto abort;
    }

    error = parse_elf_sections(mod->raw_binary_ptr,
            mod->raw_binary_size,
            &elf_sections);
    if (error) {
        printk(KERN_ERR "parse_elf_sections() failed: %d\n",
                error);
        goto abort;
    }

    error = moto_crypto_canonicalize(mod, &elf_sections);
    if (error) {
        printk(KERN_ERR "moto_crypto_canonicalize() failed: %d\n",
                error);
        goto abort;
    }

    sg = vmalloc_to_sg(mod->raw_binary_ptr, mod->raw_binary_size);
    if (IS_ERR_OR_NULL(sg)) {
        printk(KERN_ERR "vmalloc_to_sg(%lu) failed: %d\n",
                mod->raw_binary_size, (int)sg);
        error = (sg == NULL ? -ENOMEM : (int)sg);
        goto abort;
    }

    error = crypto_hash_digest(&desc, sg, mod->raw_binary_size, actual);
    if (error) {
        printk(KERN_ERR "crypto_hash_digest(%s) failed: %d\n",
                alg_name, error);
        goto abort;
    }

    kfree(sg);
    crypto_free_hash(desc.tfm);

#ifdef CONFIG_CRYPTO_MOTOROLA_FAULT_INJECTION
    if (fault_injection_mask & MOTO_CRYPTO_MODULE_INTEGRITY) {
        printk(KERN_WARNING
                "Moto crypto: injecting fault in integrity check!\n");
        actual[0] ^= 0xff;
    }
#endif

    if (memcmp(expected, actual, digest_length)) {
        printk(KERN_ERR "wrong %s digest value\n", alg_name);
        error = -EINVAL;
    } else {
        printk(KERN_INFO "%s: digest successful\n", alg_name);
        error = 0;
    }

    return error;

    abort:
    if (!IS_ERR_OR_NULL(sg))
        kfree(sg);
    if (!IS_ERR_OR_NULL(desc.tfm))
        crypto_free_hash(desc.tfm);
    if (elf_sections.sect_hdrs != NULL)
        kfree(elf_sections.sect_hdrs);
    return error == -ENOMEM ? error : -EIO;
}
/* Module entry point */
static int __init moto_crypto_init(void)
{
    int err;
    unsigned long start_jiffies;
    long diff;

    /* FSM_TRANS:T1 */
    printk(KERN_INFO "moto_crypto_main: moto_crypto_init\n");
#ifdef CONFIG_CRYPTO_MOTOROLA_FAULT_INJECTION
    printk(KERN_INFO
            "moto_crypto_main: fault_injection_mask_string=%s\n",
            fault_injection_mask_string);
    if (fault_injection_mask_string != NULL) {
        sscanf(fault_injection_mask_string, "%x",
                &fault_injection_mask);
        printk(KERN_INFO
                "moto_crypto_main: fault_injection_mask=%0x\n",
                fault_injection_mask);
    }
#endif

    start_jiffies = jiffies;

    /* Register sysfs entries for Motorola FIPS crypto module status */
    err = class_register(&moto_crypto_class);
    if (err) {
        printk(KERN_ERR "moto_crypto_init(): "
                "failed to register sysfs class, error %d\n", err);
        return err;
    }
    failures = 0;

    /* FSM_TRANS:T2 */
    err = moto_aes_start();
    if (err) {
        failures |= MOTO_CRYPTO_FAILED_ALG_AES;
        goto out;
    }
    err = moto_tdes_start();
    if (err) {
        failures |= MOTO_CRYPTO_FAILED_ALG_TDES;
        goto out;
    }
    err = moto_sha1_start();
    if (err) {
        failures |= MOTO_CRYPTO_FAILED_ALG_SHA1;
        goto out;
    }
    err = moto_sha256_start();
    if (err) {
        failures |= MOTO_CRYPTO_FAILED_ALG_SHA256;
        goto out;
    }
    err = moto_sha512_start();
    if (err) {
        failures |= MOTO_CRYPTO_FAILED_ALG_SHA512;
        goto out;
    }
    err = moto_hmac_start();
    if (err) {
        failures |= MOTO_CRYPTO_FAILED_ALG_HMAC;
        goto out;
    }
    err = moto_prng_init();
    if (err) {
        failures |= MOTO_CRYPTO_FAILED_ALG_RNG;
        goto out;
    }

    switch (self_test_integrity("moto_hmac(moto-sha256)", &__this_module)) {
    case 0:
        printk(KERN_INFO
                "FIPS crypto module integrity check passed\n");
        break;
    case -ENXIO:
        printk(KERN_ERR
                "FIPS crypto module integrity check can only be run "
                "when the module is loaded");
        failures |= MOTO_CRYPTO_FAILED_INTEGRITY;
        goto out;
    case -ENOENT:
        printk(KERN_ERR
                "FIPS crypto module integrity check cannot be made: "
                "Missing HMAC_SHA256 parameter\n");
        /* FALLTHROUGH */
    default:
        printk(KERN_ERR
                "FIPS crypto module self test integrity error\n");
        failures |= MOTO_CRYPTO_FAILED_INTEGRITY;
        goto out;
    }

    printk(KERN_INFO
            "moto_crypto_main: moto_crypto_init successful initialization\n");

    out:
    printk(KERN_INFO
            "failures: %0x\n", failures);
    diff = (long)jiffies - (long)start_jiffies;
    printk(KERN_INFO "moto_crypto_main: Time to init: %ld msec\n", 
            diff * 1000 / HZ);
    if (failures != 0) {
        /* FSM_TRANS:T4 */
        moto_aes_finish();
        moto_tdes_finish();
        moto_hmac_finish();
        moto_sha1_finish();
        moto_sha256_finish();
        moto_sha512_finish();
        moto_prng_finish();
    }
    /* else FSM_TRANS:T3 */

    return 0;
}

/* Module finalization function */
static void __exit moto_crypto_fini(void)
{
    printk(KERN_INFO "moto_crypto_fini\n");
    class_unregister(&moto_crypto_class);
    moto_aes_finish();
    moto_tdes_finish();
    moto_hmac_finish();
    moto_sha1_finish();
    moto_sha256_finish();
    moto_sha512_finish();
    moto_prng_finish();
}


module_init(moto_crypto_init);
module_exit(moto_crypto_fini);

module_param_named(hmac_sha256, moto_integrity_hmac_sha256_expected_value, 
        charp, 0444);
MODULE_PARM_DESC(hmac_sha256, "Module HMAC SHA-256 to be checked");

#ifdef CONFIG_CRYPTO_MOTOROLA_FAULT_INJECTION
unsigned fault_injection_mask;
char *fault_injection_mask_string;
module_param_named(fault, fault_injection_mask_string, charp, 0644);
#endif

MODULE_DESCRIPTION("Motorola cryptographic module");
MODULE_LICENSE("GPL");
MODULE_ALIAS("moto_crypto");
