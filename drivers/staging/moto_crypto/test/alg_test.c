
/* Please keep this list sorted by algorithm name. */
static const struct moto_alg_test_desc moto_alg_test_descs[] = {

#ifdef TEST_RNG

    {
        .alg = "ansi_cprng",
        .test = moto_alg_test_cprng,
        .suite = {
            .cprng = {
                .vecs = moto_rng,
                .count = 129
            }
        }
    },
#endif

#ifdef TEST_AES
    {
        .alg = "cbc(aes)",
        .test = moto_alg_test_skcipher,
        .suite = {
            .cipher = {
                .enc = {
                    .vecs = moto_aes_cbc_enc,
                    .count = 1069
                },
                .dec = {
                    .vecs = moto_aes_cbc_dec,
                    .count = 1069
                }
            }
        }
    },
#endif

#ifdef TEST_TDES
    {
        .alg = "cbc(des3_ede)",
        .test = moto_alg_test_skcipher,
        .suite = {
            .cipher = {
                .enc = {
                    .vecs = moto_tdes_cbc_enc,
                    .count = 265
                },
                .dec = {
                    .vecs = moto_tdes_cbc_dec,
                    .count = 265
                }
            }
        }
    },
#endif

#ifdef TEST_AES
    {
        .alg = "ecb(aes)",
        .test = moto_alg_test_skcipher,
        .suite = {
            .cipher = {
                .enc = {
                    .vecs = moto_aes_ecb_enc,
                    .count = 1069
                },
                .dec = {
                    .vecs = moto_aes_ecb_dec,
                    .count = 1069
                }
            }
        }
    },
#endif

#ifdef TEST_TDES
    {
        .alg = "ecb(des3_ede)",
        .test = moto_alg_test_skcipher,
        .suite = {
            .cipher = {
                .enc = {
                    .vecs = moto_tdes_ecb_enc,
                    .count = 265
                },
                .dec = {
                    .vecs = moto_tdes_ecb_dec,
                    .count = 265
                }
            }
        }
    },
#endif

#ifdef TEST_HMAC

    {
        .alg = "moto_hmac(moto-sha1)",
        .test = moto_alg_test_hash,
        .suite = {
            .hash = {
                .vecs = moto_hmac20,
                .count = 15
            }
        }
    },
    {
        .alg = "moto_hmac(moto-sha224)",
        .test = moto_alg_test_hash,
        .suite = {
            .hash = {
                .vecs = moto_hmac28,
                .count = 15
            }
        }
    },
    {
        .alg = "moto_hmac(moto-sha256)",
        .test = moto_alg_test_hash,
        .suite = {
            .hash = {
                .vecs = moto_hmac32,
                .count = 15
            }
        }
    },
    {
        .alg = "moto_hmac(moto-sha384)",
        .test = moto_alg_test_hash,
        .suite = {
            .hash = {
                .vecs = moto_hmac48,
                .count = 15
            }
        }
    },
    {
        .alg = "moto_hmac(moto-sha512)",
        .test = moto_alg_test_hash,
        .suite = {
            .hash = {
                .vecs = moto_hmac64,
                .count = 15
            }
        }
    },
#endif 

#ifdef TEST_SHA1
    {
        .alg = "sha1",
        .test = moto_alg_test_hash,
        .suite = {
            .hash = {
                .vecs = moto_sha1,
                .count = 129
            }
        }
    },
#endif
    
#ifdef TEST_SHA224
    {
        .alg = "sha224",
        .test = moto_alg_test_hash,
        .suite = {
            .hash = {
                .vecs = moto_sha224,
                .count = 129
            }
        }
    },
#endif
    
#ifdef TEST_SHA256
    {
        .alg = "sha256",
        .test = moto_alg_test_hash,
        .suite = {
            .hash = {
                .vecs = moto_sha256,
                .count = 129
            }
        }
    },
#endif
    
#ifdef TEST_SHA384
    {
        .alg = "sha384",
        .test = moto_alg_test_hash,
        .suite = {
            .hash = {
                .vecs = moto_sha384,
                .count = 257
            }
        }
    },
#endif
    
#ifdef TEST_SHA512
    {
        .alg = "sha512",
        .test = moto_alg_test_hash,
        .suite = {
            .hash = {
                .vecs = moto_sha512,
                .count = 257
            }
        }
    },
#endif
    
};

