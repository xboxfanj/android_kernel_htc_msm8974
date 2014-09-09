/* 
 * DES & Triple DES EDE Cipher Algorithms.
 */

#ifndef __MOTO_CRYPTO_TDES_H
#define __MOTO_CRYPTO_TDES_H

#define DES_KEY_SIZE        8
#define DES_EXPKEY_WORDS    32

#define DES3_EDE_KEY_SIZE       (3 * DES_KEY_SIZE)
#define DES3_EDE_EXPKEY_WORDS   (3 * DES_EXPKEY_WORDS)
#define DES3_EDE_BLOCK_SIZE     8
#define DES3_EDE_IV_SIZE        8

int moto_tdes_start(void);
void moto_tdes_finish(void);

#endif /* __MOTO_CRYPTO_TDES_H */
