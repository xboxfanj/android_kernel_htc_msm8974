#!/usr/bin/env python

'''
Created on Sep 20, 2012

@author: wwr004
'''

import argparse
from AESParser import parseAES
from AESParser import writeAESTests
from AESParser import parseAESKernelLogLine
from AESParser import parseAESMonteKernelLogLine
from AESParser import writeAESResp
from AESParser import writeAESMonteResp
from SHAParser import parseSHA
from SHAParser import writeShaTests
from SHAParser import writeShaMonteTests
from SHAParser import parseSHAKernelLogLine
from SHAParser import writeSHAResp
from SHAParser import parseSHAMonteKernelLogLine
from SHAParser import writeSHAMonteResp
from TDESParser import parseTDES
from TDESParser import writeTDESTests
from TDESParser import writeTdesMonteTests
from TDESParser import parseTDESKernelLogLine
from TDESParser import writeTDESResp
from TDESParser import writeTDESMonteResp
from TDESParser import parseTDESMonteKernelLogLine
from RNGParser import parseRNG
from RNGParser import writeRNGTests
from RNGParser import parseRNGKernelLogLine
from RNGParser import writeRNGResp
from HMACParser import parseHMAC
from HMACParser import writeHMACTests
from HMACParser import parseHMACKernelLogLine
from HMACParser import writeHMACResp
import cPickle

def main():
    """
    launched when running this file
    """
    parser = argparse.ArgumentParser(description='req and resp files processor')
    parser.add_argument('-g','--generate', dest='reqdir', help='generates C code for algorithm testing')
    parser.add_argument('-p','--parse', dest='klogfile', help='parser kernel log and generates response file for algorithm testing')
    parser.add_argument('-a', required=True, dest='algs', type=str, nargs='+', help='algorithm names to be parsed/generated')
    
    args = parser.parse_args()
    args.algs = map(str.lower, args.algs)
    for alg in args.algs:
        if not alg in ['aes','sha','tdes','rng','hmac']:
            print 'Unknown algorithm', alg
            exit()
    if args.reqdir != None:
        generateTestCode(args.reqdir, args.algs)
    elif args.klogfile != None:
        generateRespFile(args.klogfile, args.algs)
    else:
        print parser.error('One of -g or -p must be used')
        
def generateTestCode(reqdir, algs):
    cbcCount = ecbCount = (0,0)
    if 'aes' in algs:
        aesFile = open('alg_test_aes.c', 'w')
        (ecbData, ecbMonteData) = parseAES(aesFile, 'ECB', reqdir)
        ecbPickleFile = open('ecbdata.bin', 'w')
        cPickle.dump(ecbData, ecbPickleFile, cPickle.HIGHEST_PROTOCOL)
        ecbPickleFile.close()
        ecbPickleFile = open('ecbmontedata.bin', 'w')
        cPickle.dump(ecbMonteData, ecbPickleFile, cPickle.HIGHEST_PROTOCOL)
        ecbPickleFile.close()
        ecbCount = writeAESTests('moto_aes_ecb', aesFile, ecbData, ecbMonteData)
        (cbcData, cbcMonteData) = parseAES(aesFile, 'CBC', reqdir)
        cbcPickleFile = open('cbcdata.bin', 'w')
        cPickle.dump(cbcData, cbcPickleFile, cPickle.HIGHEST_PROTOCOL)
        cbcPickleFile.close()
        cbcPickleFile = open('cbcmontedata.bin', 'w')
        cPickle.dump(cbcMonteData, cbcPickleFile, cPickle.HIGHEST_PROTOCOL)
        cbcPickleFile.close()
        cbcCount = writeAESTests('moto_aes_cbc', aesFile, cbcData, cbcMonteData)
        aesFile.close()
    shaCount = None
    if 'sha' in algs:
        shaCount = {}
        for alg in ['SHA1', 'SHA224', 'SHA256', 'SHA384', 'SHA512']:
            (shaData, shaMonteData) = parseSHA(alg, reqdir)
            shaPickleFile = open('shadata_' + alg + '.bin', 'w')
            cPickle.dump(shaData, shaPickleFile, cPickle.HIGHEST_PROTOCOL)
            shaPickleFile.close()
            shaMontePickleFile = open('shamontedata_' + alg + '.bin', 'w')
            cPickle.dump(shaMonteData, shaMontePickleFile, cPickle.HIGHEST_PROTOCOL)
            shaMontePickleFile.close()
            fileName = 'alg_test_' + alg.lower() + ".c"
            shaFile = open(fileName, 'w')
            writeShaMonteTests(alg, shaFile, shaMonteData)
            shaCount[alg] = writeShaTests(alg, shaFile, shaData)
            shaFile.close()
    tdesEcbCount = tdesCbcCount = (0, 0)
    if 'tdes' in algs:
        tdesFile = open('alg_test_tdes.c', 'w')
        (tdesEcbData, tdesEcbMonteData) = parseTDES(tdesFile, 'TECB', reqdir)
        (tdesCbcData, tdesCbcMonteData) = parseTDES(tdesFile, 'TCBC', reqdir)
        ecbPickleFile = open('tdesecbdata.bin', 'w')
        cPickle.dump(tdesEcbData, ecbPickleFile, cPickle.HIGHEST_PROTOCOL)
        ecbPickleFile.close()
        ecbMontePickleFile = open('tdesecbmontedata.bin', 'w')
        cPickle.dump(tdesEcbMonteData, ecbMontePickleFile, cPickle.HIGHEST_PROTOCOL)
        ecbMontePickleFile.close()
        cbcPickleFile = open('tdescbcdata.bin', 'w')
        cPickle.dump(tdesCbcData, cbcPickleFile, cPickle.HIGHEST_PROTOCOL)
        cbcPickleFile.close()
        cbcMontePickleFile = open('tdescbcmontedata.bin', 'w')
        cPickle.dump(tdesCbcMonteData, cbcMontePickleFile, cPickle.HIGHEST_PROTOCOL)
        cbcMontePickleFile.close()
        writeTdesMonteTests(tdesFile, tdesEcbMonteData)
        writeTdesMonteTests(tdesFile, tdesCbcMonteData)
        tdesEcbCount = writeTDESTests('moto_tdes_ecb', tdesFile, tdesEcbData)
        tdesCbcCount = writeTDESTests('moto_tdes_cbc', tdesFile, tdesCbcData)
        tdesFile.close()
    rngCount = 0
    if 'rng' in algs:
        rngData = parseRNG(reqdir)
        rngPickleFile = open('rngdata.bin', 'w')
        cPickle.dump(rngData, rngPickleFile, cPickle.HIGHEST_PROTOCOL)
        rngPickleFile.close()
        rngFile = open('alg_test_rng.c', 'w')
        rngCount = writeRNGTests(rngFile, rngData)
    hmacCount = None
    if 'hmac' in algs:
        hmacData = parseHMAC(reqdir)
        hmacPickleFile = open('hmacdata.bin', 'w')
        cPickle.dump(hmacData, hmacPickleFile, cPickle.HIGHEST_PROTOCOL)
        hmacPickleFile.close()
        hmacFile = open('alg_test_hmac.c', 'w')
        hmacCount = writeHMACTests(hmacFile, hmacData)
         
    generalFile = open('alg_test.c', 'w')
    writeTestDesc(generalFile, ecbCount, cbcCount, shaCount, tdesEcbCount, tdesCbcCount, rngCount, hmacCount)
    generalFile.close()

def generateRespFile(klogfile, algs):
    aesLogData = {}
    aesMonteLogData = {}
    shaLogData = {}
    shaMonteLogData = {}
    tdesLogData = {}
    tdesMonteLogData = {}
    rngLogData = {}
    hmacLogData = {}
    aes = 'aes' in algs
    sha = 'sha' in algs
    tdes = 'tdes' in algs
    rng = 'rng' in algs
    hmac = 'hmac' in algs
    for line in open(klogfile, 'r').readlines():
        pos = line.index(' ')
        first = line[:pos]
        if first == 'Monte':
            if sha:
                parseSHAMonteKernelLogLine(line, shaMonteLogData)
            else:
                continue
        elif first == 'TDESMonte':
            if tdes:
                parseTDESMonteKernelLogLine(line, tdesMonteLogData)
            else:
                continue
        elif first == 'AESMonte':
            if aes:
                parseAESMonteKernelLogLine(line, aesMonteLogData)
            else:
                continue
        else:
            parts = first.split(':')
            if parts[0] != 'file':
                print 'Error in line', line
                continue
            else:
                if parts[1].startswith('ECB') or parts[1].startswith('CBC'):
                    if aes:
                        parseAESKernelLogLine(line, aesLogData)
                    else:
                        continue
                elif parts[1].startswith('SHA'):
                    if sha:
                        parseSHAKernelLogLine(line, shaLogData)
                    else:
                        continue
                elif parts[1].startswith('TECB') or parts[1].startswith('TCBC'):
                    if tdes:
                        parseTDESKernelLogLine(line, tdesLogData)
                    else:
                        continue
                elif parts[1].startswith('ANSI931'):
                    if rng:
                        parseRNGKernelLogLine(line, rngLogData)
                elif parts[1].startswith('HMAC'):
                    if hmac:
                        parseHMACKernelLogLine(line, hmacLogData)
                else:
                    print "Error in line", line
                    continue
    if aes:
        ecbPickleFile = open('ecbdata.bin', 'r')
        ecbData = cPickle.load(ecbPickleFile)
        ecbPickleFile.close()
        cbcPickleFile = open('cbcdata.bin', 'r')
        cbcData = cPickle.load(cbcPickleFile)
        cbcPickleFile.close()
        ecbMontePickleFile = open('ecbmontedata.bin', 'r')
        ecbMonteData = cPickle.load(ecbMontePickleFile)
        ecbMontePickleFile.close()
        cbcMontePickleFile = open('cbcmontedata.bin', 'r')
        cbcMonteData = cPickle.load(cbcMontePickleFile)
        cbcMontePickleFile.close()
        writeAESResp(ecbData, aesLogData)
        writeAESMonteResp(ecbMonteData, aesMonteLogData)
        writeAESResp(cbcData, aesLogData)
        writeAESMonteResp(cbcMonteData, aesMonteLogData)

    if sha:
        for (alg, size) in [('SHA1', '20'), ('SHA224', '28'), ('SHA256', '32'), ('SHA384', '48'), ('SHA512', '64')]:
            shaPickleFile = open('shadata_' + alg + '.bin', 'r')
            shaData = cPickle.load(shaPickleFile)
            shaPickleFile.close()
            shaMontePickleFile = open('shamontedata_' + alg + '.bin', 'r')
            shaMonteData = cPickle.load(shaMontePickleFile)
            shaMontePickleFile.close()
            writeSHAResp(size, shaData, shaLogData)
            writeSHAMonteResp(alg, size, shaMonteData, shaMonteLogData)

    if tdes:
        ecbPickleFile = open('tdesecbdata.bin', 'r')
        tdesEcbData = cPickle.load(ecbPickleFile)
        ecbPickleFile.close()
        ecbMontePickleFile = open('tdesecbmontedata.bin', 'r')
        tdesEcbMonteData = cPickle.load(ecbMontePickleFile)
        ecbMontePickleFile.close()
        cbcPickleFile = open('tdescbcdata.bin', 'r')
        tdesCbcData = cPickle.load(cbcPickleFile)
        cbcPickleFile.close()
        cbcMontePickleFile = open('tdescbcmontedata.bin', 'r')
        tdesCbcMonteData = cPickle.load(cbcMontePickleFile)
        cbcMontePickleFile.close()
        writeTDESResp(tdesEcbData, tdesLogData)
        writeTDESResp(tdesCbcData, tdesLogData)
        writeTDESMonteResp(tdesEcbMonteData, tdesMonteLogData)
        writeTDESMonteResp(tdesCbcMonteData, tdesMonteLogData)

    if rng:
        rngPickleFile = open('rngdata.bin', 'r')
        rngTestData = cPickle.load(rngPickleFile)
        rngPickleFile.close()
        writeRNGResp(rngTestData, rngLogData)
    
    if hmac:
        hmacPickleFile = open('hmacdata.bin', 'r')
        hmacTestData = cPickle.load(hmacPickleFile)
        hmacPickleFile.close()
        writeHMACResp(hmacTestData, hmacLogData)
        
# Writes the test description vector to a file
# out: file descriptor
# ecbCount: array with the number of AES ECB encryption (position 0) and decryption tests (position 1)
# cbcCount: array with the number of AES CBC encryption (position 0) and decryption tests (position 1)
# shaCount: number of SHA1 tests
def writeTestDesc(out, ecbCount, cbcCount, shaCount, tdesEcbCount, tdesCbcCount, rngCount, hmacCount):
    out.write('''
/* Please keep this list sorted by algorithm name. */
static const struct moto_alg_test_desc moto_alg_test_descs[] = {
''')
    if rngCount > 0:
        out.write('''
#ifdef TEST_RNG
''')
        out.write('''
    {
        .alg = "ansi_cprng",
        .test = moto_alg_test_cprng,
        .suite = {
            .cprng = {
                .vecs = moto_rng,
                .count = ''')
        out.write(str(rngCount))
        out.write('''
            }
        }
    },
#endif
''')
    if cbcCount[0] > 0:
        out.write('''
#ifdef TEST_AES
    {
        .alg = "cbc(aes)",
        .test = moto_alg_test_skcipher,
        .suite = {
            .cipher = {
                .enc = {
                    .vecs = moto_aes_cbc_enc,
                    .count = ''')
        out.write(str(cbcCount[0]))
        out.write('''
                },
                .dec = {
                    .vecs = moto_aes_cbc_dec,
                    .count = ''')
        out.write(str(cbcCount[1]))
        out.write('''
                }
            }
        }
    },
#endif
''')
    if tdesCbcCount[0] > 0:
        out.write('''
#ifdef TEST_TDES
    {
        .alg = "cbc(des3_ede)",
        .test = moto_alg_test_skcipher,
        .suite = {
            .cipher = {
                .enc = {
                    .vecs = moto_tdes_cbc_enc,
                    .count = ''')
        out.write(str(tdesCbcCount[0]))
        out.write('''
                },
                .dec = {
                    .vecs = moto_tdes_cbc_dec,
                    .count = ''')
        out.write(str(tdesCbcCount[1]))
        out.write('''
                }
            }
        }
    },
#endif
''')
    if ecbCount[0] > 0:
        out.write('''
#ifdef TEST_AES
    {
        .alg = "ecb(aes)",
        .test = moto_alg_test_skcipher,
        .suite = {
            .cipher = {
                .enc = {
                    .vecs = moto_aes_ecb_enc,
                    .count = ''')
        out.write(str(ecbCount[0]))
        out.write('''
                },
                .dec = {
                    .vecs = moto_aes_ecb_dec,
                    .count = ''')
        out.write(str(ecbCount[1]))
        out.write('''
                }
            }
        }
    },
#endif
''')
    if tdesEcbCount[0] > 0:
        out.write('''
#ifdef TEST_TDES
    {
        .alg = "ecb(des3_ede)",
        .test = moto_alg_test_skcipher,
        .suite = {
            .cipher = {
                .enc = {
                    .vecs = moto_tdes_ecb_enc,
                    .count = ''')
        out.write(str(tdesEcbCount[0]))
        out.write('''
                },
                .dec = {
                    .vecs = moto_tdes_ecb_dec,
                    .count = ''')
        out.write(str(tdesEcbCount[1]))
        out.write('''
                }
            }
        }
    },
#endif
''')
    if not hmacCount is None:
            out.write('''
#ifdef TEST_HMAC
''')
            if hmacCount['L=20'] > 0:
                out.write('''
    {
        .alg = "moto_hmac(moto-sha1)",
        .test = moto_alg_test_hash,
        .suite = {
            .hash = {
                .vecs = moto_hmac20,
                .count = ''')
                out.write(str(hmacCount['L=20']))
                out.write('''
            }
        }
    },''') 
            if hmacCount['L=28'] > 0:
                out.write('''
    {
        .alg = "moto_hmac(moto-sha224)",
        .test = moto_alg_test_hash,
        .suite = {
            .hash = {
                .vecs = moto_hmac28,
                .count = ''')
                out.write(str(hmacCount['L=28']))
                out.write('''
            }
        }
    },''') 
            if hmacCount['L=32'] > 0:
                out.write('''
    {
        .alg = "moto_hmac(moto-sha256)",
        .test = moto_alg_test_hash,
        .suite = {
            .hash = {
                .vecs = moto_hmac32,
                .count = ''')
                out.write(str(hmacCount['L=32']))
                out.write('''
            }
        }
    },''') 
            if hmacCount['L=48'] > 0:
                out.write('''
    {
        .alg = "moto_hmac(moto-sha384)",
        .test = moto_alg_test_hash,
        .suite = {
            .hash = {
                .vecs = moto_hmac48,
                .count = ''')
                out.write(str(hmacCount['L=48']))
                out.write('''
            }
        }
    },''') 
            if hmacCount['L=64'] > 0:
                out.write('''
    {
        .alg = "moto_hmac(moto-sha512)",
        .test = moto_alg_test_hash,
        .suite = {
            .hash = {
                .vecs = moto_hmac64,
                .count = ''')
                out.write(str(hmacCount['L=64']))
                out.write('''
            }
        }
    },
#endif 
''')
    if not shaCount is None:
        if shaCount['SHA1'] > 0:
            out.write('''
#ifdef TEST_SHA1
    {
        .alg = "sha1",
        .test = moto_alg_test_hash,
        .suite = {
            .hash = {
                .vecs = moto_sha1,
                .count = ''')
            out.write(str(shaCount['SHA1']))
            out.write('''
            }
        }
    },
#endif
    ''')
        if shaCount['SHA224'] > 0:
            out.write('''
#ifdef TEST_SHA224
    {
        .alg = "sha224",
        .test = moto_alg_test_hash,
        .suite = {
            .hash = {
                .vecs = moto_sha224,
                .count = ''')
            out.write(str(shaCount['SHA224']))
            out.write('''
            }
        }
    },
#endif
    ''')
        if shaCount['SHA256'] > 0:
            out.write('''
#ifdef TEST_SHA256
    {
        .alg = "sha256",
        .test = moto_alg_test_hash,
        .suite = {
            .hash = {
                .vecs = moto_sha256,
                .count = ''')
            out.write(str(shaCount['SHA256']))
            out.write('''
            }
        }
    },
#endif
    ''')
        if shaCount['SHA384'] > 0:
            out.write('''
#ifdef TEST_SHA384
    {
        .alg = "sha384",
        .test = moto_alg_test_hash,
        .suite = {
            .hash = {
                .vecs = moto_sha384,
                .count = ''')
            out.write(str(shaCount['SHA384']))
            out.write('''
            }
        }
    },
#endif
    ''')
        if shaCount['SHA512'] > 0:
            out.write('''
#ifdef TEST_SHA512
    {
        .alg = "sha512",
        .test = moto_alg_test_hash,
        .suite = {
            .hash = {
                .vecs = moto_sha512,
                .count = ''')
            out.write(str(shaCount['SHA512']))
            out.write('''
            }
        }
    },
#endif
    ''')
    out.write('''
};

''')   

if __name__ == "__main__":
    main()
