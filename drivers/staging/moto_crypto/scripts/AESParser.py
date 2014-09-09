'''
Created on Oct 25, 2012

@author: wwr004
'''

import os
from ParseUtils import hex2c
from ParseUtils import getParams
from ParseUtils import writeTag
from ParseUtils import getBuffer

# Parses the AES tests req files and outputs the test vectors
# out: file descriptor to write to
# mode: AES mode (ECB or CBC)
# reqdir: directory where to find the req files
# prefix: prefix to be used on the test array name
def parseAES(out, mode, reqdir):
    reqdata = {}
    monte = {}
    for r,d,f in os.walk(reqdir):
        for arq in f:
            if arq.startswith(mode) & arq.endswith(".req"):
                if 'MCT' in arq:
                    monte[arq] = processAESFile(os.path.join(r,arq))
                else:    
                    reqdata[arq] = processAESFile(os.path.join(r,arq))
    return (reqdata, monte)
        
# Parses one AES req file
# arq: file name
# Returns a data structure with the test data. This is a dictionary which maps a string (ENCRYPT or DECRYPT)
# to another dictionary, which maps an integer (test count in req file) to an inner dictionary which represents
# one test case and maps the test input variables (IV, KEY, etc) to a string with their values.           
def processAESFile(arq):            
    reqdata = {}
    section = None
    for line in open(arq, 'r').readlines():
        line = line.strip()
        if len(line) > 0:
            first = line[:1]
            if first != '#':
                if first =='[':
                    close = line.rindex(']')
                    section = line[1:close]
                    count = -1
                else:
                    parts = line.split()
                    if parts[1] != '=':
                        print "Error format line=", line
                    if parts[0].lower() == 'count':
                        count = int(parts[2])
                        if count == 0:
                            reqdata[section] = {}
                        reqdata[section][count] = {}
                    else:
                        reqdata[section][count][parts[0]] = parts[2]
    return reqdata                        

def writeAESTests(prefix, out, reqdata, montedata):
    out.write('static struct moto_test_cipher_testvec ')
    out.write(prefix)
    out.write('_monte_enc[] = {\n')
    for filename, filedata in montedata.items():
        toprocess = filedata['ENCRYPT'].items()       
        aeswrite(out, filename, toprocess, True)
        out.write(',\n')    
    out.write('\n};\n')    
    out.write('static struct moto_test_cipher_testvec ')
    out.write(prefix)
    out.write('_monte_dec[] = {\n')
    for filename, filedata in montedata.items():
        toprocess = filedata['DECRYPT'].items()       
        aeswrite(out, filename, toprocess, False)
        out.write(',\n')    
    out.write('\n};\n')    
    enccount = 0
    for filename, filedata in reqdata.items(): 
        toprocess = filedata['ENCRYPT'].items()       
        if enccount == 0:
            out.write('static struct moto_test_cipher_testvec ')
            out.write(prefix)
            out.write('_enc[] = {\n')
        enccount += len(toprocess)
        aeswrite(out, filename, toprocess, True)
        out.write(',\n')    
    if enccount > 0:
        out.write('\n};\n')    
    deccount = 0
    for filename, filedata in reqdata.items():        
        toprocess = filedata['DECRYPT'].items()       
        if deccount == 0:
            out.write('static struct moto_test_cipher_testvec ')
            out.write(prefix)
            out.write('_dec[] = {\n')
        deccount += len(toprocess)
        aeswrite(out, filename, toprocess, False)
        out.write(',\n')    
    if deccount > 0:                    
        out.write('\n};\n') 
    return (enccount, deccount)

# Write the parameters of a specific AES test
# f: file descriptor to write to
# tname: req file name 
# testvector: a dict which maps an integer (test count) to another dict which maps the test variables 
# (iv, key, etc) to its values  
# encrypt: a boolean which indicates if encryption tests are being processed       
def aeswrite(f, tname, testvector, encrypt):
    ident = ' ' * 4                    
    doubleident = ident * 2
    first = True                    
    for count, value in testvector:
        if not first:
            f.write(',\n')
        if encrypt:
            inputstr = value['PLAINTEXT']
        else:
            inputstr = value['CIPHERTEXT']
        if (value.has_key('IV')):     
            ivstr = value['IV']
        else:
            ivstr = None
        keystr = value['KEY']
        f.write(ident)
        f.write('{\n')
        f.write(doubleident)
        f.write('.test_file_name = "')
        f.write(tname)
        f.write('",\n')
        f.write(doubleident)
        f.write('.count = ')
        f.write(str(count))
        f.write(',\n')
        f.write(doubleident)
        f.write('.key = "')
        f.write(hex2c(keystr))
        f.write('",\n')
        f.write(doubleident)
        f.write('.klen = ')
        f.write(str(len(keystr)/2))
        f.write(',\n')
        f.write(doubleident)
        if ivstr != None:
            f.write('.iv = "')
            f.write(hex2c(ivstr))
            f.write('",\n')
            f.write(doubleident)
        f.write('.input = "')
        f.write(hex2c(inputstr))
        f.write('",\n')
        f.write(doubleident)
        f.write('.ilen = ')
        f.write(str(len(inputstr)/2))
        f.write(',\n')
        f.write(ident)
        f.write('}')
        first = False

def parseAESKernelLogLine(line, logData):
    paramsDict = getParams(line)
    if (not paramsDict.has_key('file')) or (not paramsDict.has_key('count')) or (not paramsDict.has_key('enc')):
        return
    (start, end) = getBuffer(line)
    if start == -1 or end == -1:
        print 'Possible interrupted buffer.Line=', line
        return
    filename = paramsDict['file']
    count = paramsDict['count']
    enc = paramsDict['enc']
    hexbuf = line[start:end]
    encDecDict = logData.get(filename)
    if encDecDict == None:
        encDecDict = {}
        logData[filename] = encDecDict
    countDict = encDecDict.get(enc)
    if countDict == None:
        countDict = {}
        encDecDict[enc] = countDict
    countDict[int(count)] =''.join(hexbuf.split())

def parseAESMonteKernelLogLine(line, logData):
    paramsDict = getParams(line)
    if (not paramsDict.has_key('file')) or (not paramsDict.has_key('count')) or (not paramsDict.has_key('enc')):
        return
    filename = paramsDict['file']
    count = paramsDict['count']
    enc = paramsDict['enc']
    (start, end) = getBuffer(line)
    ct = line[start:end]
    (start, end) = getBuffer(line, end + 1)
    key = line[start:end]
    (start, end) = getBuffer(line, end + 1)
    pt = line[start:end]
    (start, end) = getBuffer(line, end + 1)
    iv = None
    if end != -1:
        iv = line[start:end]
    encDecDict = logData.get(filename)
    if encDecDict == None:
        encDecDict = {}
        logData[filename] = encDecDict
    countDict = encDecDict.get(enc)
    if countDict == None:
        countDict = {}
        encDecDict[enc] = countDict
    countDict[int(count)] = {}
    countDict[int(count)]['ciphertext'] =''.join(ct.split())
    countDict[int(count)]['key'] =''.join(key.split())
    countDict[int(count)]['plaintext'] =''.join(pt.split())
    if iv != None:
        countDict[int(count)]['iv'] =''.join(iv.split())

def writeAESResp(data, logData):
    for filename, testData in data.items():
        encDec = logData.get(filename)
        if encDec == None:
            print 'Log data not found for file', filename
            continue
        tests = testData.get('ENCRYPT')
        if tests == None:
            print 'Encryption test data not found for file', filename
            continue
        resps = encDec.get('1')
        if resps == None:
            print 'No log data for encryption tests found for file', filename
            continue
        filenameParts = filename.split('.')
        respFile = open(filenameParts[0] + '.rsp', 'w')
        writeResults(respFile, filename, resps, tests, True)
        tests = testData.get('DECRYPT')
        if tests == None:
            print 'Decryption test data not found for file', filename
            continue
        resps = encDec.get('0')
        if resps == None:
            print 'No log data for decryption tests found for file', filename
            continue
        writeResults(respFile, filename, resps, tests, False)
        respFile.close()

def writeAESMonteResp(data, logData):
    for filename, testData in data.items():
        encDec = logData.get(filename)
        if encDec == None:
            print 'Log data not found for file', filename
            continue
        tests = testData.get('ENCRYPT')
        if tests == None:
            print 'Encryption test data not found for file', filename
            continue
        resps = encDec.get('1')
        if resps == None:
            print 'No log data for encryption tests found for file', filename
            continue
        filenameParts = filename.split('.')
        respFile = open(filenameParts[0] + '.rsp', 'w')
        writeMonteResults(respFile, filename, resps, True)
        tests = testData.get('DECRYPT')
        if tests == None:
            print 'Decryption test data not found for file', filename
            continue
        resps = encDec.get('0')
        if resps == None:
            print 'No log data for decryption tests found for file', filename
            continue
        writeMonteResults(respFile, filename, resps, False)
        respFile.close()

def writeMonteResults(respFile, filename, resps, enc):
    first = True
    for count, params in resps.items():
        key = params.get('key')
        iv = params.get('iv')
        algInput = params.get('plaintext')
        algOutput = params.get('ciphertext')
        if (key == None) or (algInput == None):
            print 'Missing required parameter for count', count, 'and file', filename, 'key', key, 'iv', iv, 'input',algInput, 'enc', enc
            continue
        if first:
            if enc:
                respFile.write('[ENCRYPT]\n\n')
            else:
                respFile.write('[DECRYPT]\n\n')
            first = False
        writeTag(respFile, 'COUNT', str(count))
        writeTag(respFile, 'KEY', key)
        if iv != None:
            writeTag(respFile, 'IV', iv)
        if enc:
            writeTag(respFile, 'PLAINTEXT', algInput)
            writeTag(respFile, 'CIPHERTEXT', algOutput)
        else:
            writeTag(respFile, 'CIPHERTEXT', algInput)
            writeTag(respFile, 'PLAINTEXT', algOutput)
        respFile.write('\n')
 
def writeResults(respFile, filename, resps, tests, enc):
    first = True
    for count, params in tests.items():
        hexbuf = resps.get(count)
        if hexbuf == None:
            print 'No response found for count', count, 'and file', filename
            continue
        key = params.get('KEY')
        iv = params.get('IV')
        if enc:
            algInput = params.get('PLAINTEXT')
        else:
            algInput = params.get('CIPHERTEXT')
        if (key == None) or (algInput == None):
            print 'Missing required parameter for count', count, 'and file', filename, 'key', key, 'iv', iv, 'input',algInput, 'enc', enc
            continue
        if first:
            if enc:
                respFile.write('[ENCRYPT]\n\n')
            else:
                respFile.write('[DECRYPT]\n\n')
            first = False
        writeTag(respFile, 'COUNT', str(count))
        writeTag(respFile, 'KEY', key)
        if iv != None:
            writeTag(respFile, 'IV', iv)
        if enc:
            writeTag(respFile, 'PLAINTEXT', algInput)
            writeTag(respFile, 'CIPHERTEXT', hexbuf)
        else:
            writeTag(respFile, 'CIPHERTEXT', algInput)
            writeTag(respFile, 'PLAINTEXT', hexbuf)
        respFile.write('\n')

