'''
Created on Nov 23, 2012

@author: wwr004
'''
import os
from ParseUtils import hex2c
from ParseUtils import getParams
from ParseUtils import getBuffer
from ParseUtils import writeTag

def parseHMAC(reqdir):
    reqdata = {}
    for r,d,f in os.walk(reqdir):
        for arq in f:
            if arq.startswith('HMAC') & arq.endswith(".req"):
                reqdata[arq] = processHMACFile(os.path.join(r,arq))
    return reqdata

def processHMACFile(arq):
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
                        continue
                    if parts[0].lower() == 'count':
                        count = int(parts[2])
                        if count == 0:
                            reqdata[section] = {}
                        reqdata[section][count] = {}
                    else:
                        reqdata[section][count][parts[0]] = parts[2]
    return reqdata                        

def writeHMACTests(out, reqdata):
    for filename, filedata in reqdata.items():
        count = {}
        for size, toprocess in filedata.items(): 
            out.write('static struct moto_test_hash_testvec moto_hmac')
            out.write(size.split('=')[1])
            out.write('[] = {\n')
            count[size] = len(toprocess)
            hmacWrite(out, filename, toprocess)
            out.write('\n};\n')
    return count

def hmacWrite(f, filename, filedata):
    ident = ' ' * 4
    doubleident = ident * 2
    first = True
    for count, value in filedata.items():
        if not first:
            f.write(',\n')
        key = value['Key']
        msg = value['Msg']
        f.write(ident)
        f.write('{\n')
        f.write(doubleident)
        f.write('.test_file_name = "')
        f.write(filename)
        f.write('",\n')
        f.write(doubleident)
        f.write('.count = ')
        f.write(str(count))
        f.write(',\n')
        f.write(doubleident)
        f.write('.key = "')
        f.write(hex2c(key))
        f.write('",\n')
        f.write(doubleident)
        f.write('.ksize = ')
        f.write(str(len(key)/2))
        f.write(',\n')
        f.write(doubleident)
        f.write('.plaintext = "')
        f.write(hex2c(msg))
        f.write('",\n')
        f.write(doubleident)
        f.write('.psize = ')
        f.write(str(len(msg)/2))
        f.write(',\n')
        f.write(ident)
        f.write('}')
        first = False
        
def parseHMACKernelLogLine(line, hmacLogData):
    paramsDict = getParams(line)
    if (not paramsDict.has_key('file')) or (not paramsDict.has_key('count')) or (not paramsDict.has_key('hash_size')):
        return
    (start, end) = getBuffer(line)
    filename = paramsDict['file']
    count = paramsDict['count']
    hash_size = paramsDict['hash_size']
    hexbuf = line[start:end]
    sizeDict = hmacLogData.get(filename)
    if sizeDict == None:
        sizeDict = {}
        hmacLogData[filename] = sizeDict
    countDict = sizeDict.get(hash_size)
    if countDict == None:
        countDict = {}
        sizeDict[hash_size] = countDict 
    countDict[int(count)] =''.join(hexbuf.split())

def writeHMACResp(hmacTestData, hmacLogData):
    for filename, testData in hmacTestData.items():
        filenameParts = filename.split('.')
        respFile = open(filenameParts[0] + '.rsp', 'w')
        respData = hmacLogData.get(filename)
        if respData is None:
            print 'Result HMAC data not found for file', filename
            continue
        for size in sorted(testData.iterkeys()):
            data = testData[size]
            resps = respData.get(size.split('=')[1])
            writeResults(respFile, resps, data, size)
        respFile.close()

def writeResults(respFile, resps, tests, size):
    first = True
    for count, params in tests.items():
        hexbuf = resps.get(count)
        if hexbuf is None:
            print 'No response found for count', count, 'and size', size
            continue
        key = params.get('Key')
        msg = params.get('Msg')
        klen = params.get('Klen')
        tlen = params.get('Tlen')
        if key is None or msg is None or klen is None or tlen is None:
            print 'Missing required parameter for count', count, 'and size', size, 'key', key, 'klen', klen, 'tlen', tlen
            continue
        if first:
            respFile.write('[')
            respFile.write(size)
            respFile.write(']\n\n')
            first = False
        writeTag(respFile, 'Count', str(count))
        writeTag(respFile, 'Klen', klen)
        writeTag(respFile, 'Tlen', tlen)
        writeTag(respFile, 'Key', key)
        writeTag(respFile, 'Msg', msg)
        writeTag(respFile, 'Mac', hexbuf)
        respFile.write('\n')
