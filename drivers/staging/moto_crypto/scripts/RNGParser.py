'''
Created on Nov 23, 2012

@author: wwr004
'''

import os
from ParseUtils import hex2c
from ParseUtils import getParams
from ParseUtils import getBuffer
from ParseUtils import writeTag

def parseRNG(reqdir):
    reqdata = {}
    for r,d,f in os.walk(reqdir):
        for arq in f:
            if arq.startswith('ANSI931') & arq.endswith(".req"):
                reqdata[arq] = processRNGFile(os.path.join(r,arq))
    return reqdata

def processRNGFile(arq):
    reqdata = {}
    for line in open(arq, 'r').readlines():
        line = line.strip()
        if len(line) > 0:
            first = line[:1]
            if first != '#' and first != '[':
                parts = line.split()
                if parts[1] != '=':
                    print "Error format line=", line
                if parts[0].lower() == 'count':
                    count = int(parts[2])
                    reqdata[count] = {}
                else:
                    reqdata[count][parts[0]] = parts[2]
    return reqdata                        

def writeRNGTests(out, rngData):
    out.write('struct moto_test_cprng_testvec moto_rng[] = {\n')
    count = 0
    for filename, filedata in rngData.items():
        count += len(filedata)
        rngWrite(out, filename, filedata)
        out.write(',\n')
    out.write('\n};\n') 
    return count

def rngWrite(f, filename, filedata):
    ident = ' ' * 4
    doubleident = ident * 2
    first = True
    isMonte = 'MCT' in filename
    for count, value in filedata.items():
        if not first:
            f.write(',\n')
        key = value['Key']
        dt = value['DT']
        v = value['V']
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
        f.write('.klen = ')
        f.write(str(len(key)/2))
        f.write(',\n')
        f.write(doubleident)
        f.write('.dt = "')
        f.write(hex2c(dt))
        f.write('",\n')
        f.write(doubleident)
        f.write('.dtlen = ')
        f.write(str(len(dt)/2))
        f.write(',\n')
        f.write(doubleident)
        f.write('.v = "')
        f.write(hex2c(v))
        f.write('",\n')
        f.write(doubleident)
        f.write('.vlen = ')
        f.write(str(len(v)/2))
        f.write(',\n')
        f.write(doubleident)
        f.write('.rlen = 16,\n')
        f.write(doubleident)
        f.write('.loops = ')
        if isMonte:
            f.write('10000')
        else:
            f.write('1')
        f.write('\n')
        f.write(ident)
        f.write('}')
        first = False

def parseRNGKernelLogLine(line, rngLogData):
    paramsDict = getParams(line)
    if (not paramsDict.has_key('file')) or (not paramsDict.has_key('count')):
        return
    (start, end) = getBuffer(line)
    filename = paramsDict['file']
    count = paramsDict['count']
    hexbuf = line[start:end]
    countDict = rngLogData.get(filename)
    if countDict == None:
        countDict = {}
        rngLogData[filename] = countDict
    countDict[int(count)] =''.join(hexbuf.split())
    
def writeRNGResp(rngTestData, rngLogData):
    for filename, testData in rngTestData.items():
        filenameParts = filename.split('.')
        respFile = open(filenameParts[0] + '.rsp', 'w')
        resps = rngLogData.get(filename)
        if resps is None:
            print 'Result RNG data not found for file', filename
            continue
        writeResults(respFile, filename, resps, testData)
        respFile.close()

def writeResults(respFile, filename, resps, tests):
    first = True
    for count, params in tests.items():
        hexbuf = resps.get(count)
        if hexbuf is None:
            print 'No response found for count', count, 'and file', filename
            continue
        key = params.get('Key')
        v = params.get('V')
        dt = params.get('DT')
        if key is None or v is None or dt is None:
            print 'Missing required parameter for count', count, 'and file', filename, 'key', key, 'v', v, 'dt', dt
            continue
        if first:
            respFile.write('[X9.31]\n[AES 128-Key]\n\n')
            first = False
        writeTag(respFile, 'COUNT', str(count))
        writeTag(respFile, 'Key', key)
        writeTag(respFile, 'DT', dt)
        writeTag(respFile, 'V', v)
        writeTag(respFile, 'R', hexbuf)
        respFile.write('\n')
