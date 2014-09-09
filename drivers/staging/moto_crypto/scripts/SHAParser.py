'''
Created on Oct 25, 2012

@author: wwr004
'''

import os
from ParseUtils import hex2c
from ParseUtils import getParams
from ParseUtils import writeTag

def parseSHA(alg, reqdir):
    reqdata = {}
    monte = alg + 'Monte'
    for r,d,f in os.walk(reqdir):
        for arq in f:
            if arq.endswith('.req'):
                if arq.startswith(monte):
                    shaMonte = processSHAMonteFile(os.path.join(r,arq))
                elif arq.startswith(alg):
                    reqdata[arq] = processSHAFile(os.path.join(r,arq))
    return (reqdata, shaMonte)
                
def processSHAFile(arq):
    length = '-1'
    reqdata = {}
    for line in open(arq, 'r').readlines():
        line = line.strip()
        if len(line) > 0:
            first = line[:1]
            if (first != '#') & (first != '['):
                parts = line.split()
                if parts[1] != '=':
                    print "Error format line=", line
                lp = parts[0].lower()    
                if lp == 'msg':
                    if length == '-1':
                        print 'Msg without length'
                        return
                    reqdata[int(length)] = parts[2]
                    length = '-1'
                elif lp == 'len':
                    length = parts[2]
    return reqdata                        

def processSHAMonteFile(arq):
    for line in open(arq, 'r').readlines():
        line = line.strip()
        if len(line) > 0:
            first = line[:1]
            if (first != '#') & (first != '['):
                parts = line.split()
                if parts[1] != '=':
                    print "Error format line=", line
                lp = parts[0].lower()    
                if lp == 'seed':
                    shaMonte = parts[2]
                    return shaMonte

def writeShaTests(alg, out, shaData):
    shaCount = 0
    for filename, filedata in shaData.items():
        if shaCount == 0:
            out.write('static struct moto_test_hash_testvec moto_')
            out.write(alg.lower())
            out.write('[] = {\n')
        shawrite(out, filedata, filename)
        shaCount += len(filedata)
    if shaCount > 0: 
        out.write('};\n\n')
    return shaCount    

def shawrite(f, testvector, tname):
    ident = ' ' * 4                    
    doubleident = ident * 2
    for size in sorted(testvector.iterkeys()):
        value = testvector[size]
        f.write(ident)
        f.write('{\n')
        f.write(doubleident)
        f.write('.test_file_name = "')
        f.write(tname)
        f.write('",\n')
        f.write(doubleident)
        f.write('.plaintext = "')
        f.write(hex2c(value))
        f.write('",\n')
        f.write(doubleident)
        f.write('.psize = ')
        f.write(str(size))
        f.write('\n')
        f.write(ident)
        f.write('},\n')

def writeShaMonteTests(alg, out, shaMonteData):
    out.write('static char moto_test_monte_')
    out.write(alg)
    out.write('[] = "')
    out.write(hex2c(shaMonteData))
    out.write('";\n\n')

def parseSHAKernelLogLine(line, logData):
    paramsDict = getParams(line)
    if (not paramsDict.has_key('file')) and (not paramsDict.has_key('len')):
        return
    start = line.find('!')
    if start == -1:
        print 'Possible interrupted buffer.Line=', line
        return
    end = line.find('!', start + 1)
    if end == -1:
        print 'Possible missing buffer end.Line=', line
        return
    filename = paramsDict['file']
    input_len = paramsDict['len']
    hexbuf = line[start + 1:end]
    lenDict = logData.get(filename)
    if lenDict == None:
        lenDict = {}
        logData[filename] = lenDict
    lenDict[int(input_len)] =''.join(hexbuf.split())

def writeSHAResp(shaLength, data, logData):    
    for filename, testData in data.items():
        resps = logData.get(filename)
        if resps == None:
            print 'SHA response test data not found for file', filename
            continue
        filenameParts = filename.split('.')
        respFile = open(filenameParts[0] + '.rsp', 'w')
        respFile.write('[L=')
        respFile.write(shaLength)
        respFile.write(']\n\n')
        writeResults(respFile, filename, testData, resps)
        respFile.close()
 
def writeResults(respFile, filename, tests, resps):        
    for length in sorted(tests.iterkeys()):
        msg = tests[length]
        hexbuf = resps.get(length)
        if hexbuf == None:
            print 'No response found for length', length, 'and file', filename
            continue
        respFile.write('Len = ')
        respFile.write(str(length))
        respFile.write('\nMsg = ')
        respFile.write(msg)
        respFile.write('\nMD = ')
        respFile.write(hexbuf)
        respFile.write('\n\n')

def parseSHAMonteKernelLogLine(line, shaMonteLogData):
    paramsDict = getParams(line)
    if (not paramsDict.has_key('alg')) and (not paramsDict.has_key('count')):
        return
    start = line.find('!')
    if start == -1:
        print 'Possible interrupted buffer.Line=', line
        return
    end = line.find('!', start + 1)
    if end == -1:
        print 'Possible missing buffer end.Line=', line
        return
    alg = paramsDict['alg']
    count = paramsDict['count']
    hexbuf = line[start + 1:end]
    countDict = shaMonteLogData.get(alg)
    if countDict == None:
        countDict = {}
        shaMonteLogData[alg] = countDict
    countDict[int(count)] =''.join(hexbuf.split())    

#shaMonteData: Monte test seed
#shaMonteLogData: dict from alg to (dict from count to result)
def writeSHAMonteResp(alg, shaLength, shaMonteData, shaMonteLogData):
    respData = shaMonteLogData.get('moto-' + alg.lower())
    if respData == None:
        print 'Monte result not found for alg', alg
        return
    respFile = open(alg + 'Monte.rsp', 'w')
    respFile.write('[L=')
    respFile.write(shaLength)
    respFile.write(']\n\nSeed = ')
    respFile.write(shaMonteData)
    respFile.write('\n\n')
    for count in sorted(respData.iterkeys()):
        resp = respData[count]
        writeTag(respFile, 'COUNT', str(count))
        writeTag(respFile, 'MD', resp)
        respFile.write('\n')
    respFile.close()
