'''
Created on Oct 25, 2012

@author: wwr004
'''

# Converts a hex string to a string of hex bytes according to C format (e.g., aabb = \xaa\xbb)
def hex2c(hexstr):
    fstring = []
    for index, ch in enumerate(hexstr):
        if index % 2 == 0:
            fstring.append('\\x')
        fstring.append(ch)
    return ''.join(fstring)
        
def getParams(line):
    paramsDict = {}
    tokens = line.split()
    for token in tokens:
        if ':' in token:
            paramList = token.split(':')
            paramsDict[paramList[0]] = paramList[1]
    return paramsDict

def writeTag(f, tag, value):
    f.write(tag)
    f.write(' = ')
    f.write(value)
    f.write('\n')

def getBuffer(line, posStart = 0):
    start = line.find('!', posStart)
    if start == -1:
        return (-1, -1)
    end = line.find('!', start + 1)
    return (start + 1, end)