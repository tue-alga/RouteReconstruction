# Online Python compiler (interpreter) to run Python online.
# Write Python 3 code in this online editor and run it.
import json
import collections
import functools

recursive_dict = lambda: collections.defaultdict(recursive_dict)

def setDeep(path,dictObj, value):
    retDict = recursive_dict()
    retDict.update(dictObj)
    parts = path.split('.')
    d = retDict
    for key in parts[:-1]:
        d = d[key]
    d[parts[-1]] = value

    return retDict
    
def deleteDeep(path,dictObj):
    retDict = recursive_dict()
    retDict.update(dictObj)
    parts = path.split('.')
    d = retDict
    for key in parts[:-1]:
        d = d[key]
    del d[parts[-1]]

    return retDict

def getDeep(path, dictObj):
    retDict = recursive_dict()
    retDict.update(dictObj)
    return functools.reduce(lambda d, k: d[k] if d else None, path.split('.'), retDict)
