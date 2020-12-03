from LoopsHelpers import Experiment
from LoopsHelpers.deepsetting import *
import os
from pathlib import Path
from argparse import ArgumentParser
import json

import fnmatch

class JsonDoc:
    def __init__(self,path=''):
        self.path = path
        self.data = None
        if self.path:
            self.load(self.path)
    def load(self, path):
        self.path = path
        with open(path) as f:
            self.data = json.load(f)
    def save(self):
        with open(self.path,'w') as f:
            json.dump(self.data,f,indent=4)

def replaceInFile(fullPath, valType, argData, path):
    doc = JsonDoc(fullPath)
    el = getDeep(path, doc.data)
    if valType == 'k':
        key = '.'.join(path.split('.')[:-1]) + '.' + argData
        setDeep(key,doc.data,el)
        deleteDeep(path, doc.data)
    else:
        val = argData
        if valType == 'i':
            val = int(val)
        elif valType == 'b':
            val = bool(val)
        elif valType == 'f':
            val = float(val)
        setDeep(path, doc.data, val)
    doc.save()

def addInFile(fullPath, valType, argData, path, overwrite):
    doc = JsonDoc(fullPath)
    el = None
    try:
        el = getDeep(path, doc.data)
    except:
        el = None
    if el is not None:
        if not overwrite:
            print('Key already present in {}, skipping'.format(fullPath))
            return
    # Set deep
    val = argData
    if valType == 'i':
        val = int(val)
    elif valType == 'b':
        val = bool(val)
    elif valType == 'f':
        val = float(val)
    elif valType == 's':
        val = val
    else:
        raise Exception("Unsupported argument type for add {}".format(valType))
    setDeep(path, doc.data, val)
    doc.save()

def getFiles(args):
    files = []    
    if args.isGlob:
        for f in os.listdir(args.exp.settingsFolder()):
            if fnmatch.fnmatch(f, args.fileOrGlob):
                files.append(args.exp.settingsFolder() / f)
    else:
        files.append(args.exp.settingsFolder() / args.fileOrGlob)
    return files

def replacer(args):
    valType = args.newVal[0]
    argData = args.newVal[2:]
    files = getFiles(args)
    for f in files:
        try:
            replaceInFile(f, valType, argData, args.path)
        except KeyError:
            print('Ignoring {}, path not present'.format(f)) 
        except Exception as e:
            print('Error: {}'.format(e))

def adder(args):
    valType = args.newVal[0]
    argData = args.newVal[2:]
    overwrite= args.overwrite
    files = getFiles(args)
    for f in files:
        try:
            addInFile(f, valType, argData, args.path, overwrite)
        except KeyError:
            print('Ignoring {}, path not present'.format(f)) 
        except Exception as e:
            print('Error: {}'.format(e))

if __name__ == '__main__':
    parser = ArgumentParser(description='Manipulate setting files')
    parser.add_argument('fileOrGlob', type=str, help='File or glob pattern')
    parser.add_argument('-g','--isGlob',action='store_true', help='Is fileOrGlob a glob pattern')

    subparsers = parser.add_subparsers()
    addParser = subparsers.add_parser('add')
    addParser.add_argument('-o','--overwrite', action='store_true', help="Overwrite key if it already exists")
    addParser.set_defaults(func=adder)

    replaceParser = subparsers.add_parser('replace')
    replaceParser.set_defaults(func=replacer)

    parser.add_argument('path',type=str,help='Path in the JSON to modify')
    parser.add_argument('newVal',type=str,help='New value, specified as <type>:<value>, with type one of "i","b","f","k","s" for integer, bool, float, string or key')

    #addParser.add_argument('-o','--overwrite',action='store_true',help='Overwrite key if it already exists')
    parser.set_defaults(exp=Experiment())
    print('preparse')
    args = parser.parse_args()
    # Call appropriate func
    print('running func')
    args.func(args)
