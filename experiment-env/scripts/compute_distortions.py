from LoopsHelpers import Experiment, DistortionComputer
import sys
import os
import re
from pathlib import Path
import json
import pickle
import multiprocessing
from argparse import ArgumentParser

from PyLoops import ProblemInstance, Graph, FlowField, DecompositionResult
from LoopsHelpers import Experiment, DistortionComputer, ResultsFolder, SettingsCollectionParser
from LoopsHelpers.deepsetting import getDeep
from datetime import datetime

def resolvePath(baseFolder, fullPath):
    if fullPath.startswith('/data/bram/loops/'):
        fullPath = fullPath[len('/data/bram/loops/'):]
    return baseFolder / fullPath

def mean(lst):
    if len(lst) == 0:
        return 0
    total = 0
    for l in lst:
        total += l
    return total / float(len(lst))

def computeDistortion(args):
    print(args)
    (folder, fileName, outFilePath) = args
    dc = DistortionComputer()
    dc.computeInFolder(folder, fileName,outFilePath)

def findLastResult(folderPath):
    pattern = 'it=(.*)\-([^_]*)_'
    highNum = 10000
    entries = []
    for entry in os.listdir(folderPath):
        if not entry.startswith('it'):
            continue
        match = re.search(pattern, entry)
        if match is None:
            continue
        index = int( match.group(1)) * highNum + int(match.group(2))
        entries.append((index, entry))
    if len(entries) == 0:
        return None

    entries.sort(key=lambda v: v[0])
    return entries[-1][1]

class Object(object):
    pass


if __name__ == '__main__':
    parser = ArgumentParser(description='Compute distortions')
    parser.add_argument('-f','--folder',type=str, help='Compute distortions for results in given folder')
    parser.add_argument('-i','--inputFile',type=str, help='Compute for a specific input experiment')
    parser.add_argument('-p','--parallel', type=int, default=4, help='Number parallel processes to use')
    parser.add_argument('-o','--overwrite',action='store_true', help='Overwrite existing results')
    parser.add_argument('-q','--outputFileName',type=str,help='Name(including extension) of output file')
    parser.add_argument('-r','--reverse', action='store_true', help='Compute reverse distortion')
    parser.add_argument('-w','--useRepresentatives', action='store_true', help='Compute distortion with representatives')
    parser.add_argument('-t','--topElements',type=int, help='Only compute for top specified number of elements')
    parser.add_argument('-d','--dryRun',action='store_true', help='Apply a dry run, not modifying but only printing data')
    parser.add_argument('-a','--autoRun',action='store_true', help='Dont ask verify question to start')
    parser.add_argument('-y','--perIteration', action='store_true', help='Compute a distortion the result of every iteration. Applies the distortion to the result of the last step in the iteration')
    parser.add_argument('-c','--configFile', type=str, help='Specify path to config file, relative to base folder. All settings will be overwritten with any contents of the file. Also, folders and files can be specified')

    args = parser.parse_args()
    e = Experiment()
    rfs = []
    settingDicts = []
    viaRepr = 0
    if args.configFile is not None:
        configData = None
        with open(e.baseFolder() / args.configFile) as f:
            configData = json.load(f)
        if 'parameters' in configData:
            # Overwrite parameters in the args object from the config file.
            for k,v in configData['parameters'].items():
                if k == 'parallellism':
                    continue
                setattr(args, k, v)
        print(args)
        if 'folders' in configData:
            for f in configData['folders']:
                rfs.append(ResultsFolder(e.resultsFolder() / f))
                dArgs=  {}
                try:
                    localDat = None
                    with open(e.resultsFolder() / f / 'settings.json') as inFile:
                        localDat = json.load(inFile)
                    v = getDeep('problemInstance.pathSelect.file',localDat)
                    if v is not None:
                        dArgs['useRepresentatives'] = True
                        viaRepr += 1
                except:
                    pass
                settingDicts.append(dArgs)
        if 'files' in configData:
            for f in configData['files']:
                parser = SettingsCollectionParser()
                parser.parseSettingsTxt(e.settingsFolder() / f)
                for d in parser.settingObjects:
                    rfs.append(ResultsFolder.resultFolderFromData(d,e.resultsFolder()))
                    dArgs=  {}
                    try:
                        v = getDeep('problemInstance.pathSelect.file',d)
                        if v is not None:
                            dArgs['useRepresentatives'] = True
                            viaRepr += 1
                    except:
                        pass
                    settingDicts.append(dArgs)
    dc = DistortionComputer()
    dc.perIteration = args.perIteration
    if args.outputFileName is not None:
        dc.outputFile = args.outputFileName
    dc.dryRun = args.dryRun
    dc.useRepresentatives = args.useRepresentatives
    if args.topElements is not None:
        dc.topElements = args.topElements
    if args.reverse:
        dc.computeReverse = True
    dc.overwrite = args.overwrite == True
    # Add result folders/files if not loaded yet via config file
    if len(rfs) == 0:        
        if not args.inputFile is None:
            # Read the experiments
            parser = SettingsCollectionParser()
            parser.parseSettingsTxt(e.settingsFolder() / args.inputFile)
            rfs = [ResultsFolder.resultFolderFromData(d,e.resultsFolder()) for d in parser.settingObjects]
        else:
            rfs = ResultsFolder.findResultFolders(e.resultsFolder())
    dc.parallellism = args.parallel
    print('Total tasks: {}, of which {} compare to representatives instead of MM trajectories'.format(len(rfs), viaRepr))
    if not args.autoRun:
        answer = input('Start the above runs? (y/n)\n')
        if not answer in ['y','yes','1']:
            exit()
    dc.computeResultFoldersMp(rfs, settingDicts)
