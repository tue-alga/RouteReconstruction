from pathlib import Path
from .Experiment import Experiment
from .ResultsFolder import ResultsFolder
from PyLoops import ProblemInstance, Graph, DecompositionResult, FlowField
import multiprocessing
import os
import re
import json


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
    dc = DistortionComputer()
    dc(args)

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


class DistortionComputer:
    RESULTS_FILE = 'distortion.json'
    def __init__(self):
        self.upperBound = 10000 # 10km upperbound for Frechet
        self.overwrite = False
        self.outputFile = DistortionComputer.RESULTS_FILE
        self.computeReverse = False
        self.topElements = -1
        self.dryRun = False
        self.perIteration = False
        self.useRepresentatives = False
        # Non-setting parameter
        self.parallellism = 1
        self.doneOnly = True
        self.recomputeDate = None
    def settingsToDict(self):
        keys = ['upperBound','overwrite','outputFile','computeReverse','topElements','dryRun','perIteration','useRepresentatives']
        d = {}
        for k in keys:
            d[k] = getattr(self, k)
        return d
    def setSettingsFromDict(self, dictObj):
        for k,v in dictObj.items():
            if hasattr(self, k):
                setattr(self,k,v)
    def __call__(self, dataObj={}):
        self.setSettingsFromDict(dataObj)
        if not 'folder' in dataObj:
            print('No folder in dataobject for DistortionComputer!')
            return
        if self.perIteration:
            self.computePerIteration(dataObj['folder'])
            return
        if not 'file' in dataObj:
            print('No folder and file in dataobject for DistortionComputer!')
            return
        self.compute(dataObj['folder'],dataObj['file'])
    def computePerIteration(self, folder):
        print('Starting per iteration computation')
        rf = ResultsFolder(folder)
        rf.parseResults()
        resultsPerIt = rf.resultPerIteration()
        outputFile = self.outputFile
        for k,v in resultsPerIt.items():
            self.outputFile = outputFile + str(k)
            # Take the last result
            self.compute(folder, v[-1]['file'])

    def compute(self, folder, file):
        exp = Experiment()
        fold = Path(folder)
        if not Path(str(folder / 'problemInstance.boosttxt')).exists():
            print('No result in folder {}. Skipping'.format(fold))
            return
        if (Path(folder) / self.outputFile).exists() and not self.overwrite and not self.dryRun:
            fileMTime = os.path.getmtime(Path(folder) / file)
            if os.path.getmtime(Path(folder) / self.outputFile) >= fileMTime:
                if self.recomputeDate is None or fileMTime >= self.recomputeDate:
                    print('Distortions already computed, found {} in folder {}'.format(self.outputFile,folder))
                    return
        pi = ProblemInstance()
        pi.read(str(folder / 'problemInstance.boosttxt'))
        g = exp.readMap(pi.graphFile())
        pi.setGraph(g, pi.graphFile())
        ff = exp.readFlowField(pi.fieldFile())
        pi.setField(ff, pi.fieldFile())
        dr = DecompositionResult(pi)
        path = Path(folder) / file
        dr.read(str(path))
        if self.topElements > 0:
            print('[DistortionComputer] Pruning to {} elements'.format(self.topElements))
            dr.pruneToTopWeighted(self.topElements)
        data = None
        print('[DistortionComputer] Running with {} elements in basis'.format(dr.basisSize()))
        print('Basis elements lengths:')
        print([len(dr.basisElement(i)) for i in range(dr.basisSize())])
        indices = []
        if self.dryRun:
            print(self.settingsToDict())
            print('Representatives count: {}'.format(pi.representativeCount()))
            data = []
        elif self.computeReverse:
            #(data,indices) = dr.computeReverseMinFrechet(self.upperBound)
            if self.useRepresentatives:
                data = dr.computeReverseMinFrechetRepresentatives(self.upperBound)
            else:
                (data,indices) = dr.computeReverseMinFrechet(self.upperBound)
        else:
            if self.useRepresentatives:
                data = dr.computeMinFrechetRepresentatives(self.upperBound)
            else:
                data = dr.computeMinFrechet(self.upperBound)
        outData = {}
        outData['args'] = self.settingsToDict()
        # Used basis size, can be pruned
        outData['basisSize'] = dr.basisSize()
        outData['values'] = data
        outData['closestIndices'] = indices
        outData['file'] = file
        outData['mean'] = mean(data)
        if not self.dryRun:
            with open(Path(folder) / self.outputFile, 'w') as f:
                json.dump(outData, f)
        else:
            print(outData)
        print('Computation done for {}'.format(folder))

    def computeResultFoldersMp(self, resultFolders,settingDicts = []):
        els = []
        print('DistortionComputer: {} input rfs'.format(len(resultFolders)))
        ind = -1
        for rf in resultFolders:
            ind += 1
            rf.parseMessages()
            rf.parseResults()
            if not rf.isDone() and self.doneOnly:
                print('Rf not done')
                continue
            args = self.settingsToDict()
            if len(settingDicts) == len(resultFolders):
                for k,v in settingDicts[ind].items():
                    args[k] = v
            args['folder'] = rf.folder
            args['file'] = rf.results[-1]['file']
            els.append(args)
        print('Settings:')
        print(self.settingsToDict())
        print('DistortionComputer: {} tasks'.format(len(els)))
        with multiprocessing.Pool(self.parallellism) as p:
            p.map(computeDistortion, els)
