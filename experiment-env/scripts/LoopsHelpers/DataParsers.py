import sys
import os
from .Experiment import Experiment
import re
import argparse
from pathlib import Path
import json
import pickle
from multiprocessing import Pool
from PyLoops import ProblemInstance, Graph, FlowField, DecompositionResult
from ResultsFolder import ResultsFolder

def resolvePath(baseFolder, fullPath):
    if fullPath.startswith('/data/bram/loops/'):
        fullPath = fullPath[len('/data/bram/loops/'):]
    return baseFolder / fullPath
# if __name__ == '__main__':
#     parser = argparse.ArgumentParser(description='Read progress.')
#     parser.add_argument('outFile', type=str, nargs='?',
#                         help='Output file name')
#     parser.add_argument('--prefix', type=str, action='store_const',
#                         const=sum, default=max,
#                         help='sum the integers (default: find the max)')

#     args = parser.parse_args()

class Datum:
    def __init__(self):
        self.wasRun = True
        self.name = ''
        self.runningTime = ''
        self.nnlsValues = []
        self.totalIts = 0
        self.hittingPathTimes = []
        self.weightedPathTimes = []
        self.prefixName = ''
        self.argIndices = {}
        self.minFrechetPerSrc = []
        self.meanFrechetDist = 0
        self.basisSize = 0
    def setNamesFromFolder(self, folderName):
        self.name = folderName
        parts = folderName.split('_')
        self.prefixName = parts[0]
        for i in range(1,len(parts)):
            m = re.search('([a-zA-Z]+)\[([0-9]+)\]', parts[i])
            if m is not None:
                self.argIndices[m.group(1)] = m.group(2)

def mean(lst):
    total = 0
    if len(lst) == 0:
        return 0
    for l in lst:
        total += l
    return total / float(len(lst))

def parseResultDirectory(directory, forceParse=False):
    exp = Experiment()
    if (exp.dataProcessingFolder() / '{}_data.json'.format(directory)).exists() and not forceParse:
        data = None
        with open(exp.dataProcessingFolder() / '{}_data.json'.format(directory)) as f:
            data = json.load(f)
        rf = ResultsFolder(directory)
        rf.parseProgress()
        if rf.lastSeenIt() <= data['totalIts']:
            return data
    dc = DistortionComputer()
    pp = ProgressParser()
    datum = pp.parse(directory)
    dcRes = dc.parse(directory)
    for k,v in dcRes.items():
        setattr(datum,k,v)
    with open(exp.dataProcessingFolder() / '{}_data.json'.format(datum.name), 'w') as f:
        json.dump(datum.__dict__, f)
    return datum.__dict__

# def fixResultDirectoryNames(directory):
#     """Fixes parse error of arguments in folder names
#     """
#     data = None
#     # Import json
#     with open(exp.dataProcessingFolder() / '{}_data.json'.format(datum.name)) as f:
#         data = json.load(f)
#     d = Datum()
#     d.setNamesFromFolder(directory)
#     data['prefixName'] = d.prefixName
#     data['argIndices'] = d.argIndices
#     # Set names
#     # Reexport

def parseIterationsAndTime(folder):
    exp = Experiment()
    rf = ResultsFolder(folder)
    if (exp.dataProcessingFolder() / '{}_data.json'.format(folder)).exists():
        data = None
        rf.parseProgress()
        with open(exp.dataProcessingFolder() / '{}_data.json'.format(folder)) as f:
            data = json.load(f)
        with open(exp.dataProcessingFolder() / '{}_data.jsonbckp'.format(folder),'w') as f:            
            json.dump(data,f)
        data['']
        

def parseAllResults():
    exp = Experiment()
    dc = DistortionComputer()
    pp = ProgressParser()
    totals = {}
    dirs = []
    for directory in os.listdir(exp.resultsFolder()):
        if os.path.isdir(exp.resultsFolder() / directory):
            dirs.append(directory)
    data = []        
    # with Pool(processes=8) as pool:
    # data = pool.map(parseResultDirectory, dirs)
    data = map(parseResultDirectory, dirs)
    for datum in data:
        if not (datum['prefixName'] in totals):
            totals[datum['prefixName']] = []
        totals[datum['prefixName']].append(datum)

    with open(exp.dataProcessingFolder() / 'total.json' ,'w') as f:
        json.dump(totals, f)



class DistortionComputer:
    def __init__(self):
        self.exp = Experiment()
    def parse(self, folder):
        pi = ProblemInstance()
        if not Path(str(self.exp.resultsFolder() / folder / 'problemInstance.boosttxt')).exists():
            print('Found no probleminstance in {}'.format(folder))
            return {'error':'Not started'}
        pi.read(str(self.exp.resultsFolder() / folder / 'problemInstance.boosttxt'))
        print(pi.fieldFile())
        print(pi.graphFile())
        g = Graph()
        g.read(str(resolvePath(self.exp.baseFolder(), pi.graphFile())))
        ff = FlowField()
        ff.read(str(resolvePath(self.exp.baseFolder(), pi.fieldFile())))
        
        pi.setGraph(g, pi.graphFile())
        pi.setField(ff, pi.fieldFile())
        print('Read {} paths in result')
        dr = DecompositionResult(pi)

        selectedEntry = None
        for i in range(20,-1,-1):
            p = Path(self.exp.resultsFolder() / folder / 'it={}-2_nnlsApplied.boosttxt'.format(i))
            if p.exists():
                dr.read(str(p))
                selectedEntry = folder +"-it{}".format(i)
                break
        if selectedEntry is None:
            for i in range(20,-1,-1):
                p = Path(self.exp.resultsFolder() / folder / 'it={}-1_nnlsApplied.boosttxt'.format(i))
                if p.exists():
                    dr.read(str(p))
                    selectedEntry = folder +"-it{}".format(i)
                    break
        if selectedEntry is None:
            return {'error':'No nnls apply it file found'}
        
        # 10 km upper bound
        data = dr.computeMinFrechet(10000)
        return {'minFrechetPerSrc':data,'meanFrechetDist':mean(data), 'basisSize':dr.basisSize()}
        # Read problem instance
        # Read last iteration 
        # Compute min frechet per element
        # Compute mean
        # Report

class ProgressParser:
    def __init__(self):
        self.data = []
        self.exp = Experiment()
    def parse(self, directory):
        dat = Datum()
        dat.name = directory
        dat.setNamesFromFolder(directory)
        # Add empty result
        if not Path(self.exp.resultsFolder() / directory / 'progress.txt').exists():
            dat.wasRun = False
            return dat
        # Parse progress
        with open(self.exp.resultsFolder() / directory / 'progress.txt') as f:
            
            for line in f:
                if 'Initial field sqnorm' in line:
                    parts = line.split()
                    dat.nnlsValues.append(float(parts[-1]))
                    continue
                m = re.search('it=([0-9]+)-([a-zA-Z]+):\snew basis size ([0-9]+), in ([0-9\.]+) minutes', line)
                if m is not None:
                    dat.totalIts = max(dat.totalIts, int(m.group(1)))
                    print('Found it {} for {}'.format(m.group(1),directory))
                    alg=  m.group(2)
                    if alg.startswith('Frecheth'):
                        dat.hittingPathTimes.append(float(m.group(4)))
                    elif alg.startswith('Weighted'):
                        dat.weightedPathTimes.append(float(m.group(4)))
                    continue
                m = re.search('it ([0-9]+): nnls ([0-9\.]+)', line)
                if m is not None:
                    parts = line.split()
                    dat.nnlsValues.append(float(parts[-1]))
                    print('NNLS val for {}: {}'.format(directory, dat.nnlsValues[0-1]))
        return dat
    @staticmethod
    def ifNotEmpty(lst, fun, default):
        if len(lst) > 0:
            return fun(lst)
        return default

    def writeToCsv(self, csvFile, delim):
        def writeLine(f, data,delim):
            f.write(delim.join(list(map(lambda x:'{}'.format(x), data)))+'\n')
        with open(csvFile,'w') as f:
            writeLine(f,['Name','Max it','Average WF time(min)','Average HP time(min)','NNLS diff','NNLS start'],delim)
            for d in self.data:
                writeLine(f, [dat.name, dat.totalIts, mean(dat.weightedPathTimes), mean(dat.hittingPathTimes), 
                ProgressParser.ifNotEmpty(dat.nnlsValues, lambda x: x[-1]-x[0], 'Not set'), 
                ProgressParser.ifNotEmpty(dat.nnlsValues, lambda x: x[0], 'Not set')], delim)

    def writeToScreen(self):
        print('Name    |Max it    |Average WF time(min)|Average HP time(min)|NNLS diff| NNLS start|')
        self.data.sort(key=lambda x:x.name)
        for dat in self.data:
            print('{}|{}|{}|{}|{}|{}'.format(dat.name, dat.totalIts, mean(dat.weightedPathTimes), mean(dat.hittingPathTimes), 
                ProgressParser.ifNotEmpty(dat.nnlsValues, lambda x: x[-1]-x[0], 'Not set'), 
                ProgressParser.ifNotEmpty(dat.nnlsValues, lambda x: x[0], 'Not set')))