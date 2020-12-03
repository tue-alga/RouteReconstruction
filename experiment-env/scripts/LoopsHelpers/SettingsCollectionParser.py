from .Decomposer import Decomposer
from .Experiment import Experiment
from PyLoops import WeightedFrechetDecomposition, FrechetHittingPathsDecomposition
import sys
import os
from multiprocessing import Pool
from argparse import ArgumentParser
import json
import copy
import itertools
from .deepsetting import setDeep

class SettingsCollectionParser:
    def __init__(self, overwriteExisting=False):
        self.settingFiles = []
        # Will contain all settings objects to run
        self.settingObjects = []
        self.settingFilesFullPaths = {}
        self.overwriteExisting = overwriteExisting
        self.exp = Experiment()
        # All base dictionaries from which runs are derived
        self.parentObjects = []

    def _parseAllPairsRun(self, data):
        keys = sorted(list(data['runParameters'].keys()))
        numKeys = len(keys)
        keyVals = [data['runParameters'][key] for key in keys]
        keysIt = [range(len(data['runParameters'][d])) for d in keys]
        baseRunName = data['runName']

        for combo in itertools.product(*keysIt):
            # Copy the data
            dat = copy.deepcopy(data)
            runNameSuffix = '_'.join(['{}[{}]'.format(keys[i].split('.')[-1],combo[i]) for i in range(len(keys))])
            for i in range(len(keys)):
                dat = setDeep(keys[i], dat, keyVals[i][combo[i]])

            # Add subobject that says which index of the runparameters are used
            dat['currentParameters'] = {}
            for j in range(numKeys):
                dat['currentParameters'][keys[j]] = combo[j]
            
            dat['runName'] = baseRunName + '_' + runNameSuffix
            # Create clean dict
            settingObj = dict()
            settingObj.update(dat)
            self.settingObjects.append(settingObj)

    def _getCombinationsIterator(self, data, keys, valueIterators):
        runMode = data['runMode']
        if runMode == 'allPairs':
            return itertools.product(*keysIt)
        elif runMode == 'combinations':
            return zip(*keysIt)
        else:
            raise Exception('Unknown combination type {}'.format(runMode))

    def _parseListCombinationsRun(self, data):
        keys = sorted(list(data['runParameters'].keys()))
        numKeys = len(keys)
        keyVals = [data['runParameters'][key] for key in keys]
        runLen = min([len(data['runParameters'][d]) for d in keys])
        baseRunName = data['runName']
        exclude = data.get('runExclude',[])
        
        for i in range(runLen):
            combo = [i for j in range(len(keys))]

            # Copy the data
            dat = copy.deepcopy(data)
            runNameSuffix = '_'.join(['{}[{}]'.format(keys[i].split('.')[-1],combo[i]) for i in range(len(keys))])
            # Assign the values
            for j in range(len(keys)):
                dat = setDeep(keys[j], dat, keyVals[j][combo[j]])

            # Add subobject that says which index of the runparameters are used
            dat['currentParameters'] = {}
            for j in range(numKeys):
                dat['currentParameters'][keys[j]] = combo[j]
            
            dat['runName'] = baseRunName + '_' + runNameSuffix
            settingObj = dict()
            settingObj.update(dat)
            self.settingObjects.append(settingObj)
    
    def _parseSettingFile(self, fileNameIn):
        data = None
        fileName = fileNameIn
        if fileName in self.settingFilesFullPaths:
            with open(self.settingFilesFullPaths[fileName]) as f:
                data = json.load(f)
        else:
            with open(self.exp.settingsFolder() / (fileName+'.json')) as f:
                data = json.load(f)
        self.parentObjects.append(data)
        runMode = data['runMode']
        runName = fileName

        # Set defaults
        data['overwriteExisting'] = self.overwriteExisting
        data['runName'] = runName

        if runMode == 'allPairs':
            self._parseAllPairsRun(data)
        elif runMode == 'combinations':
            self._parseListCombinationsRun(data)
        elif runMode == 'single':
            data['runName'] = runName
            self.settingObjects.append(data)
            
    def _parseSettingFiles(self):        
        for f in self.settingFiles:
            self._parseSettingFile(f)
        for obj in self.settingObjects:
            obj['overwriteExisting'] = self.overwriteExisting

    def writeSettingFiles(self, folder):
        outFolder = self.exp.settingsFolder() / folder
        os.makedirs(outFolder, exist_ok=True)
        for settObj in self.settingObjects:
            with open(outFolder / '{}.json'.format(settObj['runName']),'w') as f:
                json.dump(settObj, f)

    def parseSettingsTxt(self, file):
        self.settingFiles = []
        if file.suffix == '.json':
            self.settingFiles.append(file.stem)
        else:
            with open(file) as f:
                self.settingFiles = [ line.strip() for line in f if not line.startswith('#')]
            #print('Read {} setting files'.format(len(self.settingFiles)))
        self._parseSettingFiles()
        print('Total of {} tasks'.format(len(self.settingObjects)))
        
    def parseFolder(self, folder):
        self.settingFiles = []
        for el in os.listdir(self.exp.settingsFolder() / folder):
            el = el.strip()
            if not el.endswith('.json'):
                continue
            el = el.replace('.json','')
            self.settingFiles.append(el.strip())
            self.settingFilesFullPaths[el.strip()] = self.exp.settingsFolder() / folder / '{}.json'.format(el)
            #print('Read {} setting files: {}'.format(len(self.settingFiles), ','.join(self.settingFiles)))
        self._parseSettingFiles()
        print('Total of {} tasks'.format(len(self.settingObjects)))


    def checkExistence(self):
        notDone = []
        for settObj in self.settingObjects:
            dec = Decomposer(str(Experiment().logsFolder() / 'runs.log'))
            dec.loadSettingsFromObj(settObj)
            dec.overwriteExisting = settObj['overwriteExisting']
            res = dec.checkProgress()
            if not res[0]:
                notDone.append((dec.runName, res[1], res[2]))
        for el in notDone:
            print('{} not done: start from {},{}'.format(*el))
    
    def getSettingObjects(self):
        return self.settingObjects
