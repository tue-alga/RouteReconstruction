from .Experiment import Experiment
# Loops bindings via Python
from PyLoops import FlowField, ProblemInstance, Graph, DecompositionResult, FrechetHittingPathsDecomposition,WeightedFrechetDecomposition, \
    TrivialDecomposition

from .Messages import Messages
from .ResultsFolder import ResultsFolder

# Sys libraries
import json
import random
from pathlib import Path
import itertools
from datetime import datetime
import os
import sys
import re
from .Messages import Messages

class NnlsApplier:
    def __init__(self, decomposer, progressStream, result, runFolder, i, j):
        self.decomposer = decomposer
        self.progressStream = progressStream
        self.result = result
        self.runFolder = runFolder
        self.i = i
        self.j = j
        self.logFilePrefix = ''
        self.logPrefix = ''
        self.batchSize = -1
        self.pruneValue = 0.00001
        self.retainSize = -1
        self.retainFraction = True
        self.incremental = False
        self.greedyRefine = False
        self.subselectHighest = -1
    def name(self):
        return 'nnls'

    def nextStep(self):
        """Apply NNLS to current result and log messages related to that
        """
        self.decomposer._writeMessage(self.progressStream, Messages.MSG_BASISSIZE, 'it {it}: basis size {basisSize}',{
            'it': self.i, 'basisSize': self.result.basisSize()
        })
        self.progressStream.flush()
        # Apply NNLS
        if self.greedyRefine:
            self.result.greedyRefine()
        elif self.incremental and self.result.basisSize() > self.retainSize:
            self.decomposer._print('Incremental NNLS')
            self.result.applyNNLSIncrementally(self.batchSize, self.retainSize, self.retainFraction)
        else:
            self.result.applyNNLSAndPrune(self.pruneValue)
        # Select top weighted paths for export and objective value computation
        if self.subselectHighest > 0:
            self.decomposer._print('Subselecting high weight paths, total {}'.format(self.subselectHighest))
            self.decomposer._print('Old objective value: {}'.format(self.result.objectiveValue()))
            self.result.pruneToTopWeighted(self.subselectHighest)
            self.result.recomputeObjectiveValue()
        self.decomposer._print('Objective value: {}'.format(self.result.objectiveValue()))
        self.decomposer._writeMessage(self.progressStream, Messages.MSG_NNLSAPPLIED, 'it {it}: nnls {objectiveValue}',{
            'it': self.i, 'objectiveValue': self.result.objectiveValue()
        })
        self.progressStream.flush()
        self.i += 1
    def paramDescription(self):
        return ''
class Decomposer:
    # MSG_RESUME = 'resume'
    # MSG_NEWBASIS = 'newbasis'
    # MSG_BASISSIZE = 'basissize'
    # MSG_NNLSAPPLIED = 'nnlsapplied'
    # MSG_INITNORM = 'initnorm'

    # RESULT_FILE_RE = 'it=([0-9]+)\-([0-9]+)_(newbasis|nnlsApplied)(?:\-)?([a-zA-Z]+)*\.boosttxt'

    # NNLS_FILENAME_FORMAT = 'it={}-{}_nnlsApplied.boosttxt' # Args: iteration and phase
    # ALG_RESULT_FILENAME_FORMAT = 'it={}-{}_newbasis-{}.boosttxt' # Args: iteration, phase and alg name

    def __init__(self, globalLogFile):
        self.epsilon = 10
        self.exp = Experiment()
        self.graph = None
        self.field = None
        self.problemInstance = None
        self.runName = ''
        self.iterations = 1
        self.overwriteExisting = False
        self.algNames = []
        # Raw JSON dict
        self.data = None
        self.globalLogFile = globalLogFile
        # Results folder
        self.resultsFolder = None
        # The NNLS applier
        self.nnlsApplier = None

    def _print(self, string):
        print('[Decomposer-{}] {}'.format(os.getpid(),string))

    def _baseFolder(self):
        return self.exp.resultsFolder() / self.data['name']

    def logFolder(self):
        return self.exp.logsFolder() / self.parameterizedRunName()

    def resultFolderFromData(self):
        parKeys = sorted(list(self.data['runParameters'].keys()))
        parts = []
        baseFolder = self._baseFolder()
        for key in parKeys:
            val = key + '_' + str(self.data['currentParameters'][key])
            baseFolder = baseFolder / val
        return baseFolder

    def parameterizedRunName(self):
        name = self.data['name']
        parKeys = sorted(list(self.data['runParameters'].keys()))
        for key in parKeys:
            name += '_' + key + '_' + str(self.data['currentParameters'][key])
        return name

    def _loadProblemInstance(self, data):
        """
        Loads the problem instance, as specified by the data dictionary assigned to this decomposer
        """
        probInstDict = data['problemInstance']
        # Check field path
        fieldFile = probInstDict['field']
        if not Path(self.exp.fieldsFolder() / fieldFile).exists():
            raise Exception('Field file does not exist!')
        # Read field
        fieldFilePath = str(self.exp.fieldsFolder() / fieldFile)
        self.field = FlowField()
        self.field.read(fieldFilePath)
        self._print('Field read')

        # Read associated graph
        self.graph = Graph()
        graphFile = probInstDict['map']
        graphFile = self.exp.mapsFolder() / graphFile
        self._print('Reading graph from {}'.format(graphFile))
        self.graph.read(str(graphFile))
        self._print('Graph read')
        
        # Update field according to graph
        self.field.setFromPaths(self.graph.edgeCount())   

        self._print('Creating ProblemInstance')

         # Setup problem instance
        self.problemInstance = ProblemInstance()
        
        self.problemInstance.setField(self.field,fieldFilePath)
        self.problemInstance.setGraph(self.graph,str(graphFile))

        # Set the epsilon
        self.problemInstance.epsilon = probInstDict['epsilon']
        amount = 0
        pathSelectDict = probInstDict['pathSelect']
        self._print('Determining representatives')
        if pathSelectDict['mode'] == 'percentage':
            percentage = pathSelectDict['value']
            random.seed(pathSelectDict['seed'])
            pCount = self.field.pathCount()
            inds = list(range(pCount))
            # Randomize the path selection
            random.shuffle(inds)
            amount = int(percentage * pCount)
            self._print('Via percentage, total {} paths'.format(amount))
            if 'file' in pathSelectDict:
                self.problemInstance.setAvailablePathsFromFile(str(self.exp.baseFolder() / pathSelectDict['file']), inds[:amount],convertCrs=True)
            else:
                self.problemInstance.setAvailablePaths(inds[:amount])
        elif pathSelectDict['mode'] == 'count':
            random.seed(pathSelectDict['seed'])
            pCount = self.field.pathCount()
            inds = list(range(pCount))
            random.shuffle(inds)
            amount = pathSelectDict['value']
            self._print('Via fixed amount, total {} paths'.format(amount))
            if 'file' in pathSelectDict:
                self.problemInstnace.setAvailablePathsFromFile(str(self.exp.baseFolder() / pathSelectDict['file']), inds[:amount],convertCrs=True)
            else:
                self.problemInstance.setAvailablePaths(inds[:amount])
        else:
            raise Exception("Invalid argument for path select")

        self._print('Selected paths {}'.format(amount))
        self.iterations = probInstDict['iterations']

    def _writeMessage(self, stream, msgType, messageFormat, dataDict):
        msg = messageFormat.format(**dataDict)
        timeStr = self._timeString()
        jsonMsg = json.dumps({'msg':msg,'type':msgType,'logTime':timeStr, **dataDict})
        stream.write('[{}]: {}\n'.format(timeStr, jsonMsg))
    
    def loadSettingsFromJson(self, fileName, runName = None):
        runNameToUse = fileName
        if runName is not None:
            runNameToUse = runName
        
        path = self.exp.settingsFolder() / (fileName+'.json')
        data = None
        with open(path) as f:
            data = json.load(f)
        if data is None:
            raise Exception("Failed loading json")

        self.loadSettingsFromObj(data, runNameToUse)

    def loadSettingsFromObj(self, data, runName = None):
        self.setData(data)
        self._print('Running with algs {}'.format(','.join(self.algNames)))
        self._print('Number of iterations {}'.format(self.iterations))

    def setData(self,data):
        """Set the data object to the given object and set local related state
        :param data dict: The data dictionary
        """
        self.data = data
        self.runName = data['name']
        self.overwriteExisting = data['overwriteExisting']
        self.algs = data['algs']
        # Make sure results folder exists
        os.makedirs(self.resultFolderFromData(),exist_ok=True)
        # Setup results folder object
        self.resultsFolder = ResultsFolder(self.resultFolderFromData())

    def decompose(self, decomposeArgs={}):
        self._print('Running decompose()')
        if self.checkIsOccupied():
            self._print('Given decomposition object is already being processed')
            return
        if self.resultsFolder.isDone():
            self._print('Already done')
            return
        self._loadProblemInstance(self.data)
        if 'redoPartial' in decomposeArgs:
            i = decomposeArgs['redoPartial']['i']
            j = decomposeArgs['redoPartial']['j']
            self.decomposeSingle(i,j)
        if self.overwriteExisting:
            self._print('Overwriting existing')
            self.decomposeSingle()
        else:
            self._print('Resuming')
            self.tryResume()

    def checkIsOccupied(self):
        runFolder = self.resultFolderFromData()
        if runFolder.exists():
            if not (runFolder / 'process.lock').exists():
                #print('No process running setting')
                return
            pid = None
            with open(runFolder / 'process.lock') as f:
                pid = f.read()
                #print('Read pid "{}"'.format(pid))
            if len(pid) == 0:
                return False
            pid = int(pid)
            try:
                os.kill(pid,0)
            except OSError:
                #print('Already occupied')
                return False
            else:
                #print('Process not running')
                return True
        else:
            #print('No process lock file')
            return False

    def _timeString(self):
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    def _setupDecomposersAndResult(self, logsFolder, progressStream, runFolder):
        # Global parameters
        algParams = {}
        algParams['logDir'] = str(logsFolder)
        algParams['logPrefix'] = 'output'

        # KEEP THE LIST SORTED! Then it can be reproduced deterministically.
        self._print('Creating decomposers')
        decomposerNames = sorted([name for name in self.algs.keys()])
        # Create the decomposition result object
        result = DecompositionResult(self.problemInstance)
        decomposers = [self._initDecomposer(a,result, algParams) for a in decomposerNames]
        self._print('Creating decomposition result')
        self.nnlsApplier = NnlsApplier(self, progressStream, result, runFolder, 0, len(decomposers))
        if 'greedyRefine' in self.data['problemInstance']:
            self.nnlsApplier.greedyRefine = True
        if 'subselectHighest' in self.data['problemInstance']:
            self.nnlsApplier.subselectHighest = self.data['problemInstance']['subselectHighest']
        if 'incremental' in self.data['problemInstance']:
            self.nnlsApplier.incremental = True
            self.nnlsApplier.batchSize = self.data['problemInstance']['incremental']['batchSize']
            self.nnlsApplier.retainSize = self.data['problemInstance']['incremental']['retainSize']
            self.nnlsApplier.retainFraction = self.data['problemInstance']['incremental'].get('retainFraction',True)
        if 'batchSize' in self.data['problemInstance']:
            self.nnlsApplier.batchSize = self.data['problemInstance']['batchSize']
        if 'pruneValue' in self.data['problemInstance']:
            self.nnlsApplier.pruneValue = self.data['problemInstance']['pruneValue']
        decomposers.append(self.nnlsApplier)
        decomposerNames.append(self.nnlsApplier.name())
        return (decomposers, decomposerNames, result, algParams)

    def _applyNnls(self, i, j, result, progress, runFolder):
        """Apply NNLS to current result and log messages related to that
        """
        self._writeMessage(progress, Messages.MSG_BASISSIZE, 'it {it}: basis size {basisSize}',{
            'it': i, 'basisSize': result.basisSize()
        })
        progress.flush()
        # Apply NNLS
        result.applyNNLSAndPrune(0.00001)
        self._print('Objective value: {}'.format(result.objectiveValue()))
        self._writeMessage(progress, Messages.MSG_NNLSAPPLIED, 'it {it}: nnls {objectiveValue}',{
            'it': i, 'objectiveValue': result.objectiveValue()
        })
        progress.flush()
        outPath = runFolder / Messages.NNLS_FILENAME_FORMAT.format(i,j)
        result.write(str(outPath))

    def tryResume(self):
        self._print('Trying to resume')

        self.resultsFolder.parseResults()
        if not self.resultsFolder.hasResults():
            self._print('No results in result folder')
            self.decomposeSingle()
            return
        if self.resultsFolder.isDone():
            self._print('Already done')
            return
        lastResult = self.resultsFolder.results[-1]
        i = lastResult['it']
        j = lastResult['step']
        self._print('Resuming after {}-{}'.format(i,j))
        self.decomposeSingle(i,j)

    def _initDecomposer(self, decomposerName, decompositionResult, baseParams):
        d = self.exp.createDecomposer(decomposerName)
        d.setDecompositionResultObject(decompositionResult)
        pars = {**self.algs[decomposerName], **baseParams}
        self._print('Setting parameters for decomposer {}'.format(decomposerName))
        for k,v in pars.items():
            if hasattr(d, k):
                self._print('[Decomposer]: Setting attribute "{}" to {} for {}'.format(k,v,d.name()))
                setattr(d,k,v)
            else:
                self._print('[Decomposer]: {} has not attribute "{}"'.format(d.name(), k))
        return d

    def _decomposeStep(self, d, logPrefix, runFolder, i, j, progress, result):
        # Update log prefix
        d.logFilePrefix = '{}_{}'.format(logPrefix,d.name())
        d.logPrefix='[pid={},run={}]'.format(os.getpid(),self.parameterizedRunName())
        # Run step and compute time spent (seconds)
        start = datetime.timestamp(datetime.now())
        d.nextStep()
        timeSpent = datetime.timestamp(datetime.now()) - start
        # Write intermediate result
        outPath = runFolder / Messages.ALG_RESULT_FILENAME_FORMAT.format(i,j,d.name())
        result.write(str(outPath))
        # Convert to time spent to minutes and log
        timeSpentMinutes = timeSpent / 60
        self._writeMessage(progress, Messages.MSG_NEWBASIS,'it={it}-{step} new basis size {basisSize}, in minutes {timeSpent}',
            {'it':i,'step':d.name(), 'basisSize':result.basisSize(), 'timeSpent':timeSpentMinutes}
        )
        progress.flush()

    def decomposeSingle(self,startI = 0,startJ=-1):
        isResuming = startI > 0 or startJ >= 0


        with open(self.globalLogFile, 'a+') as f:
            if isResuming:
                f.write('{}: resuming {} at it {}, step {}\n'.format(self._timeString(), self.runName, startI, startJ))
            else:
                f.write('{}: running {}\n'.format(self._timeString(), self.runName))
        # The folder to save stuff to
        runFolder = self.resultFolderFromData()
        # Fail if we overwrite previous resultss
        os.makedirs(runFolder, exist_ok=True)
        # Logs folder
        logsFolder = self.exp.logsFolder() / self.parameterizedRunName()
        # Fail if we overwrite previous results
        os.makedirs(logsFolder, exist_ok=True)
        if not isResuming:
            with open(runFolder /'settings.json', 'w') as f:
                json.dump(self.data, f)

        with open(runFolder / 'process.lock','w') as f:
            f.write(str(os.getpid()))

        progress = open(runFolder / 'progress.txt','a+')
        # Setup decomposers etc.
        (decomposers, decomposerNames, result, algParams) = self._setupDecomposersAndResult(logsFolder, progress, runFolder)

        if isResuming:
            found = False
            for f in os.listdir(runFolder):
                if f.startswith('it={}-{}'.format(startI, startJ)):
                    self._print('Loading previous result {}'.format(f))
                    found = True
                    result.read(str(runFolder/f))
            if not found:
                print("Could not find appropriate start file")
                return

            # Write algorithm descriptions
        with open(runFolder /'algDesc.txt', 'a+') as f:
            f.write('{}\n'.format(self._timeString()))
            for d in decomposers:
                f.write(d.name() +' ' + d.paramDescription() + '\n')
        self._print('Wrote alg description')
        # Get list of algorithms
        decompNames = '_'.join([d.name() for d in decomposers])
        # Save the problem instance
        if not isResuming:
            self.problemInstance.write(str(runFolder / 'problemInstance.boosttxt'))
        self._print('Wrote problem instance')

        if startI > 0:
            self._writeMessage(progress, Messages.MSG_RESUME,'Resuming',{})
        self._writeMessage(progress, Messages.MSG_INITNORM,'Initial field sqnorm: {norm}',
            {'norm':self.field.squareLength()}
        )
        progress.flush()
        # Run the iterations
        self._print('Running {} iterations'.format(self.iterations))
        for data in itertools.product(range(self.iterations), range(len(decomposers))):
            (i,j) = data
            if i < startI or (i == startI and j <= startJ):
                continue
            # Setup log prefix
            logPrefix = algParams['logPrefix']
            logPrefix += '_it={}'.format(i)
            if j == 0:
                self._print('Starting iteration {}/{}'.format(i+1, self.iterations))
            # Active decomposer
            d = decomposers[j]
            self._print('Starting step {}: {}'.format(j+1,decomposerNames[j]))
            self._decomposeStep(d, logPrefix, runFolder, i, j, progress, result)
            self._print('Step {} done'.format(j+1))

            if j == len(decomposers)-1:
                self._print('Done with it {}/{}'.format(i+1, self.iterations))
        progress.close()

        with open(self.globalLogFile, 'a+') as f:
            f.write('{}: finished {}\n'.format(self._timeString(), self.parameterizedRunName()))
        os.remove(runFolder / 'process.lock')
