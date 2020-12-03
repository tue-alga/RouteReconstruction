import os
from pathlib import Path
from LoopsHelpers import Experiment, ResultsFolder,SettingsCollectionParser, Decomposer
from PyLoops import DecompositionResult, ProblemInstance
from argparse import ArgumentParser
import json
import time
from datetime import datetime

class CsvData:
    def __init__(self):
        self.name = ''
        self.lastIt = -1
        self.distortion = -1
        self.totalTime = -1
        self.args = ''
        self.lastStep = -1
        self.objectiveVal = -1
        self.basisSize = -1
        self.runData = ''
        self.runTime = {}
        self.reverseDistortion = -1
        self.reverseDistortionData = {}
        self.reverseDistortionWeighted = -1
        self.perIteration = {}
        self.totalWeightReducedBasis = -1
    @staticmethod
    def convert(value):
        if isinstance(value, str):
            return value
        if isinstance(value, list) or isinstance(value, dict):
            return json.dumps(value)
        else:
            return CsvData.convert(value.__dict__)
    @staticmethod
    def headers():
        return ['name','lastIt','lastStep','distortion','totalTime','args','objectiveVal','runData','runTime','basisSize','reverseDistortion','perIteration','reverseDistortionWeighted',
        'totalWeightReducedBasis','reverseDistortionData']
    def getLine(self, delim):
        args = list(map(CsvData.convert, [getattr(self,at) for at in CsvData.headers()]))
        return delim.join(args)

def print_table(table,separatingLines=True,hasHeader=True):
    col_width = [max(len(x) for x in col) for col in zip(*table)]
    totalWidth = (len(col_width) -1) * 3 + 4 + sum(col_width)
    start = 0
    if separatingLines:
        print('-'*totalWidth)
    if hasHeader:
        start = 1
        print( "| " + " | ".join(["{:{}}".format(x, col_width[i]) for i, x in enumerate(table[0])]) + " |")
        if separatingLines:
             print('-'*totalWidth)
         
    for j in range(start,len(table)):
        if table[j][0] == '-':
            print('-'*totalWidth)
        else:
            print( "| " + " | ".join(["{:{}}".format(x, col_width[i]) for i, x in enumerate(table[j])]) + " |")
    if separatingLines:
        print('-'*totalWidth)

def writeJsonFile(outputFileName,settingCollection, runDataObjects):
    toSerialize = {}
    toSerialize['parentObjects'] = settingCollection.parentObjects
    runsData = {}
    for run in runDataObjects:
        if not run.name in runsData:
            runsData[run.name] = []
        runData = {}
        for h in CsvData.headers():
            runData[h] = getattr(run, h)
        runsData[run.name].append(runData)
    toSerialize['data']=runsData
    with open(outputFileName,'w') as f:
        json.dump(toSerialize, f,indent=4)
def writeCsvFile(outputFileName, settingCollection, runDataObjects):
    with open(outputFileName, 'w') as f:
        f.write('{}\n'.format(';'.join(CsvData.headers())))
        for rd in runDataObjects:
            f.write('{}\n'/format(rd.getLine(';')))

def reconstructTimes(rfObj):
    """Reconstruct running times from the ResultsFolder messages
    """
    returnDict = {'totalTime':-1}
    typeMessages = rfObj.messagesByType()
    if not 'newbasis' in typeMessages:
        return returnDict
    # New style: sum times of all newbasis messages
    total = 0.0
    newBasisData = {}
    for m in typeMessages['newbasis']:
        if not m['step'] in newBasisData:
            newBasisData[m['step']] = {}
        newBasisData[m['step']][m['it']] = m['timeSpent']
    for k,v in newBasisData.items():
        returnDict[k] = sum(v.values())
        total += returnDict[k]
    if 'nnlsapplied' in typeMessages and not any([m['step'] == 'nnls' for m in typeMessages['newbasis']]):
        # Parse nnls from nnlsApplied messages
        returnDict['nnls'] = 0
        for m in typeMessages['nnlsapplied']:
            index = m['index']
            if index == 0:
                raise Exception('Invalid index for nnlsapplied message')
            prevTime = rfObj.messages[index-1]['logTime']
            ownTime = m['logTime']
            # Time is in minutes
            t = (ownTime - prevTime).total_seconds() / 60
            returnDict['nnls'] += t
    # Remove NNLS from total time
    for k,v in returnDict.items():
        if k == 'totalTime' or k == 'nnls':
            continue
        returnDict['totalTime'] += v
    return returnDict

def reconstructTimesPerIteration(rfObj):
    """Reconstruct running times from the ResultsFolder messages
    """
    returnList = {}
    typeMessages = rfObj.messagesByType()
    if not 'newbasis' in typeMessages:
        return returnList
    # New style: sum times of all newbasis messages
    total = 0.0
    
    newBasisData = {}
    # Collect data, potentially overwriting old data, as to get the latest
    for m in typeMessages['newbasis']:
        if not m['step'] in newBasisData:
            newBasisData[m['step']] = {}
        newBasisData[m['step']][m['it']] = m['timeSpent']
    for k,v in newBasisData.items():
        for k2,v2 in v.items():
            if not k2 in returnList:
                returnList[k2] = 0.0
            returnList[k2] += v2
            total += returnList[k2]
    if 'nnlsapplied' in typeMessages and not any([m['step'] == 'nnls' for m in typeMessages['newbasis']]):
        # Parse nnls from nnlsApplied messages
        #returnDict['nnls'] = 0
        for m in typeMessages['nnlsapplied']:
            index = m['index']
            if index == 0:
                raise Exception('Invalid index for nnlsapplied message')
            prevTime = rfObj.messages[index-1]['logTime']
            ownTime = m['logTime']
            # Time is in minutes
            t = (ownTime - prevTime).total_seconds() / 60
            returnList[m['it']] += t
    # Remove NNLS from total time
    return list(returnList.values())

def parseProgressOnly(args):
    r = SettingsCollectionParser(False)
    args = parser.parse_args()
    
    exp = Experiment()
    r.parseFolder(exp.settingsFolder())
    # Data to write
    writeData = []
    runs = {}
    for p in r.parentObjects:
        runs[p['name']] = {'finished':0, 'in-progress':0,'not-started':0, 'partial':0}
    currentlyRunning  = []
    els = r.getSettingObjects()
    current = 1
    for el in els:
        current += 1
        d = Decomposer('')
        d.setData(el)
        exists = True
        data = [d.parameterizedRunName()]
        if not d.resultFolderFromData().exists():
            exists = False
            data.append('non-existent')
            runs[el['name']]['not-started'] += 1
            dataElement.lastIt = -1
        if exists:
            # Setup results folder object
            rf = ResultsFolder(d.resultFolderFromData())
            rf.parseResults()
            rf.parseMessages()
            messagesByType = rf.messagesByType()
            
            if rf.isDone():
                data.append('done')
                runs[el['name']]['finished'] += 1
            elif d.checkIsOccupied():
                data.append('in-progress')
                runs[el['name']]['in-progress']+=1
                currentlyRunning.append(d.parameterizedRunName())
            elif rf.hasResults():
                data.append('partial')
                runs[el['name']]['partial'] += 1
            else:
                runs[el['name']]['not-started'] += 1
   # Parse the data and display progress in table 
    tableData= []
    headers = ['name','finished','in-progress','not-started','partial','completed']
    tableData.append(headers)
    total = {'name':'total','finished':0, 'in-progress':0,'not-started':0,'partial':0,'completed':0}
    for k,v in runs.items():
        completed= ''
        if v['not-started']==0 and v['in-progress']==0 and v['partial'] == 0:
            completed = 'y'
            total['completed'] += 1
        for k2 in total.keys():
            if k2 in v:
                total[k2] += v[k2]
        tableData.append(list(map(str,[k,v['finished'], v['in-progress'], v['not-started'],v['partial'],completed])))
    tableData.sort(key=lambda x:x[0])
    tableData.append(['-' for k in headers])
    tableData.append(list(map(str,[total[k] for k in headers])))
    print_table(tableData)
    # Print actively running elements
    if len(currentlyRunning) > 0:
        print('Running:')
        print('\n'.join(currentlyRunning))
    else:
        print('Nothing currently running')

distortionFile = 'distortion.json'
reverseDistortionFile = 'distortion_reverse.json'

if __name__ == '__main__':
    parser = ArgumentParser(description='Parse results in subfolders of the result folder')

    parser.add_argument('-o','--outputFile',type=str,help='Ouput file. When omitted, prints to screen. Specify relative to root directory of project')
    parser.add_argument('-p','--progressOnly',action='store_true', help='Only show progress table')
    parser.add_argument('-r','--progressPoll',type=int, help='Continuously poll for progress every specified number of seconds, interrupt to quit')
    r = SettingsCollectionParser(False)
    args = parser.parse_args()

    if args.progressOnly:
        parseProgressOnly(args)
        exit()
    if args.progressPoll is not None:
        while True:
            parseProgressOnly(args)
            time.sleep(args.progressPoll)
        exit()

    # Check output file writability first
    if args.outputFile is not None:
        # Overwrite this later
        with open(args.outputFile,'w') as f:
            f.write('test')
    
    exp = Experiment()
    r.parseFolder(exp.settingsFolder())
    # Data to write
    writeData = []
    runs = {}
    for p in r.parentObjects:
        runs[p['name']] = {'finished':0, 'in-progress':0,'not-started':0, 'partial':0}
    currentlyRunning  = []
    els = r.getSettingObjects()
    current = 1
    for el in els:
        print('At {}/{}'.format(current,len(els)))
        current += 1
        d = Decomposer('')
        d.setData(el)
        exists = True
        data = [d.parameterizedRunName()]
        dataElement = CsvData()
        dataElement.name = d.runName
        dataElement.args = d.parameterizedRunName()
        if not d.resultFolderFromData().exists():
            exists = False
            data.append('non-existent')
            runs[el['name']]['not-started'] += 1
            dataElement.lastIt = -1
        if exists:
            dataElement.runData = el['currentParameters']
            # Setup results folder object
            rf = ResultsFolder(d.resultFolderFromData())
            rf.parseResults()
            rf.parseMessages()
            if rf.hasResults():
                dataElement.lastIt = rf.results[-1]['it']
                dataElement.lastStep = rf.results[-1]['step']
            if rf.isDone():
                dataElement.runTime = reconstructTimes(rf)
            else:
                dataElement.runTime={'totalTime':-1}
            messagesByType = rf.messagesByType()
            if 'basissize' in messagesByType and rf.isDone():
                # Read the result
                pi = ProblemInstance()
                dr = DecompositionResult(pi)
                dr.read(str(rf.folder / rf.results[-1]['file']))
                dataElement.basisSize = dr.basisSize()
            else:
                dataElement.basisSize = -1
            if rf.hasMessages():
                lastMsg = rf.messages[-1]
                startMsg = None
                for i in range(len(rf.messages)-2,-1,-1):
                    if rf.messages[i]['type'] == 'initnorm':
                        startMsg = rf.messages[i]
                if startMsg is not None:
                    # Parse time diff and save
                    startDate = startMsg['logTime']
                    endDate = lastMsg['logTime']
                    diff = endDate - startDate
                    dataElement.totalTime = diff.total_seconds() / 60.0

            # Check that this is for the last element!
            if len(rf.nnlsValues) > 0:
                dataElement.objectiveVal = rf.nnlsValues[-1]
            if rf.isDone():
                data.append('done')
                runs[el['name']]['finished'] += 1
                # Compute stats per iteration if necessary
                if 'perIterationStats' in el and el['perIterationStats']:
                    perItResults = rf.resultPerIteration()
                    for k in perItResults.keys():
                        dataElement.perIteration[k] = {}
                    for k,v in perItResults.items():
                        # Read the last result for the iteration
                        pi = ProblemInstance()
                        dr = DecompositionResult(pi)
                        dr.read(str(rf.folder / v[-1]['file']))
                        dataElement.perIteration[k]['basisSize'] = dr.basisSize()
                        dataElement.perIteration[k]['deviation'] = dr.objectiveValue()
                        # Compute time of iteration

                        # Read distortion if available
                        if rf.hasFile('distortion.json'+str(k)):
                            with open(rf.folder / 'distortion.json{}'.format(k)) as f:
                                data = json.load(f)
                                dataElement.perIteration[k]['distortion'] = data['mean']
                                dataElement.perIteration[k]['distortionData'] = data
                        if rf.hasFile(reverseDistortionFile+str(k)):
                            reverseDistortionData = None
                            with open(rf.folder / (reverseDistortionFile+str(k))) as f:
                                reverseDistortionData = json.load(f)
                            dataElement.perIteration[k]['reverseDistortion'] = data['mean']
                            dataElement.perIteration[k]['reverseDistortionData'] = data
                            # Compute weighted mean. For this, we need the top something elements
                            basisSize = reverseDistortionData['basisSize']
                            # Read the file containing the basis and get the top
                            piDecRes = ProblemInstance()
                            decRes = DecompositionResult(piDecRes)
                            decRes.read(str(rf.folder / reverseDistortionData['file']))
                            decRes.pruneToTopWeighted(basisSize)
                            weights = [decRes.coefficient(i) for i in range(basisSize)]
                            vals = reverseDistortionData['values']
                            dataElement.perIteration[k]['reverseDistortionWeighted'] =  sum([vals[i] * weights[i] for i in range(basisSize)])/sum(weights)
                            # Also compute total weight in this reduced basis
                            dataElement.perIteration[k]['totalWeightReducedBasis'] = sum([len(decRes.basisElement(i))*decRes.coefficient(i) for i in range(basisSize)])
                    perItTimes = reconstructTimesPerIteration(rf)
                    for i in range(len(perItTimes)):
                        dataElement.perIteration[i]['time'] = perItTimes[i]
            elif d.checkIsOccupied():
                data.append('in-progress')
                runs[el['name']]['in-progress']+=1
                currentlyRunning.append(d.parameterizedRunName())
            elif rf.hasResults():
                data.append('partial')
                runs[el['name']]['partial'] += 1
            else:
                runs[el['name']]['not-started'] += 1
            # Parse per iteration results
        if (d.resultFolderFromData() / 'distortion.json').exists():
            distortionData= None
            with open(d.resultFolderFromData() / 'distortion.json') as f:
                distortionData = json.load(f)
            dataElement.distortion = distortionData['mean']
        if (d.resultFolderFromData() / 'distortion_reverse.json').exists():
            reverseDistortionData = None
            with open(d.resultFolderFromData() / 'distortion_reverse.json') as f:
                reverseDistortionData = json.load(f)
            dataElement.reverseDistortionData = reverseDistortionData
            dataElement.reverseDistortion = reverseDistortionData['mean']
            # Compute weighted mean. For this, we need the top something elements
            k = reverseDistortionData['basisSize']
            # Read the file containing the basis and get the top
            piDecRes = ProblemInstance()
            decRes = DecompositionResult(piDecRes)
            decRes.read(str(rf.folder / reverseDistortionData['file']))
            decRes.pruneToTopWeighted(k)
            weights = [decRes.coefficient(i) for i in range(k)]
            vals = reverseDistortionData['values']
            dataElement.reverseDistortionWeighted =  sum([vals[i] * weights[i] for i in range(k)])/sum(weights)
            # Also compute total weight in this reduced basis
            dataElement.totalWeightReducedBasis = sum([len(decRes.basisElement(i))*decRes.coefficient(i) for i in range(k)])
            
        writeData.append(dataElement)
   # Parse the data and display progress in table 
    tableData= []
    headers = ['name','finished','in-progress','not-started','partial','completed']
    tableData.append(headers)
    total = {'name':'total','finished':0, 'in-progress':0,'not-started':0,'partial':0,'completed':0}
    for k,v in runs.items():
        completed= ''
        if v['not-started']==0 and v['in-progress']==0 and v['partial'] == 0:
            completed = 'y'
            total['completed'] += 1
        for k2 in total.keys():
            if k2 in v:
                total[k2] += v[k2]
        tableData.append(list(map(str,[k,v['finished'], v['in-progress'], v['not-started'],v['partial'],completed])))
    tableData.sort(key=lambda x:x[0])
    tableData.append(['-' for k in headers])
    tableData.append(list(map(str,[total[k] for k in headers])))
    print_table(tableData)
    # Print actively running elements
    if len(currentlyRunning) > 0:
        print('Running:')
        print('\n'.join(currentlyRunning))
    else:
        print('Nothing currently running')
    if args.outputFile is not None:
        if args.outputFile.endswith('.csv'):
            writeCsvFile(args.outputFile, r, writeData)
        elif args.outputFile.endswith('.json'):
            writeJsonFile(args.outputFile, r, writeData)
        else:
            raise Exception('Unsupported file extension for {}, only accept csv and json'.format(args.outputFile))
