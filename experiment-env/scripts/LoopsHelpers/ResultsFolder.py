from .Experiment import Experiment
from pathlib import Path
from datetime import datetime
from .Messages import Messages
import re
import json
import os

class ResultsFolder:
    def __init__(self, folder):
        self.folder = Path(folder)
        self.exp = Experiment()
        self.data = {}
        self.bestNnls = -1
        self.bestNnlsAt = -1
        self.nnlsValues = []
        # Messages
        self.messages = []
        # Result files
        self.results = []
        if (Path(folder) / 'settings.json').exists():
            with open(Path(folder) / 'settings.json') as f:
                self.data = json.load(f)

    def hasMessages(self):
        return len(self.messages) > 0
    def hasResults(self):
        return len(self.results) > 0

    @staticmethod
    def resultFolderFromData(data, baseFolder):
        parKeys = sorted(list(data['runParameters'].keys()))
        parts = []
        folder = baseFolder / data['name']
        for key in parKeys:
            val = key + '_' + str(data['currentParameters'][key])
            folder = folder / val
        return ResultsFolder(folder)

    @staticmethod
    def findResultFolders(rootFolder):
        resultFolders = []
        for root, directories, files in os.walk(rootFolder):
            if (Path(root) / 'settings.json').exists():
                rf = ResultsFolder(root)
                rf.parseResults()
                rf.parseMessages()
                resultFolders.append(rf)
        return resultFolders
    def isDone(self):
        if not 'problemInstance' in self.data:
            return False
        its = self.data['problemInstance']['iterations']
        steps = len(self.data['algs'].keys()) + 1
        if len(self.results) == 0:
            return False
        return self.results[-1]['it'] == its-1 and self.results[-1]['step'] == steps-1
    def messagesByType(self):
        typedMessages = {}
        for m in self.messages:
            if not m['type'] in typedMessages:
                typedMessages[m['type']] = []
            typedMessages[m['type']].append(m)
        for k in typedMessages.keys():
            typedMessages[k].sort(key=lambda el: el['logTime'])
        return typedMessages
    def resultPerIteration(self):
        toReturn = {}
        for r in self.results:
            if not r['it'] in toReturn:
                toReturn[r['it']] = []
            toReturn[r['it']].append(r)
        for k,v in toReturn.items():
            v.sort(key=lambda el: el['step'])
        return toReturn
    def hasFile(self, fileName):
        return (self.folder / fileName).exists()

    def parseMessages(self):
        """
        Parse the messages in the progress.txt file

        """
        if not (self.folder / 'progress.txt').exists():
            return
        self.messages = []
        self.nnlsValues = []
        index = 0
        with open(self.folder / 'progress.txt') as f:
            for line in f:
                m = re.search('\[.*\]\s*:?',line)
                msgData = line[m.end():]
                msg = json.loads(msgData)
                # Parse to datetime object
                msg['logTime'] = datetime.strptime(msg['logTime'], '%Y-%m-%d %H:%M:%S')
                self.messages.append(msg)
                if msg['type'] == Messages.MSG_NNLSAPPLIED:
                    self.nnlsValues.append(msg['objectiveValue'])
                    if msg['objectiveValue'] < self.bestNnls or self.bestNnls < 0:
                        self.bestNnls = msg['objectiveValue']
                        self.bestNnlsAt = msg['it']
                msg['index']=index
                index += 1
    def nnlsDiffs(self):
        diffs = []
        for i in range(len(self.nnlsValues)-1):
            diffs.append(self.nnlsValues[i+1] - self.nnlsValues[i])
        return diffs
    def deduplicateResults(self, dryRun=False):
        if len(self.results) == 0:
            self.parseResults()
        if len(self.results) == 0:
            return
        seenEls = {}
        for i in range(len(self.results)):
            it = self.results[i]['it']
            step = self.results[i]['step']
            if (it,step) in seenEls:
                # Need to delete one of the two
                otherI = seenEls[(it,step)]
                toDelete = i
                if self.results[i]['mtime'] > self.results[otherI]['mtime']:
                    toDelete = otherI
                if dryRun:
                    print('Duplicate to be deleted: {}'.format(self.results[toDelete]['file']))
                else:
                    os.remove(Path(self.folder) / self.results[toDelete]['file'])
            else:
                seenEls[(it,step)] = i
            pass
        # Update the results
        self.parseResults()
    def parseResults(self):
        self.results = []
        for el in os.listdir(self.folder):
            m = re.match('it\=([0-9]+)\-([0-9]+)_.*\.boosttxt',el)
            if m is not None:
                result = {'it':int(m.group(1)), 'step':int(m.group(2)),'file':m.group(0),'mtime':os.path.getmtime(Path(self.folder) / el)}
                self.results.append(result)
        self.results.sort(key=lambda el: el['it']*1000+el['step'])

    def loggedTotalTime(self):
        if len(self.messages) == 0:
            return 0
        return (self.messages[-1]['logTime'] - self.messages[0]['logTime']).total_seconds()

    def lastSeenIt(self):
        if self.hasResults():
            return self.results[-1]['it']
        return -1
        
