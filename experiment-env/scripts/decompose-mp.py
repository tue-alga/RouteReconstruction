from LoopsHelpers import Decomposer
from LoopsHelpers import Experiment
import sys
import os
from multiprocessing import Pool
from argparse import ArgumentParser
import json
import copy
import itertools
from LoopsHelpers.deepsetting import setDeep
from LoopsHelpers import SettingsCollectionParser

class DecomposerRunner:
    def __init__(self, extraArgs={}):
        self.extraArgs = extraArgs
    def __call__(self, settingsObjDict):
        dec = Decomposer(str(Experiment().logsFolder() / 'runs.log'))
        dec.loadSettingsFromObj(settingsObjDict)
        dec.overwriteExisting = settingsObjDict['overwriteExisting']
        dec.decompose(self.extraArgs)

def runDecomposer(settingsObj,extraArgs):
    dec = Decomposer(str(Experiment().logsFolder() / 'runs.log'))
    dec.loadSettingsFromObj(settingsObj)
    dec.overwriteExisting = settingsObj['overwriteExisting']
    dec.decompose(extraArgs)

if __name__ == "__main__":
    # CLI arguments
    parser = ArgumentParser(description='Decompose a field with multiple settings, using Python multiprocessing')
    parser.add_argument('settingsText', type=str, help='.txt file containing lines of .json files to run, or a directory with .json files when running in folder mode')
    parser.add_argument('poolSize', type=int, help='Size of the pool to use. Specify <= 1 for non-mp sequential run')
    parser.add_argument('-c','--checkexistence', help='Checks existence of run, continues if incomplete',action='store_true')
    parser.add_argument('-w','--writesettings', help='Write the settings to the result folder of the run',action='store_true')
    parser.add_argument('-o','--overwrite', help='Overwrite previous results',action='store_true')
    parser.add_argument('-f','--parsefolder', help='Parse folder?',action='store_true')
    parser.add_argument('-d','--deduplicate',help='Deduplicate results in folder based on mtime', action='store_true')
    parser.add_argument('-s','--startFrom',help='Start from the given iteration and step, specified as <it>,<step>. Loads result of (it,step) and starts computing (it,step+1) etc',type=str)

    args = parser.parse_args()

    r = SettingsCollectionParser(args.overwrite)
    dr = DecomposerRunner()
    if args.parsefolder:
        r.parseFolder(args.settingsText)
    else:
        r.parseSettingsTxt(Experiment().settingsFolder() / args.settingsText)
    if args.startFrom is not None:
        parts = list(map(int, args.startFrom.split(',')))
        dr.extraArgs = {'redoPartial':{'i':parts[0],'j':parts[1]}}
    if args.checkexistence:
        r.checkExistence()
    elif args.deduplicate:
        for so in r.getSettingObjects():
            d = Decomposer('')
            d.setData(so)
            d.resultsFolder.deduplicateResults()
    elif args.writesettings:
        r.writeSettingFiles('dump')
    else:
        settingObjs = r.getSettingObjects()
        print('[decompose-mp.py] Running {} tasks on pool of size {}'.format(len(settingObjs), args.poolSize))
        if args.poolSize <= 1:
            print('decompose-mp.py] Running sequential')
            # Run sequential
            for so in settingObjs:
                dr(so)
        else:
            # This had a reason, but don't know why anymore
            pool = Pool(min(args.poolSize,len(settingObjs)))
            with Pool(min(args.poolSize,len(settingObjs))) as pool:
                pool.map(dr, settingObjs)

    print('[decompose-mp.py] Done processing')
