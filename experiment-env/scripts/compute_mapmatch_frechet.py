from LoopsHelpers import Experiment
from PyLoops import Graph, GraphTrajectorySet, TimestampedTrajectorySet
from LoopsHelpers.deepsetting import getDeep
import json
import os

e = Experiment()
g = Graph()

data = None
with open(e.settingsFolder() / 'rerun_thr-rw-sec5.1.json') as f:
    data = json.load(f)

mapFile = getDeep('problemInstance.map',data)

mmTrajectories = 'dutchu-hague-hitting-square-10000_properid_split_filtered_mm.boosttxt'
regTrajectories = 'dutchu-hague-hitting-square-10000_properid_split_filtered_bbox.wktcsv?idCol=0&trajectoryCol=1'

g.read(str(e.mapsFolder() / mapFile))
gts = GraphTrajectorySet()
gts.read(str(e.trajectoriesFolder() / mmTrajectories))

# Convert to trajectory set
gtset = gts.convertToTrajectorySet(g)

tts = TimestampedTrajectorySet()
tts.read(str(e.trajectoriesFolder() / regTrajectories))
ttsSet = tts.convertToTrajectorySet()
# Get to correct CRS
ttsSet.convertToCRSOfOther(gtset)

outFile = e.baseFolder() / 'data-processing' / 'mm-frechets.json'
os.makedirs(e.baseFolder() / 'data-processing', exist_ok=True)

# Compute frechets
tIdsMap = {}
tIds = gts.trajectoryIds
for i in range(len(tIds)):
    tIdsMap[tIds[i]] = i

compareTIds = ttsSet.trajectoryIds

frechetVals = []
compareIds = []
totalLen = len(compareTIds)
computed = 0
minVal = 500000
maxVal = 0
for i in range(totalLen):
    trajId = compareTIds[i]
    val = -1
    if trajId in tIdsMap:
        compareIds.append(trajId)
        val = ttsSet.computeFrechetToOther(i,gtset,tIdsMap[trajId])
        minVal = min(minVal,val)
        maxVal = max(maxVal, val)
        frechetVals.append(val)
        computed += 1
    print('Done with {}/{}, actual computed: {}, val = {}, min-max seen: {},{}'.format(i+1, totalLen, computed,val,minVal,maxVal))
outData = {'mmFile':mmTrajectories, 'trajFile':regTrajectories,'vals':frechetVals, 'trajIds':compareIds}
with open(outFile,'w') as f:
    json.dump(outData, f)