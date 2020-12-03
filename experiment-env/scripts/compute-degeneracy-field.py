from pathlib import Path
from helpers import Experiment
import sys

#print(os.environ['LOOPS_DIR'])
pyloopsDir = '/data/bram/loops/bin'
sys.path.insert(0,pyloopsDir)

from PyLoops import FlowField, Graph
# Yada yada, get all paths
exp = Experiment()

# Read the given input field
fieldFile = sys.argv[1]

ff = FlowField()

g = Graph()
g.read(str(exp.mapsFolder() / 'theHagueRotterdam.osm'))

# Flow field
ff.read(str(exp.fieldsFolder() / fieldFile))
print('Path count:{}'.format(ff.pathCount()))


totalEdgesCount = dict()
total = 0
for p in range(ff.pathCount()):
    # Get unique edges
    edges = set()
    path = ff.path(p)
    for e in path:
        edges.add(e)
    for e in edges:
        eId = int(e)
        if not eId in totalEdgesCount:
            totalEdgesCount[eId] = 1
        else:
            totalEdgesCount[eId] += 1
        total += 1
print('Total edges seen (deduped){}'.format(total))
print('Average edge coverage {}'.format(total / len(totalEdgesCount.keys())))
print('Min :{}'.format(min(totalEdgesCount.values())))
print('Max :{}'.format(max(totalEdgesCount.values())))
# maxNum = max(els.values())
# minNum = min(els.values())
# meanNum = totalOccurences / totalEdges
