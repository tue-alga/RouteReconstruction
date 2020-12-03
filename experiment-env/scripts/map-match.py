import sys
import os

#print(os.environ['LOOPS_DIR'])
pyloopsDir = '/data/bram/loops/bin'
sys.path.insert(0,pyloopsDir)
from LoopsHelpers import Experiment
from PyLoops import Graph, FastMapMatching, TimestampedTrajectorySet
from pathlib import Path
from argparse import ArgumentParser
# Enable matplotlib inline
import random
import math
print(dir(Graph))
print(dir(FastMapMatching))

if __name__ == "__main__":
    parser = ArgumentParser(description='Mapmatch a file contaning trajectories')
    parser.add_argument('inputFile', type=str, help='The input csv-like file containing trajectories. Either .wkbcsv or .wktcsv')
    parser.add_argument('outputFile',type=str, help='Output file for storing the mapmatched result')
    parser.add_argument('graphFile',type=str, help='File containing the graph to match to')
    parser.add_argument('-k',type=int, default=10, help='Number of closest edges to consider for candidate generation')
    parser.add_argument('-e','--gpsError',type=float, default=50, help='Expected gps error in local coordinate system')
    parser.add_argument('-b','--speedBound',type=float, default= 140. / 3.6, help='Speed bound for searching paths in mapmatching')
    parser.add_argument('-r','--radius',type=float, default=300, help='Radius to look for closest edges for candidate generation')
    parser.add_argument('-t','--numThreads',type=int, default=1, 
        help='Number of threads to use in computation. More than 1 thread will require merging resulting files afterwards')
    parser.add_argument('-c','--count',type=int,default=-1, help='Number of trajectories from start to mapmatch, -1 for all')
    parser.add_argument('-s','--withStats',action='store_true', help='Save stats of map-matching to <outputFile>.stats.csv in csv format ')

    args = parser.parse_args()

    e = Experiment()
    # Read the trajectories
    inputFilePath = str(e.trajectoriesFolder() / args.inputFile)

    # Read the map
    g = Graph()
    g.read(str(e.mapsFolder() / args.graphFile))

    trajSet = TimestampedTrajectorySet()
    trajSet.read(inputFilePath)

    # Setup the mapmatcher
    mapMatcher = FastMapMatching(g)
    mapMatcher.k = args.k
    mapMatcher.gpsError = args.gpsError
    mapMatcher.candidateRadius = args.radius
    mapMatcher.speedBound = args.speedBound # m/s

    if args.withStats:
        (resultMapmatched, trajIds, offsets, sizes,inputSizes) = mapMatcher.mapMatchSetWithStats(trajSet,args.count)
        resultMapmatched.write(str(e.trajectoriesFolder() / args.outputFile))
        with open(e.trajectoriesFolder() / '{}.stats.csv'.format(args.outputFile),'w') as f:
            f.write('ID;Offset;Size;InputSize\n')
            for i in range(len(trajIds)):
                f.write('{};{};{};{}\n'.format(trajIds[i], offsets[i], sizes[i],inputSizes[i]))
    elif args.numThreads <= 1:
        resultMapmatched = mapMatcher.mapMatchSet(trajSet,args.count)
        resultMapmatched.verifyPaths(g)
        # Write to file
        resultMapmatched.write(str(e.trajectoriesFolder() / args.outputFile))
    else:
        mapMatcher.mapMatchSetConcurrent(trajSet, args.numThreads, str(e.trajectoriesFolder() / args.outputFile))
