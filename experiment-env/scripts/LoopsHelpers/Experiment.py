from pathlib import Path
import os
import sys
from datetime import datetime

# Place pyloops in 'bin' folder in parent folder of the folder of this script!
pyloopsDir = str((Path(__file__).parent.resolve().parent / 'bin').resolve())
os.environ['PATH'] = pyloopsDir + os.pathsep + os.environ['PATH']
sys.path.insert(0,pyloopsDir)
import PyLoops
from PyLoops import Graph, FlowField


class Experiment:

    def __init__(self,key='LOOPS_DIR'):
        if not key in os.environ:
            if sys.platform.startswith('win'):
                os.environ[key] = 'D:/Repos/LoopsScripts'
            else:
                print('Set key ' + key + ' in your environment to the experiments folder')
                raise Exception("Error with environment path key")
        
        self._baseFolder = Path(os.environ[key])

        # Save the available decomposers
        self.decomposers = PyLoops.AllDecomposers
        # Get the class objects
        self.decomposerClasses = [getattr(PyLoops,d) for d in self.decomposers]

    def createDecomposer(self, name):
        if not name in self.decomposers:
            available = ','.join(self.decomposers)
            raise Exception('Unknown decomposer: {}. Available:{}'.format(name, available))
        # Get index of decomposer
        i  = self.decomposers.index(name)
        # Construct with empty result
        return self.decomposerClasses[i](None)

    def baseFolder(self):
        return self._baseFolder

    def resultsFolder(self):
        return self._baseFolder / 'results'

    def fieldsFolder(self):
        return self._baseFolder / 'fields'

    def mmTrajectoriesFolder(self):
        return self._baseFolder / 'mm-trajectories'

    def trajectoriesFolder(self):
        return self._baseFolder / 'trajectories'

    def dataProcessingFolder(self):
        return self._baseFolder / 'data-processing'

    def logsFolder(self):
        return self._baseFolder / 'logs'

    def scriptsFolder(self):
        return self._baseFolder / 'scripts'
        
    def settingsFolder(self):
        return self._baseFolder / 'settings'

    def mapsFolder(self):
        return self._baseFolder / 'maps'

    def _tryResolve(self, filePath, folder):
        if (folder / filePath).exists():
            return folder / filePath
        tryOut = os.path.baseName(filePath)
        if (folder / tryOut).exists():
            return folder / tryOut
        raise Exception("Could not reoslve path {}".format(filePath))

    def readMap(self, mapFileName):
        path = self._tryResolve(mapFileName, self.mapsFolder())
        g = Graph()
        g.read(str(path))
        return g
    
    def readFlowField(self, flowFieldName):
        path = self._tryResolve(flowFieldName, self.fieldsFolder())
        field = FlowField()
        field.read(str(path))
        return field

    def createResultFolder(self, folderName):
        os.makedirs(str(self.resultsFolder() / folderName), exist_ok=True)

    def writeStart(self, folder,prefix=''):
        date_time = datetime.now().strftime("{}start=%Y-%m-%d_%H-%M-%S".format(prefix))
        print("date and time:",date_time)	
        with open(folder /date_time,'w') as f:
            f.write('Start at {}'.format(date_time))
            
    def run(self):
        """To be implemented
        """
        pass
