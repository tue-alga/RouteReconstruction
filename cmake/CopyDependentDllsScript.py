import shutil
from pathlib import Path
import subprocess
import os
import sys
import codecs
from argparse import ArgumentParser

def removeQuotes(strEl):
    if strEl[0] == '"' and strEl[-1] == '"':
        return strEl[1:-1]
    return strEl

def copy_if_newer(srcPath, targetPath):
    didCopy = False
    if not targetPath.exists() or os.stat(srcPath).st_mtime - os.stat(targetPath).st_mtime > 1:
        shutil.copy2 (srcPath, targetPath)
        didCopy = True
    return didCopy


if __name__ == "__main__":
    parser = ArgumentParser(description='Copy dependent DLLs for an executable or dll library. Run in the binary directory of your CMake project')
    parser.add_argument('targetName',help='Name of the CMake target',type=str)
    parser.add_argument('buildTypeName',help='Name of the CMake build type, i.e. Debug,RelWithDebInfo etc.',type=str)
    parser.add_argument('binaryName',help='Name of the binary. When no extension is provided, attaches .exe',type=str)
    print("Running CopyDependentDllsScript.py...")

    args=  parser.parse_args()

    # Parse args
    workingDirectory = os.getcwd()
    targetName = args.targetName
    buildType = args.buildTypeName
    binaryName = args.binaryName
    if binaryName.find('.') == -1:
        binaryName += '.exe'
    # The file we are looking for
    targetFile = 'link.command.1.tlog'

    # Log info
    print("Arguments: targetName = {}, buildType = {}, binaryName = {}".format(targetName, buildType,binaryName))
    print("Running in directory {}".format(workingDirectory))

    # Set the path to the expected binary 
    binaryDir = Path(workingDirectory) / buildType
    pathToBinary = binaryDir / binaryName

    pathToTLog = Path(workingDirectory) / '{}.dir'.format(targetName) / buildType / '{}.tlog'.format(targetName)
    if not pathToTLog.exists():
        raise Exception("Could not find tlog directory. Is the project already built?")
    
    # Get the command file
    targetFilePath = pathToTLog / targetFile
    if not targetFilePath.exists():
        raise Exception("Could not find linker command log. Is the project already built?")
    
    # Find the appropriate line with the linker info
    print('Reading {}'.format(targetFilePath))
    targetLine = None
    with codecs.open(str(targetFilePath),'r',encoding='utf-16-le') as f:
        lineNum = 0
        for line in f:
            line = line.strip()
            if line.startswith('/OUT'):
                targetLine = line
                break
            lineNum += 1
    if targetLine is None:
        raise Exception("Could not find /OUT command in linker log. Is the project already built?")

    # Parse line to get linked .lib files. They may suggest where to find the dlls

    lineParts = targetLine.split(' ')
    libs = [Path(removeQuotes(part.strip().lower())) for part in lineParts 
        if (part.strip().lower().endswith(".lib") or part.strip().lower().endswith('.lib"')) and not part.strip().startswith('/')
    ]
    print(libs)

    libNames = [os.path.basename(str(libPath)).lower().replace('.lib','') for libPath in libs]
    libMap = dict(zip(libNames,libs))
    print('Read .lib\'s:{}'.format(','.join(libNames)))
    
    # Find all referenced dlls via the 'dumpbin' command
    # TODO: detect dumpbin exists
    out = subprocess.run(['dumpbin','/DEPENDENTS','{}'.format(pathToBinary)],universal_newlines=True,stdout=subprocess.PIPE)
    parts = out.stdout.split('\n')
    dependentDlls = []
    for line in parts:
        line = line.strip()
        if line.endswith('.dll'):
            dependentDlls.append(line.replace('.dll',''))
    print('To resolve: {}'.format(','.join(dependentDlls)))

    for dllEl in dependentDlls:
        print('# {}.dll'.format(dllEl))
        if not dllEl.lower() in libNames:
            print('\tdll {}.dll does not have an associated .lib linked'.format(dllEl))
            continue
        targetDllPath = binaryDir / '{}.dll'.format(dllEl)

        consideredPaths = []

        # Try to find the relevant dll
        assocLib = libMap[dllEl.lower()]
        libPath = None
        if not assocLib.is_absolute():
            libPath = (workingDirectory / assocLib).resolve().parent
        else:
            libPath = assocLib.resolve().parent
        
        consideredDllPath = libPath / '{}.dll'.format(dllEl)
        if consideredDllPath.exists():
            if copy_if_newer(consideredDllPath, targetDllPath):
                print('\tCopied {} to {}'.format(consideredDllPath, targetDllPath))
            else:
                print('\tUp-to-date (from: {},to: {})'.format(consideredDllPath, targetDllPath))
            # Try copy pdb
            consideredPdbPath = libPath / '{}.pdb'.format(dllEl)
            targetPdbPath = binaryDir / '{}.pdb'.format(dllEl)
            if consideredPdbPath.exists():
                if copy_if_newer(consideredPdbPath, targetPdbPath):
                    print('\tCopied .pdb ({} to {})'.format(consideredPdbPath, targetPdbPath))
                else:
                    print('\tPdb was up to date')

            #Skip rest of iteration
            continue
        
        consideredPaths.append(libPath)
        
        if libPath.name.lower() == 'lib' or libPath.name.lower() == 'libs':
            libPath = libPath / "../bin"
            consideredDllPath = libPath / '{}.dll'.format(dllEl)
            if consideredDllPath.exists():
                if copy_if_newer(consideredDllPath, targetDllPath):
                    shutil.copy2 (consideredDllPath, targetDllPath)
                    print('\tCopied {} to {}'.format(consideredDllPath, targetDllPath))
                else:
                    print('\tUp-to-date (from: {},to: {})'.format(consideredDllPath, targetDllPath))
                continue
            else:
                consideredPaths.append(libPath)
        print('\tCould not find {}.dll'.format(dllEl))
        print('\tConsidered paths:\n\t\t{}'.format('\n\t\t'.join(map(str,consideredPaths))))
        




