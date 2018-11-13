#!/usr/bin/env python
## @package summarize
## Analyze the results produced by a given test
#
#  The name of the test is specified on the command line. It looks for
#  a directory with that name inside the runDir directory. The test results
#  are assumed to be a python module, so this script just loads that module
#  and calls addLogs. This creates a top level dictionary with all the log
#  results.
#
#  It then calls an analysis function, determined by the test name, to
#  loop over those test results and generate the relevant statistics. The
#  stats are stored to a `log.csv` file, and an overall summary is printed
#  to stdout
import pkgutil
import sys

import argparse
parser = argparse.ArgumentParser(prog='summarize.py')
parser.add_argument('-d', 
                    help='Directory for test results. Required.', 
                    dest='runDir')
parser.add_argument('-t', 
                    help='Test name to summarize', 
                    dest='testToSummarize')
args = parser.parse_args()

## The LogsDict class is used to store test results for analysis.
#
#  The data structure for log records is a dict of dicts:
#
#
#       [statNameA] -> [1: 100]
#               [2: 200]
#               ...
#       [statNameB] -> [1: 1000]
#               [2: 5000]
#               ...
#
class LogsDict(object):
    def __init__(self):
        self.logSets = {}

    def addLogSet(self, name):
        self.logSets[name] = {}

    def getLogSet(self, name):
        """If a log set with that name exists, return it. Else create a
           new one, insert it, and then return it"""
        l = self.logSets.get(name)
        if not l:
            self.addLogSet(name)
            return self.logSets.get(name)
        # else
        return l

    def printLogSet(self, name):
        """Print the values in a given log set"""
        if name in self.logSets:
            vals = self.getLogSet(name)
            for l in vals.keys():
                print(str(vals[l]))

    def printLogs(self):
        """Go through each log set and print it"""
        for k in self.logSets.keys():
            print("Key: " + k)
            self.printLogSet(k)
    
    def addToLogSet(self, name, id, val):
        self.getLogSet(name)[id] = val


def calcTimeDiff(dict, name1, name2):
    """Simple analysis function to calculate time difference between
       two named log sets, and their average"""
    timeDiffs = []
    vals1 = dict.getLogSet(name1)
    vals2 = dict.getLogSet(name2)

    # Write the raw data to a CSV file, in addition to analyzing it
    f = open( args.runDir + '/log.csv', 'a')

    # We might lose messages. Take care of that case by only using
    # sequence ids we find in the receiver, and skipping the rest
    missingVals = []

    # Now go through and simply calculate the difference between the two
    # sets of values
    for v in vals1.keys():
        if v in vals2:
            td = vals2[v] - vals1[v]
            if td <= 0:
                sys.stderr.write("Skipping invalid value [" + str(v) + "]: " + str(td) + " = " + str(vals2[v]) + " - " + str(vals1[v]))
            else:
                timeDiffs.append(td)
                f.write(str(v) + ', ' + str(td) + '\n')
        else:
            missingVals.append(v)
            
    x = len(missingVals)
    if x:
        sys.stderr.write(str(x) + " values in " + name1 + " missing from " + name2 + ": " + str(missingVals))

    f.close()
    # This is the actual analysis
    sys.stdout.write(str((reduce(lambda x,y: x+y, timeDiffs)/len(timeDiffs))/1000000.0) +
                         "," + str(min(timeDiffs)/1000000.0)+
                         "," + str(max(timeDiffs)/1000000.0) + "\n")

def summarizeIonBufferStats(dict):
    """Calculate a variety of stats for this case"""
    timesALLOC = []
    timesSHARE = []
    timesMMAP = []
    timesXmit = []
    timesBinder = []
    timesIMPORT = []
    timesMMAPClient = []

    # Write the raw data to a CSV file, in addition to analyzing it
    f = open( args.runDir + '/log.csv', 'a')

    # We might lose messages. Take care of that case by only using
    # sequence ids we find in the receiver, and skipping the rest
    missingVals = []

    f.write ("Sequence, ION_ALLOC, ION_SHARE, mmap-server, binder call, ION_IMPORT, mmap, Rx-Tx, Total Time\n")

    # Now go through and calculate the status
    for v in dict.getLogSet("ION_ALLOC_Pre").keys():
        if v not in dict.getLogSet("ClientCallbackRx").keys():
            missingVals.append(v)
            continue
        v1 = dict.getLogSet("ION_ALLOC_Pre")[v]
        v2 = dict.getLogSet("ION_ALLOC_Done")[v]
        v3 = dict.getLogSet("ION_SHARE_Done")[v]
        v4 = dict.getLogSet("ION_MMAP_Done")[v]
        v5 = dict.getLogSet("ServerMsgTx")[v]
        v6 = dict.getLogSet("ION_Binder_Pre")[v]
        v7 = dict.getLogSet("ION_Binder_Done")[v]
        v8 = dict.getLogSet("ION_IMPORT_Done")[v]
        v9 = dict.getLogSet("ION_MMAP_Client_Done")[v]
        v10 =dict.getLogSet("ClientCallbackRx")[v]

        timesALLOC.append(v2-v1)
        timesSHARE.append(v3-v2)
        timesMMAP.append(v4-v3)
        timesBinder.append(v7 - v6)
        timesIMPORT.append(v8 - v7)
        timesMMAPClient.append(v9 - v8)
        timesXmit.append(v10 -v5)

        f.write(str(v) + ", " + 
                str((v2 - v1)/1000000.0) + ", " +
                str((v3 - v2)/1000000.0) + ", " +
                str((v4 - v3)/1000000.0) + ", " +
                str((v7 - v6)/1000000.0) + ", " +
                str((v8 - v7)/1000000.0) + ", " +
                str((v9 - v8)/1000000.0) + ", " +
                str((v10 - v5)/1000000.0) + ", " +
                str((v9 - v2)/1000000.0) + "\n")
    f.close()
    # Now calculate the average, max and min
    sys.stdout.write(str((reduce(lambda x,y: x+y, timesXmit)/len(timesXmit))/1000000.0) + ", " + 
                     str(min(timesXmit)/1000000.0) + ", " +
                     str(max(timesXmit)/1000000.0) + ", " +
                     str((reduce(lambda x,y: x+y, timesALLOC)/len(timesALLOC))/1000000.0) + ", " +
                     str(min(timesALLOC)/1000000.0) + ", " +
                     str(max(timesALLOC)/1000000.0) + ", " +
                     str((reduce(lambda x,y: x+y, timesSHARE)/len(timesSHARE))/1000000.0) + ", " +
                     str(min(timesSHARE)/1000000.0) + ", " +
                     str(max(timesSHARE)/1000000.0) + ", " +
                     str((reduce(lambda x,y: x+y, timesMMAP)/len(timesMMAP))/1000000.0) + ", " +
                     str(min(timesMMAP)/1000000.0) + ", " +
                     str(max(timesMMAP)/1000000.0) + ", " +
                     str((reduce(lambda x,y: x+y, timesBinder)/len(timesBinder))/1000000.0) + ", " +
                     str(min(timesBinder)/1000000.0) + ", " +
                     str(max(timesBinder)/1000000.0) + ", " +
                     str((reduce(lambda x,y: x+y, timesIMPORT)/len(timesIMPORT))/1000000.0) + ", " +
                     str(min(timesIMPORT)/1000000.0) + ", " +
                     str(max(timesIMPORT)/1000000.0) + ", " +
                     str((reduce(lambda x,y: x+y, timesMMAPClient)/len(timesMMAPClient))/1000000.0) + ", " +
                     str(min(timesMMAPClient)/1000000.0) + ", " +
                     str(max(timesMMAPClient)/1000000.0) + "\n")

    x = len(missingVals)      
    if x:                                                              
        sys.stderr.write(str(x) + " values missing from the client" + ": " + str(missingVals))

def summarizeIonCameraBufferStats(dict):                              
    """Calculate a variety of stats for this case"""
    timesFrameInserted = []
    timesFrameDequeued = []
    timesXmit = []
    timesBinder = []
    timesIMPORT = []
    timesMMAPClient = []

    # Write the raw data to a CSV file, in addition to analyzing it
    f = open( args.runDir + '/log.csv', 'a')

    # We might lose messages. Take care of that case by only using
    # sequence ids we find in the receiver, and skipping the rest
    missingVals = []

    # Write column titles
    f.write ("Camera Timestamp, enqueue-onPreview, binder call, ION_IMPORT, mmap, Rx-Tx, dequeue-enqueue, Total\n")
    # Now go through and calculate the status
    for v in dict.getLogSet("onPreviewFrame").keys():
        if v not in dict.getLogSet("ClientCallbackRx").keys():
            missingVals.append(v)
            continue
        v1 = dict.getLogSet("onPreviewFrame")[v]
        v2 = dict.getLogSet("PreviewFrameInserted")[v]
        v3 = dict.getLogSet("ION_CameraFrame_Dequeued")[v]
        v4 = dict.getLogSet("ServerMsgTx")[v]
        v5 = dict.getLogSet("ION_Binder_Pre")[v]
        v6 = dict.getLogSet("ION_Binder_Done")[v]
        v7 = dict.getLogSet("ION_IMPORT_Done")[v]
        v8 = dict.getLogSet("ION_MMAP_Client_Done")[v]
        v9 = dict.getLogSet("ClientCallbackRx")[v]

        timesFrameInserted.append(v2-v1)
        timesFrameDequeued.append(v3-v2)
        timesXmit.append(v9-v4)
        timesBinder.append(v6 - v5)
        timesIMPORT.append(v7 - v6)
        timesMMAPClient.append(v8 - v7)

        f.write(str(v) + ", " + 
                str((v2 - v1)/1000000.0) + ", " +
                str((v6 - v5)/1000000.0) + ", " +
                str((v7 - v6)/1000000.0) + ", " +
                str((v8 - v7)/1000000.0) + ", " +
                str((v9 - v4)/1000000.0) + ", " +    
                str((v3 - v2)/1000000.0) + ", " +
                str((v8 - v4)/1000000.0) + "\n")
    f.close()
    # Now calculate the average, max and min
    sys.stdout.write(str((reduce(lambda x,y: x+y, timesXmit)/len(timesXmit))/1000000.0) + ", " + 
                     str(min(timesXmit)/1000000.0) + ", " +
                     str(max(timesXmit)/1000000.0) + ", " +
                     str((reduce(lambda x,y: x+y, timesBinder)/len(timesBinder))/1000000.0) + ", " +
                     str(min(timesBinder)/1000000.0) + ", " +
                     str(max(timesBinder)/1000000.0) + ", " +
                     str((reduce(lambda x,y: x+y, timesIMPORT)/len(timesIMPORT))/1000000.0) + ", " +
                     str(min(timesIMPORT)/1000000.0) + ", " +
                     str(max(timesIMPORT)/1000000.0) + ", " +
                     str((reduce(lambda x,y: x+y, timesMMAPClient)/len(timesMMAPClient))/1000000.0) + ", " +
                     str(min(timesMMAPClient)/1000000.0) + ", " +
                     str(max(timesMMAPClient)/1000000.0) + "\n")

    x = len(missingVals)      
    if x:                                                              
        sys.stderr.write(str(x) + " values missing from the client" + ": " + str(missingVals))

def loadLogs(dict, loc="/tmp/logs"):
    """This function goes through a package (a directory with
       a __init__.py file) and loads all the modules. For each module
       it calls its addLogs function"""
    modules = []
    for module_loader, mod_name, ispkg in pkgutil.iter_modules(path=[loc]):
        if '__main__' in mod_name or '__init__' in mod_name:
            continue
        modules.append(mod_name)
        if mod_name not in sys.modules:
            # Load module
            mod = module_loader.find_module(mod_name).load_module(mod_name)
            mod.addLogs(dict)

if __name__ == "__main__":
    myLogSets = LogsDict()

    loadLogs(myLogSets, args.runDir)
    if args.testToSummarize.find("client_server") != -1:
        calcTimeDiff(myLogSets, 'ServerMsgTx', 'ClientCallbackRx')
    elif args.testToSummarize.find("ion_buffer") != -1:
        summarizeIonBufferStats(myLogSets)    
    elif args.testToSummarize.find("ion_camera_buffer") != -1:
        summarizeIonCameraBufferStats(myLogSets)    
    else:
        print "Nothing to summarize"
