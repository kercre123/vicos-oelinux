#!/usr/bin/env python
## @package runtests
# Top-level script to run these ROS tests
# 
# This script runs a set of test cases specified on the command line.
# The test cases may be specified individually on the command line,
# or as a collection of pre-defined test suites.
# 
# The test cases use a dict to store their test configs.
# 
# The overall design is:
# - Use roslaunch to start the nodes and let them run to completion.
# - Call summarize.py, with the test name to analyze the results
# - Create a combinedLogs.csv file as the result
# 
import os
import subprocess
import argparse
import shutil

DEFAULT_RUN_DIR = './logs'

## Big test suite with all the tests
#  May be out of date since tests may be added without them actually getting added to this list
allTestCases = [
    "client_server_1_10",
    "client_server_10_100",
    "client_server_100_1000",
    "client_server_1000_1000",
    "client_server_10000_10000",
    "client_server_1_10_nd",
    "client_server_10_100_nd",
    "client_server_100_1000_nd",
    "client_server_1000_1000_nd",
    "client_server_10000_10000_nd",
    "client_server_1_10_udp",
    "client_server_10_100_udp",
    "client_server_100_1000_udp",
    "client_server_1000_1000_udp",
    "client_server_10000_10000_udp",
    "client_server_buffer_1000_1000_1000",
    "client_server_buffer_1000_1000_1000_udp",
    "client_server_qvga_30_1000",
    "client_server_qvga_30_1000_udp",
    "client_server_vga_30_1000",
    "client_server_vga_30_1000_udp",
    "client_server_1080p_30_1000",
    "client_server_1080p_30_1000_udp",
    "client_server_4k_30_1000",
    "client_server_4k_30_1000_udp",
    "ion_camera_buffer_30_100_200",
    "ion_camera_buffer_60_100_200",
    "ion_camera_buffer_90_100_200",
    "ion_camera_buffer_120_100_200",
]

## Minimal test suite
sanityTestCases = [
    "client_server_1000_1000",
    "client_server_1000_1000_nd",
    "client_server_1000_1000_udp",
    "client_server_10000_10000_udp",
    "client_server_buffer_1000_1000_1000",
    "ion_camera_buffer_30_100_200"]

## Test suite for Camera buffer tests
ionCameraSuite = [
    "ion_camera_buffer_30_100_200",
    "ion_camera_buffer_60_100_200",
    "ion_camera_buffer_90_100_200",
    "ion_camera_buffer_120_100_200",
    "ion_camera_buffer_30_100_1000",
    "ion_camera_buffer_60_100_1000",
    "ion_camera_buffer_90_100_1000",
    "ion_camera_buffer_120_100_1000",
    "ion_camera_buffer_30_100_10000",
    "ion_camera_buffer_60_100_10000",
    "ion_camera_buffer_90_100_10000",
    "ion_camera_buffer_120_100_10000"
]

## Selected tests for performance measurement    
performanceSuite = [
    "client_server_1_10",
    "client_server_10_100",
    "client_server_100_1000",
    "client_server_1000_1000",
    "client_server_10000_10000",
    "client_server_qvga_30_1000",
    "client_server_vga_30_1000",
    "client_server_1080p_30_1000",
    "client_server_4k_30_1000",
    "ion_camera_buffer_30_100_200",
    "ion_camera_buffer_60_100_200",
    "ion_camera_buffer_90_100_200",
    "ion_camera_buffer_120_100_200",
]

## The test configs for each test.
# The test suites only contain the test names. This `<test name> -> <test config>`
# mapping is used to define how the test needs to be run, using roslaunch
# 
# the convention used to name the tests is <root>_xx_yy_... where xx, yy, zz, etc. denote
# different things for different tests. The hope is that these are self explanatory.
# 
# E.g. for the first few tests _xx_yy denotes loop rate and iterations, but for
# client_server_xx_yy_zz they denote loop rate, iterations and message size
testConfigs = {
    'client_server_1_10': {'name': 'client_server', 'loopRate': 1, 'iterations': 10, 'transport': 'tcp', 'msgType': 'string'},
    'client_server_10_100': {'name': 'client_server', 'loopRate': 10, 'iterations': 100, 'transport': 'tcp', 'msgType': 'string'},
    'client_server_100_1000': {'name': 'client_server', 'loopRate': 100, 'iterations': 100, 'transport': 'tcp', 'msgType': 'string'},
    'client_server_1000_1000': {'name': 'client_server', 'loopRate': 1000, 'iterations': 1000, 'transport': 'tcp', 'msgType': 'string'},
    'client_server_10000_10000': {'name': 'client_server', 'loopRate': 10000, 'iterations': 10000, 'transport': 'tcp', 'msgType': 'string'},

    'client_server_1_10_nd': {'name': 'client_server', 'loopRate': 1, 'iterations': 10, 'transport': 'tcpNoDelay', 'msgType': 'string'},
    'client_server_10_100_nd': {'name': 'client_server', 'loopRate': 10, 'iterations': 100, 'transport': 'tcpNoDelay', 'msgType': 'string'},
    'client_server_100_1000_nd': {'name': 'client_server', 'loopRate': 100, 'iterations': 100, 'transport': 'tcpNoDelay', 'msgType': 'string'},
    'client_server_1000_1000_nd': {'name': 'client_server', 'loopRate': 1000, 'iterations': 10000, 'transport': 'tcpNoDelay', 'msgType': 'string'},
    'client_server_10000_10000_nd': {'name': 'client_server', 'loopRate': 10000, 'iterations': 10000, 'transport': 'tcpNoDelay', 'msgType': 'string'},

    'client_server_1_10_udp': {'name': 'client_server', 'loopRate': 1, 'iterations': 10, 'transport': 'udp', 'msgType': 'string'},
    'client_server_10_100_udp': {'name': 'client_server', 'loopRate': 10, 'iterations': 100, 'transport': 'udp', 'msgType': 'string'},
    'client_server_100_1000_udp': {'name': 'client_server', 'loopRate': 100, 'iterations': 1000, 'transport': 'udp', 'msgType': 'string'},
    'client_server_1000_1000_udp': {'name': 'client_server', 'loopRate': 1000, 'iterations': 1000, 'transport': 'udp', 'msgType': 'string'},
    'client_server_10000_10000_udp': {'name': 'client_server', 'loopRate': 10000, 'iterations': 100000, 'transport': 'udp', 'msgType': 'string'},

    'client_server_buffer_1000_1000_1000': {'name': 'client_server', 'loopRate': 1000, 'iterations': 1000, 'transport': 'tcp', 'msgType': 'buffer', 'msgSize': 1000},
    'client_server_buffer_1000_1000_1000_udp': {'name': 'client_server', 'loopRate': 1000, 'iterations': 1000, 'transport': 'udp', 'msgType': 'buffer', 'msgSize': 1000},
    'client_server_qvga_30_1000': {'name': 'client_server', 'loopRate': 30, 'iterations': 1000, 'transport': 'tcp', 'msgType': 'buffer', 'msgSize': 115200},
    'client_server_qvga_30_1000_udp': {'name': 'client_server', 'loopRate': 30, 'iterations': 1000, 'transport': 'udp', 'msgType': 'buffer', 'msgSize': 115200},
    'client_server_vga_30_1000': {'name': 'client_server', 'loopRate': 30, 'iterations': 1000, 'transport': 'tcp', 'msgType': 'buffer', 'msgSize': 460800},
    'client_server_vga_30_1000_udp': {'name': 'client_server', 'loopRate': 30, 'iterations': 1000, 'transport': 'udp', 'msgType': 'buffer', 'msgSize': 460800},
    'client_server_1080p_30_1000': {'name': 'client_server', 'loopRate': 30, 'iterations': 1000, 'transport': 'tcp', 'msgType': 'buffer', 'msgSize': 3110400},
    'client_server_1080p_30_1000_udp': {'name': 'client_server', 'loopRate': 30, 'iterations': 1000, 'transport': 'udp', 'msgType': 'buffer', 'msgSize': 3110400},
    'client_server_4k_30_1000': {'name': 'client_server', 'loopRate': 30, 'iterations': 1000, 'transport': 'tcp', 'msgType': 'buffer', 'msgSize': 12441600},
    'client_server_4k_30_1000_udp': {'name': 'client_server', 'loopRate': 30, 'iterations': 1000, 'transport': 'udp', 'msgType': 'buffer', 'msgSize': 12441600},

    'ion_camera_buffer_30_100_200': {'name': 'ion', 'loopRate': 30, 'iterations':100, 'transport':'tcp', 'msgType': 'ion_camera_buffer', 'msgSize': 200},
    'ion_camera_buffer_60_100_200': {'name': 'ion', 'loopRate': 60, 'iterations':100, 'transport':'tcp', 'msgType': 'ion_camera_buffer', 'msgSize': 200},
    'ion_camera_buffer_90_100_200': {'name': 'ion', 'loopRate': 90, 'iterations':100, 'transport':'tcp', 'msgType': 'ion_camera_buffer', 'msgSize': 200},
    'ion_camera_buffer_120_100_200': {'name': 'ion', 'loopRate': 120, 'iterations':100, 'transport':'tcp', 'msgType': 'ion_camera_buffer', 'msgSize': 200},

    'ion_camera_buffer_30_100_1000': {'name': 'ion', 'loopRate': 30, 'iterations':100, 'transport':'tcp', 'msgType': 'ion_camera_buffer', 'msgSize': 1000},
    'ion_camera_buffer_60_100_1000': {'name': 'ion', 'loopRate': 60, 'iterations':100, 'transport':'tcp', 'msgType': 'ion_camera_buffer', 'msgSize': 1000},
    'ion_camera_buffer_90_100_1000': {'name': 'ion', 'loopRate': 90, 'iterations':100, 'transport':'tcp', 'msgType': 'ion_camera_buffer', 'msgSize': 1000},
    'ion_camera_buffer_120_100_1000': {'name': 'ion', 'loopRate': 120, 'iterations':100, 'transport':'tcp', 'msgType': 'ion_camera_buffer', 'msgSize': 1000},

    'ion_camera_buffer_30_100_10000': {'name': 'ion', 'loopRate': 30, 'iterations':100, 'transport':'tcp', 'msgType': 'ion_camera_buffer', 'msgSize': 10000},
    'ion_camera_buffer_60_100_10000': {'name': 'ion', 'loopRate': 60, 'iterations':100, 'transport':'tcp', 'msgType': 'ion_camera_buffer', 'msgSize': 10000},
    'ion_camera_buffer_90_100_10000': {'name': 'ion', 'loopRate': 90, 'iterations':100, 'transport':'tcp', 'msgType': 'ion_camera_buffer', 'msgSize': 10000},
    'ion_camera_buffer_120_100_10000': {'name': 'ion', 'loopRate': 120, 'iterations':100, 'transport':'tcp', 'msgType': 'ion_camera_buffer', 'msgSize': 10000},
}

## Create the specified log directory
#
# All the logs will be put here
def makeLogDir(d):
    if not os.path.isdir(d):
        try:
            os.makedirs(d)
        except e:
            print "[ERROR] Creating directory " + d + ": " + str(e)
            exit(1)
        f = d+"/__init__.py"
        subprocess.call(["touch", f])
    else:
        print ("[WARNING] " + d + " already exists. Existing content may be lost")

## Run the specified test
#
# It uses roslaunch, using:
# - 'name'.xml config option as the launch file
# - All the other config options are provided as ROS launch options.
#
# After the test finishes, it invokes summarize.py, providing it:
# - The log directory
# - The test name 'name'
#
# It then takes what summarize.py printed to stdout, and adds it to logs.csv
# and also renames the log.csv produced by summarize.py and renames it to
# 'name'.csv
# 
def runTest(test):
    logDir = args.runDir + '/' + test
    makeLogDir(logDir)
    cmd = ['roslaunch',
           'ros_tests',
           testConfigs[test]['name']+'.launch',
           'logDir:='+logDir]
    for k in testConfigs[test].keys():
        cmd.append(k + ':=' + str(testConfigs[test][k]))
                         
    print "Test: " + test,
    print "Name: " + testConfigs[test]['name'],
    for k in testConfigs[test].keys():
        print k + ':=' + str(testConfigs[test][k]) + " ",
    print

    p = subprocess.Popen(cmd,
                         stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE)
    out,err=p.communicate()
    p = subprocess.Popen(['/opt/ros/indigo/share/ros_tests/summarize.py',
                          '-d',
                          logDir,
                          '-t',
                          test],
                          stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE)
    out,err = p.communicate()
    print out
    if  err:
        print err
    os.rename(logDir + '/log.csv', args.runDir + '/' + test + '.csv')

    f = open( args.runDir + '/logs.csv', 'a')
    f.write(test + "," + out)
    f.close()

## @cond
# An attempt to prevent Doxygen from parsing the rest of this file
parser = argparse.ArgumentParser(prog='runtests.py')
parser.add_argument('-d', 
                    help='Directory for test artifacts. Default: '+DEFAULT_RUN_DIR, dest='runDir', 
                    default=DEFAULT_RUN_DIR)
parser.add_argument('-t', 
                    help='List of tests to run. If not specified, all tests are run', 
                    dest='tests', 
                    nargs='+', 
                    default=[])
parser.add_argument('-s', 
                    help='Test suite to run. Choose from ion_camera_suite, performance_suite, all, sanity',
                    dest='suite')
args = parser.parse_args()

args.runDir = os.path.abspath(args.runDir)

if args.suite :
    if args.suite == 'ion_camera_suite':
        testsToRun = ionCameraSuite
    elif args.suite == 'performance_suite':
        testsToRun = performanceSuite
    elif args.suite == 'all':
        testsToRun = allTestCases
    elif args.suite == 'sanity':
        testsToRun = sanityTestCases
    else:
        print "Unknown suite " + args.suite
        sys.exit(1)
else:
    if not args.tests :
        testsToRun = allTestCases
    else:
        testsToRun = args.tests

# Disable SELinux
print "[INFO] Setting SELinux to permissive"
cmd = ['setenforce',
       '0']

p = subprocess.Popen(cmd,
                     stdout=subprocess.PIPE,
                     stderr=subprocess.PIPE)
out,err=p.communicate()
print out

print "===Starting==="
print "   Running Tests: " + str(testsToRun)
print "   Logs in      : " + args.runDir
print "============="

for test in testsToRun:
    runTest(test)
## @endcond
# Now combine all the csv files into one
outf = args.runDir + "/combinedLogs.csv"
with open(outf, 'w') as outfile:
   shutil.copyfileobj(open(args.runDir+'/logs.csv'),outfile)
   for infile in testsToRun:
        f = args.runDir + '/' + infile + '.csv'
        outfile.write(infile + '\n')
        shutil.copyfileobj(open(args.runDir+'/'+infile+'.csv'),outfile)

## @endcond
