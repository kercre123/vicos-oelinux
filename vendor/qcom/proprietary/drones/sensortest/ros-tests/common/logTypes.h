#pragma once

#include <iostream>
#include <tuple>
#include <fstream>
#include <stdint.h>
#include <boost/thread.hpp>
#define NANOSEC 1000000000ULL
/**
   @class ROSLogs
   This class is used to store performance stats generated during a test.
   Each stat entry is a tuple of the format <name, id, timestamp> and the
   stats are simply stored in a vector to make the log colletion lightweight.
   The stats may be printed to a file at the end, and postprocessed.

   Design
   ------
   It's envisioned that the stats will be printed in the form of python files
   that will be used during the post-processing of the stats.
   The <name, id, timestamp> tuple will be used to populate a dictionary of
   "logsets", where the key is the "name" and the value is the logset.
   The logset itself is a dictionary where the key is the "id" and the value
   is the "timestamp".
   E.g.:
   ~~~~~~~~~~
   [statNameA] -> [1: 100]
                  [2: 200]
                  ...
   [statNameB] -> [1: 1000]
                  [2: 5000]
                  ...
   ~~~~~~~~~~
   Then a post-processing function can make arbitrary calculations on
   these timestamps.

   It is assumed that a summarizing script, e.g. scripts/summarize.py will
   provide a python object logSets, which has a function addToLogSet that
   adds the <id> and <value> to a dictionary corresponding to <name>. Thus
   we'll end up with dictionaries for each of the log type which can be used
   to calculate various stats.
   
   Example
   -------
   The simplest example is that of a client-server pair.
   The server creates a file called server.py:
   ~~~~~~~~~~
     def addLogs(logSets):
       logSets.addToLogSet('ServerMsgTx', 0, 86145791419114)
       logSets.addToLogSet('ServerMsgTx', 1, 86145793088698)
       logSets.addToLogSet('ServerMsgTx', 2, 86145794307083)
       logSets.addToLogSet('ServerMsgTx', 3, 86145795264948)
       logSets.addToLogSet('ServerMsgTx', 4, 86145796494010)
       logSets.addToLogSet('ServerMsgTx', 5, 86145797910260)
   ~~~~~~~~~~
   The client creates client.py:
   ~~~~~~~~~~
     def addLogs(logSets):
       logSets.addToLogSet('Subscribed', 0, 86144781135417)
       logSets.addToLogSet('ClientCallbackRx', 0, 86145797642864)
       logSets.addToLogSet('ClientCallbackRx', 1, 86145805419375)
       logSets.addToLogSet('ClientCallbackRx', 2, 86145808193698)
       logSets.addToLogSet('ClientCallbackRx', 3, 86145808816406)
       logSets.addToLogSet('ClientCallbackRx', 4, 86145811456198)
       logSets.addToLogSet('ClientCallbackRx', 5, 86145813137291)
   ~~~~~~~~~~
   Now the summarizing python script can load each file, call its addLogs
   method, and create two logsets, "ServerMsgTx" and "ClientCallbackRx". It
   can then iterate over each id (0 to 5) and calculate the time differences
   between when the message was sent and when it was received.
**/
                
class ROSLogs
{
 public:
  typedef std::tuple<std::string, std::string, int64_t> LogRecord;
 private:
  static std::vector<LogRecord> logs;
  static boost::mutex mutex;
 public:
  static int addLog(const std::string& name, const std::string& id, int64_t t){ \
    boost::lock_guard<boost::mutex> lock{mutex};
    logs.push_back(LogRecord(name, id, t));
  }

  /**
`   Compute the timestamp and add a log entry
  **/
  static int addLog(const std::string& name, const std::string& id) {
    return addLog(name, id, getLogTime());
  }

  static int64_t getLogTime() {
    struct timespec ts;
    if ( clock_gettime( CLOCK_MONOTONIC, &ts) ) {
      std::cout << "Error: clock_gettime failed. errno: " << errno << std::endl;
      return 0;
    }
    // else
    int64_t t = (int64_t) ((int64_t)ts.tv_sec*NANOSEC + ts.tv_nsec);
    return t;
  }

  /**
     Print logs to std::cout. Mainly needed for debugging
  **/
  static void printLogs() {
    boost::lock_guard<boost::mutex> lock{mutex};
    std::vector<LogRecord>::const_iterator it = logs.begin();
    std::string n, i;
    int64_t t;
    std::cout << "Number of logs: " << logs.size() << std::endl;
    while ( it != logs.end() ){
      std::tie(n, i, t) = *(it);
      std::cout << "name: " << n << ", id: " << i << ", time: " <<  t << std::endl;
      ++it;
    }
  }
  
  /**
     Print the collected data to the file `<logDir>/<name>.py`. The contents of the file
     are a python function of the form:
     ~~~~~~~~~~
     def addLogs(logSets):
        logSets.addToLogSet(<name>, <id>, <value>)
        logSets.addToLogSet(<name>, <id>, <value>)
        logSets.addToLogSet(<name>, <id>, <value>)
	...
     ~~~~~~~~~~
     @param[in] name: Filename for the created log, e.g. name.py
     @param[in] logDir: Directory where to create the file
  **/
  static void printPyLogs(const std::string& name, const std::string& logDir) {
    boost::lock_guard<boost::mutex> lock{mutex};
    std::vector<LogRecord>::const_iterator it = logs.begin();
    std::string n, i;
    int64_t t;
    std::stringstream ss;
    ss << logDir << "/" << name << ".py";
    std::ofstream of(ss.str(), std::ofstream::out);
    of << "def addLogs(logSets):\n";
    while ( it != logs.end() ){
      std::tie(n, i, t) = *(it);
      of << "   logSets.addToLogSet('" << n << "', " << i << ", " << t << ")\n";
      ++it;
    }
    of.close();
  }
};
