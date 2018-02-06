/*
 * Copyright (c) 2014 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */
/*
 * CwSim.h
 *
 *  Created on: Aug 15, 2014
 */

#ifndef CANSIM_H_
#define CANSIM_H_

#include "CwBase.h"
#include "CwFrame.h"
#include <pthread.h>

class CSEntry;

///////////////////////////////////////////////////////////////
class CwSim {
protected:
    CwSim(int simMode =0, int playBackDelayMS =1000);
    virtual ~CwSim();
    static CwSim * pTheCwSim;
public:
    // lifecycle APIs
    //
    static CwSim * getInstance();
    int start(int simMode =0, int playBackDelayMS =1000);
    int stop();

//protected:
    static CSEntry canSimPlaylist[];

    int mMode;
    int mDelayMS;

    int createThread();
    pthread_t mThreadId;
    int mThreadRunning;
    int mThreadStop;

    void mainSimLoop();
};

///////////////////////////////////////////////////////////////
class CSEntry {
public:
    CSEntry(int inFrameId, int inFlags, int interfaceNum, int dlen, int d1, int d2);

    void blog(char *pStr);    // blog (dump to log) current entry, id with pStr

    static void blogPlaylist(CSEntry *p, char*);  // blog -1 terminated array of CSEntries

    int frameId;
    int flags;
    int ifNo;
    int dlen;
    uint8_t data[8];

    CwFrame * makeFrame();
};

#endif /* CANSIM_H_ */
