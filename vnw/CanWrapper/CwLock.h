/*
 * Copyright (c) 2014-2016 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
/*
 * CwLock.h
 *
 *  Interfaces for lock/unlock  requirements around critical sections.
 *  These are semaphore based.
 *  These locks don't nest.
 */

#ifndef CWLOCK_H_
#define CWLOCK_H_

#include <semaphore.h>

///////////////////////////////////////////////////////////////
// CwLock - a basic blocking semaphore
//        - makes call to sem_wait for lock()
//        - makes call to sem_post for unlock()
//
class CwLock {
public:
    CwLock();   // creates an open lock
    virtual ~CwLock();
    void lock();    // enter critical section
    void unlock();  // leave critical section
private:
    sem_t mSemLok;
};
#endif /* CWLOCK_H_ */
