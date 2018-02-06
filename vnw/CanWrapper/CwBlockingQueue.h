/*
 * Copyright (c) 2014-2016 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
/*
 * CwBlockingQueue.h
 *
 *  CwBlockingQueue implements a C++ Version of the Java BlockingQueue
 *  It works with CwNodes as the items being queued
 *  and CwWorkers which wrap the threads consuming CwNodes from the queue
 *
 *  enqueueNode(&CwNode) returns int
 *      - is called to add a CwNode to the queue
 *      - if there are CwWorkers waiting,
 *          - then the front CwWorker will immediately get the node
 *      - if no CwWorkers waiting and the queue is not full
 *          - then the node is added to the queue
 *      - if the queue is full, then addNode will return a value of -1 = failure
 *          - CwNode is not consumed in this case
 *  dequeueNode() returns &CwNode
 *      - is called to get a CwNode from the queue
 *      - if there are no CwNodes in the queue, execution will block until there is
 *      - note that CwWorkers will be awoken in any order
 *  dequeueNodeSet(&CwNodeSet, maxNodes) returns int
 *      - is called to get a list of CwNodes from the queue
 *      - can specify maximum nodes to be returned
 *      - returns error (value = -1) when maxNodes > maxEntryCount
 *  countNodes() returns int
 *      - return the size of the queue, number of Nodes to process
 */

#ifndef CWBLOCKINGQUEUE_H_
#define CWBLOCKINGQUEUE_H_

#include "CwNode.h"
#include "CwLock.h"
#include "CwNodeSet.h"
#include <semaphore.h>

///////////////////////////////////////////////////////////////
// CwBlockingQueue
//
class CwBlockingQueue {
public:
    CwBlockingQueue(int maxEntryCount = 1024);
    virtual ~CwBlockingQueue();
    int enqueueNode(CwNode * pNode);
    CwNode * dequeueNode();
    int dequeueNodeSet(CwNodeSet * pNodeSet, unsigned int maxNodes);
    int countNodes();
protected:
    CwNodeSet mNodes;     // queue of CwNodes to process
    int mMaxEntryCount;
    sem_t mSemaphore;
};

#endif /* CWBLOCKINGQUEUE_H_ */
