/*
 * Copyright (c) 2014-2016 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
/*
 * CwNodeSet.h
 *
 * Encapsulate linked list handling for CwNodes
 * Implements a set of objects of type CwNode
 * Provides interfaces to add/remove CwNode to/from the list
 */

#ifndef CWNODESET_H_
#define CWNODESET_H_

#include "CwBase.h"
#include "CwNode.h"
#include "CwLock.h"
#include <semaphore.h>

////////////////////////////////////////
// CwNodeSet
//
class CwNodeSet: public CwNode, public CwLock {
public:
    // queue methods, FIFO, add to end, remove from front
    virtual void add(CwNode *pNewTail);   // adds to the end of the queue (list)
    virtual CwNode * rem(CwNode *pObjOut = 0); // removes from the front of the queue (list)
    // stack methods, LIFO,
    virtual void push(CwNode *pNewNext); // add (insert) to head of a stack
    virtual CwNode * pop() {
        return rem();
    }; //remove from the top of a stack (front of queue)
    virtual int count(); // return number of items linked in this list
};
#endif /* CWNODESET_H_ */
