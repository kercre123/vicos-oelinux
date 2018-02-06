/*
 * Copyright (c) 2014 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */
/*
 * CwNode.h
 * Inherit from this to make an object a node in a list.
 * CwNodeSet is used to make a list head of CwNode objects.
 */

#ifndef CWNODE_H_
#define CWNODE_H_

class CwNode {
public:
    CwNode() {mNext = 0;};
    virtual ~CwNode() {;};

    virtual void setNext(CwNode * atThis) {
        mNext = atThis;
    }
    virtual CwNode * getNext() {
        return mNext;
    }
protected:
    CwNode * mNext;        // next object after this one
};

#endif /* CWNODE_H_ */
