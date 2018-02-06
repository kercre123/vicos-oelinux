/*
 * Copyright (c) 2014, 2016 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

/*----------------------------------------------------------
 * CanWrapper.h
 * ----------------------------------------------------------
 * This class is the "Front-End" to: libCanWrapper.
 * It provides APIs to Send messages, get callbacks, and discern interfaces
 *
 * "libcwtest" demonstrates these APIs (if you have source)
 * or at least will exercise your installation.
 * "libcwtest -?" for help
 *
 * The test simulator is exposed for your convenience.
 */
#ifndef CANWRAPPER_H_
#define CANWRAPPER_H_

#include "CwBase.h"
#include "CwFrame.h"    // CanWrapper sends and receives CwFrames which wrap can data
#include "CwBlockingQueue.h"

#define DRIVER_MODE_RAW_FRAMES 0
#define DRIVER_MODE_PROPERTIES 1
#define DRIVER_MODE_AMB 2

///////////////////////////////////////////////////////////////
// CanCallback - callback prototype - for receiving CAN traffic
///@param pFrame - pointer at incoming CwFrame object
///              - CanWrapper retains ownership of CwFrame
///@param userData - the unit32 passed into registerListener() for this callback
///@param ifNo - interface number this frame was received on
///            - e.g. it's 1 if the frame was received on "can1"
//
typedef void (*CanCallback)(CwFrame* pFrame, void* userData, int ifNo);
typedef void (*CanQCallback)(CwNodeSet *nodeSet, void *userData, int numFrames);

///////////////////////////////////////////////////////////////
// CanWrapper - the Client interface singleton
//
class CanWrapper {
private:  // CanWrapper is a SINGLETON
    static CanWrapper * pTheCanWrapper;
    CanWrapper(int mode);
    ~CanWrapper();   // we sustain a CanWrapper until the creating process dissolves
    int init(int mode);
public:
    /*----------------------------------------------------
     * getInstance()
     * @returns CanWrapper * the VN Can Wrapper Singleton, in Raw Frames mode
     */
    static CanWrapper * getInstance();

    /*----------------------------------------------------
     * getInstance()
     * @returns CanWrapper * the VN Can Wrapper for raw or FD frames
     */
    static CanWrapper * getInstance(int mode);

    /*------------------------------------------------------------
     * createWorkerQueue
     * @brief  This function creates queue, establishes a callback function with queue
     * @parm numWorkers  Number of worker threads which would call the callback function
     * @param cb - Callback function to be called upon receiving frames
     * @parm userData - this is returned, unchanged, in the CanQCallback() function
     * @returns pointer to receiver queue
     * @desc This is the mechanism used to receive incoming messages over queue
     *         It creates worker threads which call callback function on incoming frames
     */
    CwBlockingQueue * createWorkerQueue(int numWorkers, CanQCallback cb,
            void *userData);

    /*------------------------------------------------------------
     * registerListener - using callback method
     * @brief  This function establishes a callback function for a FrameID set
     * @parm frameId  use this to match incoming frame ID after anding the mask
     * @parm frIdMask is anded with frameId, then matched with incoming frameId & frIdMask,
     *       e.g:  match = (frameId & frIdMask) == (wire.frameId & frIdMask)
     *       note: frIdMask==0 will match all frames
     *       note: typically, for normal frames use CwBase::MASK11,
     *             or CwBase::MASK29 for extended address frames
     * @parm callbackFunc  called when matching msg is read
     *       note: this type is defined as CanCallback above
     * @parm userData - this is returned, unchanged, in the CanCallback() function
     * @parm ifNo - logical interface (or special, e.g. CwBase::IFACE_ANY)
     * @parm filterType - frame can be requested onchange or onrefresh
     * @returns earId - unique ID used in the unregisterListener(earId) call
     * @desc This is the mechanism used to receive incoming messages.
     *     >> Ownership of the CwFrame in the Callback STAYS WITH CanWrapper <<
     *       Copy what you need from the CwFrame before you return to CanWrapper.
     */
    int registerListener(uint32_t frameId, uint32_t frIdMask, CanCallback callbackFunc,
            void* userData = NULL, int ifNo = CwBase::IFACE_ANY, uint8_t filterType = 0);

    /*------------------------------------------------------------
     * registerBufferedDataListener - using separate callback method for buffered frames.
     * @brief This function establishes a callback function for the buffered frames.
     * @parm frameId  use this to match incoming frame ID after anding the mask
     * @parm frIdMask is anded with frameId, then matched with incoming frameId & frIdMask,
     *       e.g:  match = (frameId & frIdMask) == (wire.frameId & frIdMask)
     *       note: frIdMask==0 will match all frames
     *       note: typically, for normal frames use CwBase::MASK11,
     *             or CwBase::MASK29 for extended address frames
     * @parm callbackFunc  called when matching msg is read
     *       note: this type is defined as CanCallback above
     * @parm userData - this is returned, unchanged, in the CanCallback() function
     * @parm ifNo - logical interface (or special, e.g. CwBase::IFACE_ANY)
     * @parm filterType - frame can be requested onchange or onrefresh
     * @returns earId - unique ID used in the unregisterListener(earId) call
     * @desc The application need to specify the frame IDs it wants to buffer using
     * registerBufferedDataListener API and this function establishes a callback funtion
     * for the buffered frames.
     */
    int registerBufferedDataListener(uint32_t frameId, uint32_t frIdMask, CanCallback callbackFunc,
            void* userData = NULL, int ifNo = CwBase::IFACE_ANY, uint8_t filterType = 0);
    /*------------------------------------------------------------
     * unregisterListener - by earId as returned in registerListener
     */
    int unregisterListener(int earId);

    /*------------------------------------------------------------
     * updateListener - update filter settings
     * @brief  This function updates a filter setting for a FrameID
     * @parm frameId  use this to match incoming frame ID after anding the mask
     * @parm frIdMask is anded with frameId, then matched with incoming frameId & frIdMask,
     * @parm callbackFunc  called when matching msg is read
     * @parm userData - this is returned, unchanged, in the CanCallback() function
     * @parm ifNo - logical interface (or special, e.g. CwBase::IFACE_ANY)
     * @parm filterType - frame can be requested onchange or onrefresh
     * @returns earId - unique ID used in the unregisterListener(earId) call
     */
    int updateListener(uint32_t frameId, uint32_t frIdMask, CanCallback callbackFunc,
            void* userData, int ifNo, uint8_t filterType);

    /*------------------------------------------------------------
     * registerListener - using queue based worker thread  method
     * @brief  This function establishes receiver queue for a FrameID
     * @parm frameId  use this to match incoming frame ID after anding the mask
     * @parm frIdMask is anded with frameId, then matched with incoming frameId & frIdMask,
     *       e.g:  match = (frameId & frIdMask) == (wire.frameId & frIdMask)
     *       note: frIdMask==0 will match all frames
     *       note: typically, for normal frames use CwBase::MASK11,
     *             or CwBase::MASK29 for extended address frames
     * @param rxQueue - Receive Queue to which frames received from socket are pushed
     * @parm ifNo - logical interface (or special, e.g. CwBase::IFACE_ANY)
     * @parm filterType - frame can be requested onchange or onrefresh
     * @returns earId - unique ID used in the unregisterListener(earId) call
     * @desc This is the mechanism used to receive incoming messages over queue
     */
    int registerListener(uint32_t frameId, uint32_t frIdMask,
            CwBlockingQueue *rxQueue, int ifNo = CwBase::IFACE_ANY, uint8_t filterType = 0);

    /*------------------------------------------------------------
     * registerBufferedDataListener - using queue based worker thread method for buffered frames.
     * @brief This function establishes receiver queue for a FrameID
     * @parm frameId  use this to match incoming frame ID after anding the mask
     * @parm frIdMask is anded with frameId, then matched with incoming frameId & frIdMask,
     *       e.g:  match = (frameId & frIdMask) == (wire.frameId & frIdMask)
     *       note: frIdMask==0 will match all frames
     *       note: typically, for normal frames use CwBase::MASK11,
     *             or CwBase::MASK29 for extended address frames
     * @param rxQueue - Receive Queue to which frames received from socket are pushed
     * @parm ifNo - logical interface (or special, e.g. CwBase::IFACE_ANY)
     * @parm filterType - frame can be requested onchange or onrefresh
     * @returns earId - unique ID used in the unregisterListener(earId) call
     * @desc The application need to specify the frame IDs it wants to buffer using
     * registerBufferedDataListener API and this function is used to receive the buffered frames
     * over queue.
     */
    int registerBufferedDataListener(uint32_t frameId, uint32_t frIdMask,
            CwBlockingQueue *rxQueue, int ifNo = CwBase::IFACE_ANY, uint8_t filterType = 0);

    /* releaseBufferedData
     * @brief This functions releases buffered CAN frames to the applications
     * @param ifNo - logical interface (or special, e.g. CwBase::IFACE_ANY)
     * @returns 0 on success and error code on failure.
     */
    int releaseBufferedData(int ifNo = CwBase::IFACE_ANY);

    /*------------------------------------------------------------
     * sendMessage
     * @brief This function sends a can message frame out a single interface.
     * @param frame pointer to the CwFrame object
     *      ownership of this object is passed to CanWrapper on method success
     *      and CwFrame object is deleted before returning.
     * @param ifNo = the interface number
     * @returns 0 = success, can message queued to be sent, frame accepted
     *          1 = error, message not sent, queue blocked by errors, frame not consumed
     *              ** the CwFrame is still owned by caller in this condition
     * @desc it bears repeating:
     *    >> ownership of this object is passed to CanWrapper except when error <<
     */
    int sendMessage(CwFrame* frame, int ifNo);

    /* ---------------------------------------------------
     * getRevision - return CanWrapper version.revision
     * @returns float - in ver.rev format
     */
    float getRevision();

    ///////////////////////////////////////////////////////////////
    // Test calls follow
    /* ------------------------------------------------------------
     * "simulate()" starts the simulator running.
     * CanSim is the simulator, it loops sending the playlist of CanMessages
     *   as defined in CanSim.h.
     * The caller may wish to issue one or more registerListeners
     *   to get callbacks.
     * Note that calling registerListener with frIdMask=0 and ifNo=CwBase::IFACE_ANY
     *   should allow you to receive all messages within the playlist.
     * "simulate()" works whether connected to a can bus or not
     *   and will intersperse messages with real messages, if they are arriving.
     * "real(0)" will turn off incoming "real" messages
     *   as well as cause outgoing (sendMessage() frames) to be dropped before the wire.
     */
    ///////////////////////////////////////////////////////////////

    /*------------------------------------------------
     * simulate - start a simulate playback
     * @param mode - - a hook for future
     * @param playBackDelayMS - sleep this between incoming can frames
     * @returns errorCode = 0 is okay
     */
    int simulate(int mode =0, int playbackDelayMs =1000);

    /*------------------------------------------------
     * unSimulate - STOP a simulate playback
     * @returns errorCode = 0 is okay
     */
    int unSimulate();

    /*------------------------------------------------------------
     * real - set the doReal flag here and in the key driving components (Boss & SockCan)
     */
    void real(int is =1);    // set to 0 to turn it off

    /*------------------------------------------------------------
     * getIface -- low-level test, check for existance of an interface!!
     * @brief return a logical id for a given CAN interface
     * @parm psdev  a char string such as: "/dev/can1"
     *    NULL gets the default
     * @returns int a logical number to use in ensuing calls
     *    a negative return signifies an error, usually "Not Found"<br>
     *    NOTE: this number is PROBALBY what socketCAN uses,
     *    however, it might be one of the Meta ifaces:
     *       e.g.: CwBase::IFACE_ALL
     * @desc This function returns the system interface number. <br>
     *    This is solely for test or troubleshooting uses.
     */
    int getIface(char * psdev=0);
    //
    //$ ToDO:
    //$ findIfaces - causes us to look for any and all possible CAN interfaces
    //$ listIfaces - returns array of strings naming the interfaces
    //$ listIfaceNums - returns int array of valid interface numbers

    /*
     * Add this canId to the list of messages that should be buffered upon boot
     * Buffered messages will be released to the first client
     * who calls getInstance()
     */
    int enableBuffering(uint8_t ifNo, uint32_t canId, uint32_t mask);
protected:
    int mDoReal;  // flag used by the above sender & receivers to enable real bus access
};

#endif /* CANWRAPPER_H_ */
