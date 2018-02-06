/*
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

//////////////////////////////////////////////////////////////////////////////////////
//
// This file contains calls that are made through Binder and on to TrustZone. The functions
// here are called by the Binder BnQvopService::OnTransact function and will do the
// interface calls. The tz interface is responsible for marshaling data in/out of TZ.

#include <stdio.h>

#include <map>

#include "QvopServiceAndroid.h"
#include "QvopCommandGetVersion.h"
#include "QvopCommandProcessFrame.h"
#include "QvopCommandMatch.h"
#include "QvopCommandMatchGetResult.h"
#include "QvopCommandEnroll.h"
#include "QvopCommandEnrollTrainStart.h"
#include "QvopCommandEnrollTrainComplete.h"
#include "QvopCommandEnrollCommit.h"
#include "QvopCommandMatchAny.h"
#include "QvopCommandDelete.h"
#include "QvopCommandRename.h"
#include "QvopCommandSetThreshold.h"
#include "QvopNullCallback.h"



namespace android {

// map binder transaction codes to their string representation
static const std::pair<int32_t, std::string> tzResponseCode[] = {
  std::make_pair(QVOP_SUCCESS, "QVOP_SUCCESS"),
  std::make_pair(QVOP_FAILURE, "QVOP_FAILURE"),
  std::make_pair(QVOP_TIMEOUT, "QVOP_TIMEOUT"),
  std::make_pair(QVOP_CANCEL, "QVOP_CANCEL"),
  std::make_pair(QVOP_STOPPED, "QVOP_STOPPED"),
  std::make_pair(QVOP_PCM_DATA_REQUEST, "QVOP_PCM_DATA_REQUEST"),
  std::make_pair(QVOP_PCM_REWIND, "QVOP_PCM_REWIND"),
  std::make_pair(QVOP_ENROLLMENT_COMPLETE, "QVOP_ENROLLMENT_COMPLETE"),
  std::make_pair(QVOP_TZ_CANCEL_SUCCESS, "QVOP_TZ_CANCEL_SUCCESS"),
  std::make_pair(QVOP_ENROLLMENT_FAILED, "QVOP_ENROLLMENT_FAILED"),
  std::make_pair(QVOP_ENROLLMENT_NO_SPACE, "QVOP_ENROLLMENT_NO_SPACE"),
  std::make_pair(QVOP_ENROLLEE_NOT_FOUND, "QVOP_ENROLLEE_NOT_FOUND"),
  std::make_pair(QVOP_MATCH_SUCCESS, "QVOP_MATCH_SUCCESS"),
  std::make_pair(QVOP_MATCH_COMPLETE, "QVOP_MATCH_COMPLETE"),
  std::make_pair(QVOP_MATCH_FAILURE, "QVOP_MATCH_FAILURE"),
  std::make_pair(QVOP_ERR_UNKNOWN_COMMAND, "QVOP_ERR_UNKNOWN_COMMAND"),
  std::make_pair(QVOP_ERR_NOT_IMPLEMENTED, "QVOP_ERR_NOT_IMPLEMENTED"),
  std::make_pair(QVOP_ERR_NOT_INITIALIZED, "QVOP_ERR_NOT_INITIALIZED"),
  std::make_pair(QVOP_ERR_BAD_PARAM, "QVOP_ERR_BAD_PARAM"),
  std::make_pair(QVOP_ERR_MALLOC, "QVOP_ERR_MALLOC"),
  std::make_pair(QVOP_ERR_FREE, "QVOP_ERR_FREE"),
  std::make_pair(QVOP_ENROLLMENT_SECURE_UI_ERR, "QVOP_ENROLLMENT_SECURE_UI_ERR"),
};

static const std::map<uint32_t, std::string> binderCodes(tzResponseCode,
    tzResponseCode + sizeof tzResponseCode / sizeof *tzResponseCode);

QvopService::QvopService() : mSampleRate(AUDIO_DEFAULT_SAMPLE_RATE),
mNumberChannels(AUDIO_DEFAULT_NUMBER_CHANNELS),
mAudioFormat(AUDIO_DEFAULT_FORMAT),
mIsShutdown(false)
{
    logfile_print_v("%s:QvopService constructed", QVOP_FN);
}

QvopService::~QvopService() {

}

void QvopService::unlink(sp<IQvopCallback> const& callback) {
    // release callback strong pointer
    sp<IBinder> binder = QvopAsBinder(callback);
            binder->unlinkToDeath(this);
}

void QvopService::link(sp<IQvopCallback> const& callback) {
    // obtain callback strong pointer & link
    sp<IBinder> binder = QvopAsBinder(callback);
    binder->linkToDeath(this);
}

QvopStatus QvopService::getVersion(sp<IQvopCallback> const& callback) {
    logfile_print_v("%s: entry", QVOP_FN);
    QvopCommandGetVersion cmd;
    return execute(callback, cmd);
}

QvopStatus QvopService::enroll(sp<IQvopCallback> const& callback,
                               const char* user_id,
                               const char* identifier,
                               const char* keyphrase) {
    logfile_print_v("%s: entry user=%s, id=%s, phrase=%s", QVOP_FN, user_id, identifier, keyphrase);
    QvopCommandEnroll cmd(user_id, identifier, keyphrase);
    return execute(callback, cmd);
}

QvopStatus QvopService::setThreshold(sp<IQvopCallback> const& callback,
                               const char* user_id,
                               const char* identifier,
                               int32_t setThreshold) {
    logfile_print_v("%s: entry user=%s, id=%s, setThreshold=%d", QVOP_FN, user_id, identifier, setThreshold);
    QvopCommandSetThreshold cmd(user_id, identifier, setThreshold);
    return execute(callback, cmd);
}

QvopStatus QvopService::rename(sp<IQvopCallback> const& callback,
                               const char* user_id,
                               const char* old_id,
                               const char* new_id) {
    logfile_print_v("%s: entry user=%s, old_id=%s, new_id=%s", QVOP_FN, user_id, old_id, new_id);
    QvopCommandRename cmd(user_id, old_id, new_id);
    return execute(callback, cmd);
}

QvopStatus QvopService::enrollCaptureStart(sp<IQvopCallback> const& callback) {
    logfile_print_v("%s: entry", QVOP_FN);
    //do nuttin'
    return QVOP_SUCCESS;
}

QvopStatus QvopService::enrollCaptureComplete(sp<IQvopCallback> const& callback) {
    logfile_print_v("%s: entry", QVOP_FN);
    QvopCommandEnrollTrainComplete cmd;
    return execute(callback, cmd);
}

QvopStatus QvopService::enrollCommit(sp<IQvopCallback> const& callback) {
    logfile_print_v("%s: entry", QVOP_FN);
    QvopCommandEnrollCommit cmd;
    return execute(callback, cmd);
}

QvopStatus QvopService::matchCaptureComplete(sp<IQvopCallback> const& callback) {
    logfile_print_v("%s: entry", QVOP_FN);
    QvopCommandMatchGetResult cmd;
    return execute(callback, cmd);
}

QvopStatus QvopService::match(sp<IQvopCallback> const& callback, const char* user_id) {
    logfile_print_v("%s: entry", QVOP_FN);
    return matchWithId(callback, user_id, 0);
}

QvopStatus QvopService::matchWithId(sp<IQvopCallback> const& callback,
                                    const char* user_id,
                                    const char* identifier) {
    logfile_print_v("%s: entry", QVOP_FN);
    QvopCommandMatch cmd(user_id, identifier);
    return execute(callback, cmd);
}

QvopStatus QvopService::matchSecure(sp<IQvopCallback> const& callback,
                                    const char* user_id) {
    logfile_print_v("%s: entry", QVOP_FN);
    QvopCommandMatch cmd(user_id, 0);
    return execute(callback, cmd);
}

QvopStatus QvopService::matchAny(sp<IQvopCallback> const& callback,
                                const char* user_id) {
    logfile_print_v("%s: entry", QVOP_FN);
    QvopCommandMatchAny cmd(user_id, 0);
    return execute(callback, cmd);
}

QvopStatus QvopService::matchAnyKeyPhrase(sp<IQvopCallback> const& callback,
                                            const char* user_id,
                                            const char* phrase) {
    logfile_print_v("%s: entry", QVOP_FN);
    QvopCommandMatchAny cmd(user_id, 0);
    return execute(callback, cmd);
}

QvopStatus QvopService::matchAnySecure(sp<IQvopCallback> const& callback,
                                        const char* user_id) {
    logfile_print_v("%s: entry", QVOP_FN);
    QvopCommandMatchAny cmd(user_id, 0);
    return execute(callback, cmd);
}

QvopStatus QvopService::deleteUser(sp<IQvopCallback> const& callback,
                                    const char* user_id) {
    logfile_print_v("%s: entry", QVOP_FN);
    QvopCommandDelete cmd(user_id);
    return execute(callback, cmd);
}

QvopStatus QvopService::deleteWithId(sp<IQvopCallback> const& callback,
                                        const char* user_id,
                                        const char* identifier) {
    logfile_print_v("%s: entry", QVOP_FN);
    QvopCommandDelete cmd(user_id, identifier, false);
    return execute(callback, cmd);
}

QvopStatus QvopService::deleteAll(sp<IQvopCallback> const& callback,
                                    const char* user_id) {
    logfile_print_v("%s: entry", QVOP_FN);
    QvopCommandDelete cmd(user_id, 0, true);
    return execute(callback, cmd);
}

//this API needs to change back up to Java. Should not assume 16bit bit depth
QvopStatus QvopService::processFrame(sp<IQvopCallback> const& callback,
                                        int64_t timestamp,
                                        int32_t bufferlen,
                                        int16_t const * buffer) {

    uint64_t frameId = 0;
    return processFrameWithId(callback, timestamp, frameId, bufferlen, buffer);
}

//this API needs to change back up to Java. Should not assume 16bit bit depth
QvopStatus QvopService::processFrameWithId(sp<IQvopCallback> const& callback,
                                            int64_t timestamp,
                                            int64_t frameId,
                                            int32_t bufferlen,
                                            int16_t const * buffer) {

    //modified buflen - accounting for 16 bit assumption of parameters
    size_t modBufLen = bufferlen * sizeof(int16_t);
    logfile_print_v("%s: entry ts: %ul, frameId: %ul, bufferlen=%d, sample size: %d", QVOP_FN, timestamp, frameId, bufferlen);
    QvopCommandProcessFrame cmd(timestamp, frameId, mSampleRate, mNumberChannels, mAudioFormat, modBufLen, (int8_t const *)buffer);
    return execute(callback, cmd);
}

QvopStatus QvopService::onStartAudioCapture(sp<IQvopCallback> const& callback,
                                                int32_t sampleRate,
                                                int32_t numberOfChannels,
                                                int32_t audioFormat) {
    logfile_print_v("%s: entry sampleRate=%d, numberOfChannels=%d, audioFormat=%d", QVOP_FN, sampleRate, numberOfChannels, audioFormat);

    mSampleRate = sampleRate;
    mNumberChannels = numberOfChannels;
    mAudioFormat = audioFormat;

    return QVOP_SUCCESS;
}

QvopStatus QvopService::onStopAudioCapture(sp<IQvopCallback> const& callback) {
    logfile_print_v("%s: entry", QVOP_FN);
    return QVOP_SUCCESS;
}

QvopStatus QvopService::execute(sp<IQvopCallback> const& callback, QvopCommandService& target) {
    logfile_print_v("%s: entry", QVOP_FN);
    if (mIsShutdown) {
        logfile_print_w("%s: called when shutdown", QVOP_FN);
    return QVOP_FAILURE;
    }

    //link for death recipient
    link(callback);

    //NULL pattern without the factory.
    qvop::IQvopCallback* iqcb = static_cast<qvop::IQvopCallback*>(callback.get());
    if (iqcb == NULL) {
        QvopNullCallback qcb;
        iqcb = &qcb;
    }

    target.executeCmd(*iqcb);

    //need to clear the callback to decrement ref count in sp<>
    //param is a const, so const_cast is one way to defeat this problem.
    //
    sp<IQvopCallback> cb = const_cast<sp<IQvopCallback>&>(callback);
    cb.clear();

    //unlink from death recipient
    unlink(callback);

    logfile_print_v("%s: exit", QVOP_FN);
    return QVOP_SUCCESS;
}

QvopStatus QvopService::cancel() {
    logfile_print_v("%s: entry", QVOP_FN);

    QvopStatus res = QvopSessionEnd();
    if (QVOP_SUCCESS != res) {
        logfile_print_v("QvopSessionEnd failed");
    } else {
        logfile_print_d("QvopSessionEnd success");
    }

    logfile_print_v("%s: ret=%d", QVOP_FN, res);

    return QVOP_SUCCESS;
}

status_t QvopService::onTransact(uint32_t code, const Parcel & data, Parcel * reply, uint32_t flags) {
    logfile_print_v("%s: entry... returning", QVOP_FN);
    return BnQvopService::onTransact(code, data, reply, flags);
}


sp<QvopService> QvopService::instantiate() {
    logfile_print_v("%s: getServiceName()=%s", QVOP_FN, IQvopService::getServiceName());

    // create and init service
    sp<QvopService> qvopService(new QvopService());

    // register service
    sp<IServiceManager> sm(defaultServiceManager());
    sm->addService(String16(IQvopService::getServiceName()), qvopService, false);

    return qvopService;
}

void QvopService::binderDied(const wp<IBinder>& who) {
    logfile_print_v("%s: callback binder died, cancel", QVOP_FN);

    sp<IBinder> binder = who.promote();
    if (binder != NULL) {
        binder->unlinkToDeath(this);
    }
}

}; // namespace android
