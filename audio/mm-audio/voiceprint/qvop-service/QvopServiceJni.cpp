/*
 * Copyright (c) 2015-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#define LOG_TAG "QVOP"

#include <android/log.h>
#include <utils/Log.h>
#include <binder/IServiceManager.h>
#include "android_runtime/AndroidRuntime.h"

#include "jni.h"
#include "IQvopServiceAndroid.h"
#include "QvopCallbackAndroid.h"
#include <cutils/properties.h>
#include <string>
#include <vector>

using namespace android;

#include "QvopJniHelper.h"

/**
 * The JNI binder callback.
 */
class QvopJniCallback : public BnQvopCallback {
public:

  QvopJniCallback() :
    mListenerCallback(0)
  {
  }

  virtual ~QvopJniCallback() {
    setListeningCallback(0);
  }

  void setListeningCallback(jobject callback) {

    JNIEnv* env = android::AndroidRuntime::getJNIEnv();

    // remove existing callback
    if (mListenerCallback != 0) {
      env->DeleteGlobalRef(mListenerCallback);
      mListenerCallback = 0;
    }

    // Setup Java Callbacks
    if (callback != 0) {
      mListenerCallback = env->NewGlobalRef(callback);
      jclass cls = env->GetObjectClass(mListenerCallback);

      mListenerOnResult = env->GetMethodID(cls, "onResult", "(I)V");
      if ( NULL == mListenerOnResult ) {
        logfile_print_e("mListenerOnResult is NULL. Method may not exist.");
      }

      mListenerOnError = env->GetMethodID(cls, "onError", "(I)V");
      if ( NULL == mListenerOnError ) {
        logfile_print_e("mListenerOnError is NULL. Method may not exist.");
      }

      mListenerOnMatchStatus = env->GetMethodID(cls, "onMatchStatus", "(ILjava/lang/String;Ljava/lang/String;IDDF)V");
      if ( NULL == mListenerOnMatchStatus ) {
        logfile_print_e("mListenerOnMatchStatus is NULL. Method may not exist.");
      }

      mListenerOnEnrollStatus = env->GetMethodID(cls, "onEnrollStatus", "(I)V");
      if ( NULL == mListenerOnEnrollStatus ) {
        logfile_print_e("mListenerOnEnrollStatus is NULL. Method may not exist.");
      }

      mListenerOnDeleteStatus = env->GetMethodID(cls, "onDeleteStatus", "(ILjava/lang/String;Ljava/lang/String;)V");
      if ( NULL == mListenerOnDeleteStatus ) {
        logfile_print_e("mListenerOnDeleteStatus is NULL. Method may not exist.");
      }

      mListenerOnVersionInfo = env->GetMethodID(cls, "onVersionInfo", "(JJ)V");
      if ( NULL == mListenerOnVersionInfo ) {
        logfile_print_e("mListenerOnVersionInfo is NULL. Method may not exist.");
      }

      mListenerOnConfigUpdateStatus = env->GetMethodID(cls, "onConfigUpdateStatus", "(I)V");
      if ( NULL == mListenerOnConfigUpdateStatus ) {
        logfile_print_e("mListenerOnConfigUpdateStatus is NULL. Method may not exist.");
      }

      mListenerOnRenameStatus = env->GetMethodID(cls, "onRenameStatus", "(I)V");
      if ( NULL == mListenerOnRenameStatus ) {
        logfile_print_e("mListenerOnRenameStatus is NULL. Method may not exist.");
      }

    }
  }
  virtual void onResult(int32_t result) {
    logfile_print_d("%s: onResult %d", QVOP_FN, result);

    if (mListenerCallback != 0 && mListenerOnResult != NULL) {
      JNIEnv* env = android::AndroidRuntime::getJNIEnv();
      env->CallVoidMethod(mListenerCallback, mListenerOnResult, result);
    }

  }

  virtual void onError(int32_t error) {
    logfile_print_d("%s: onError %d", QVOP_FN, error);

    if (mListenerCallback != 0 && mListenerOnError != NULL) {
      JNIEnv* env = android::AndroidRuntime::getJNIEnv();
      env->CallVoidMethod(mListenerCallback, mListenerOnError, error);
    }
  }

  // On a match or match any successful search
  virtual void onMatchFound(const char* user_id, const char* identifier,
                            int32_t index, double sentence_score, double user_score, float spoofScore) {
    logfile_print_d("%s: onMatchFound user=%s, id=%s, index=%d, sentence_score=%lf, user_score=%lf, spoofScore=%f", QVOP_FN,
          user_id, identifier, index, sentence_score, user_score, spoofScore);

    if (mListenerCallback != 0 && mListenerOnMatchStatus != NULL) {
      JNIEnv* env = android::AndroidRuntime::getJNIEnv();
      jstring userStr = env->NewStringUTF(user_id);
      jstring identifierStr = env->NewStringUTF(identifier);
      env->CallVoidMethod(mListenerCallback,
                          mListenerOnMatchStatus,
                          QVOP_SUCCESS,
                          userStr,
                          identifierStr,
                          index,
                          sentence_score,
                          user_score,
                          spoofScore);
      env->DeleteLocalRef(userStr);
      env->DeleteLocalRef(identifierStr);
    }
  }

  // On a match or match any failed
  virtual void onMatchNotFound(int32_t error, int32_t index, double sentence_score, double user_score, float spoofScore)  {
    logfile_print_d("%s: onMatchNotFound, error=%d, index=%d, sentence_score=%lf, user_score=%lf, spoofScore=%f", QVOP_FN, error,
          index, sentence_score, user_score, spoofScore);

    if (mListenerCallback != 0 && mListenerOnMatchStatus != NULL) {
      JNIEnv* env = android::AndroidRuntime::getJNIEnv();
      env->CallVoidMethod(mListenerCallback,
                          mListenerOnMatchStatus,
                          error,
                          NULL,
                          NULL,
                          index,
                          sentence_score,
                          user_score,
                          spoofScore);
    }
  }


  // Enrollment status
  virtual void onEnrollStatus(int32_t status) {
    logfile_print_d("%s: status=%d", QVOP_FN, status);

    if (mListenerCallback != 0 && mListenerOnEnrollStatus != NULL) {
      JNIEnv* env = android::AndroidRuntime::getJNIEnv();
      env->CallVoidMethod(mListenerCallback, mListenerOnEnrollStatus, status);
    }
  }


  // Delete Successful - identifier
  virtual void onDeleteStatus(int32_t status, const char* user_id, const char* identifier) {
    logfile_print_d("%s: onDeleteSuccess user=%s, id=%s", QVOP_FN, user_id, identifier);

    if (mListenerCallback != 0 && mListenerOnDeleteStatus != NULL) {
      JNIEnv* env = android::AndroidRuntime::getJNIEnv();
      jstring userStr = env->NewStringUTF(user_id);
      jstring identifierStr = env->NewStringUTF(identifier);
      env->CallVoidMethod(mListenerCallback, mListenerOnDeleteStatus, status, userStr, identifierStr);
      env->DeleteLocalRef(userStr);
      env->DeleteLocalRef(identifierStr);
    }
  }

  // Delete Successful - identifier
  virtual void onDeleteAllStatus(int32_t status) {
    logfile_print_d("%s: onDeleteAllSuccess", QVOP_FN);

    if (mListenerCallback != 0 && mListenerOnDeleteStatus != NULL) {
      JNIEnv* env = android::AndroidRuntime::getJNIEnv();
      env->CallVoidMethod(mListenerCallback, mListenerOnDeleteStatus, status, NULL, NULL);
    }
  }



  // Version info
  virtual void onVersionInfo(long major, long minor) {
    logfile_print_d("%s: onVersionInfo", QVOP_FN);
    if (mListenerCallback != 0 && mListenerOnVersionInfo != NULL) {
      JNIEnv* env = android::AndroidRuntime::getJNIEnv();

      env->CallVoidMethod(mListenerCallback, mListenerOnVersionInfo, major, minor);

    }
  }


  // Config setting success
  virtual void onConfigStatus(int32_t status) {
    logfile_print_d("%s: onConfigSuccess", QVOP_FN);

    if (mListenerCallback != 0 && mListenerOnConfigUpdateStatus != NULL) {
      JNIEnv* env = android::AndroidRuntime::getJNIEnv();
      env->CallVoidMethod(mListenerCallback, mListenerOnConfigUpdateStatus, status);
    }
  }

  // Rename successful
  virtual void onRenameStatus(int32_t status) {
    logfile_print_d("%s: ", QVOP_FN);

    if (mListenerCallback != 0 && mListenerOnRenameStatus != NULL) {
      JNIEnv* env = android::AndroidRuntime::getJNIEnv();
      env->CallVoidMethod(mListenerCallback, mListenerOnRenameStatus, status);
    }
  }


private:

  // listener callbacks
  jobject mListenerCallback;
  jmethodID mListenerOnResult;
  jmethodID mListenerOnError;
  jmethodID mListenerOnMatchStatus;
  jmethodID mListenerOnEnrollStatus;
  jmethodID mListenerOnDeleteStatus;
  jmethodID mListenerOnConfigUpdateStatus;
  jmethodID mListenerOnRenameStatus;
  jmethodID mListenerOnVersionInfo;

};

/**
 * The JNI context
 */
struct QvopJniContext {
  sp<IQvopService> mService;

  QvopJniContext(JNIEnv* env, sp<IQvopService> const& service) :
    mService(service)
  {
  }

  virtual ~QvopJniContext() {
  }
};

/**
 * Get the context.
 */
static QvopJniContext* GetContext(jlong context) {
  // the context is a pointer to QvopJniContext
  return reinterpret_cast<QvopJniContext*>(context);
}

/**
 * Get the service interface from the context.
 */
static sp<IQvopService> const& GetService(jlong context) {
  return GetContext(context)->mService;
}

extern "C"
{

  /**
   * Public API
   */

  JNIEXPORT jlong JNICALL
  Java_com_qualcomm_qti_biometrics_voiceprint_service_Native_open(
    JNIEnv* env, jobject thiz) {
    logfile_print_d("nativeOpen");

    // connect to service
    sp<IQvopService> service;
    status_t res = getService(String16(IQvopService::getServiceName()), &service);
    if (res != NO_ERROR) {
      // service connect failed
      logfile_print_e("nativeConnect failed to connect to %s %d", IQvopService::getServiceName(), res);
      return 0;
    }

    logfile_print_e("nativeConnect success, connected to %s", IQvopService::getServiceName());

    // the context is a pointer to QvopJniContext
    QvopJniContext* context = new QvopJniContext(env, service);
    logfile_print_d("context=%p", context);

    return reinterpret_cast<jlong>(context);
  }

  JNIEXPORT void JNICALL
  Java_com_qualcomm_qti_biometrics_voiceprint_service_Native_close(
    JNIEnv* env, jobject thiz, jlong context) {
    logfile_print_d("nativeClose");

    delete GetContext(context);
  }

  JNIEXPORT jint JNICALL
  Java_com_qualcomm_qti_biometrics_voiceprint_service_Native_shutdown(
    JNIEnv* env,
    jobject thiz,
    jlong context) {
    logfile_print_d("nativeShutdown");

    jint rc = NO_ERROR;

    // jint rc = GetService(context)->shutdown();
    if (rc != NO_ERROR) {
      logfile_print_e("nativeShutdown error %d", rc);
    }

    return rc;
  }

  JNIEXPORT jint JNICALL
  Java_com_qualcomm_qti_biometrics_voiceprint_service_Native_enroll(
    JNIEnv* env,
    jobject thiz,
    jlong context,
    jobject receiver,
    jstring user,
    jstring identifier,
    jstring phrase) {
    logfile_print_d("nativeEnroll");

    sp<QvopJniCallback> jnicb = new QvopJniCallback();
    jnicb->setListeningCallback(receiver);

    jint rc = GetService(context)->enroll(jnicb,
                                          GetJavaString(env, user),
                                          GetJavaString(env, identifier),
                                          GetJavaString(env, phrase ));

    if (rc != QVOP_SUCCESS) {
      logfile_print_e("nativeEnroll error %d", rc);
    }

    return rc;
  }

  JNIEXPORT jint JNICALL
  Java_com_qualcomm_qti_biometrics_voiceprint_service_Native_setThreshold(
    JNIEnv* env,
    jobject thiz,
    jlong context,
    jobject receiver,
    jstring user,
    jstring identifier,
    jint threshold) {
    logfile_print_d("nativeSetThreshold");

    sp<QvopJniCallback> jnicb = new QvopJniCallback();
    jnicb->setListeningCallback(receiver);

    jint rc = GetService(context)->setThreshold(jnicb,
              GetJavaString(env, user),
              GetJavaString(env, identifier),
              threshold);

    if (rc != QVOP_SUCCESS) {
      logfile_print_e("nativeSetThreshold error %d", rc);
    }

    return rc;
  }


  JNIEXPORT jint JNICALL
  Java_com_qualcomm_qti_biometrics_voiceprint_service_Native_match(
    JNIEnv* env,
    jobject thiz,
    jlong context,
    jobject receiver,
    jstring user) {
    logfile_print_d("nativeMatch");

    sp<QvopJniCallback> jnicb = new QvopJniCallback();
    jnicb->setListeningCallback(receiver);
    logfile_print_d("match call");
    jint rc = GetService(context)->match(jnicb,
                                         GetJavaString(env, user));

    logfile_print_d("match exit");
    if (rc != NO_ERROR) {
      logfile_print_e("nativeMatch error %d", rc);
    }

    return rc;
  }

  JNIEXPORT jint JNICALL
  Java_com_qualcomm_qti_biometrics_voiceprint_service_Native_matchWithId(
    JNIEnv* env,
    jobject thiz,
    jlong context,
    jobject receiver,
    jstring user,
    jstring identifier) {
    logfile_print_d("nativeMatchWithId");
    logfile_print_d("context=%d", context);
    sp<QvopJniCallback> jnicb = new QvopJniCallback();
    jnicb->setListeningCallback(receiver);
    logfile_print_d("matchWithId call");
    jint rc = GetService(context)->matchWithId(jnicb,
              GetJavaString(env, user),
              GetJavaString(env, identifier));

    logfile_print_v("matchWithId exit");
    if (rc != NO_ERROR) {
      logfile_print_e("nativeMatchWithId error %d", rc);
    }

    return rc;
  }


  JNIEXPORT jint JNICALL
  Java_com_qualcomm_qti_biometrics_voiceprint_service_Native_matchSecure(
    JNIEnv* env,
    jobject thiz,
    jlong context,
    jobject receiver,
    jstring user) {
    logfile_print_d("nativeMatchSecure");

    sp<QvopJniCallback> jnicb = new QvopJniCallback();
    jnicb->setListeningCallback(receiver);

    jint rc = GetService(context)->matchSecure(jnicb,
              GetJavaString(env, user));


    if (rc != NO_ERROR) {
      logfile_print_e("nativeMatchSecure error %d", rc);
    }

    return rc;
  }

  JNIEXPORT jint JNICALL
  Java_com_qualcomm_qti_biometrics_voiceprint_service_Native_matchAny(
    JNIEnv* env,
    jobject thiz,
    jlong context,
    jobject receiver,
    jstring user) {
    logfile_print_d("nativeMatchAny");

    sp<QvopJniCallback> jnicb = new QvopJniCallback();
    jnicb->setListeningCallback(receiver);

    jint rc = GetService(context)->matchAny(jnicb,
                                            GetJavaString(env, user));


    if (rc != NO_ERROR) {
      logfile_print_e("nativeMatchAny error %d", rc);
    }

    return rc;
  }

  JNIEXPORT jint JNICALL
  Java_com_qualcomm_qti_biometrics_voiceprint_service_Native_matchAnyKeyPhrase(
    JNIEnv* env,
    jobject thiz,
    jlong context,
    jobject receiver,
    jstring user,
    jstring phrase) {
    logfile_print_d("nativeMatchAnyKeyPhrase");

    sp<QvopJniCallback> jnicb = new QvopJniCallback();
    jnicb->setListeningCallback(receiver);

    jint rc = GetService(context)->matchAnyKeyPhrase(jnicb,
              GetJavaString(env, user),
              GetJavaString(env, phrase));

    if (rc != NO_ERROR) {
      logfile_print_e("nativeMatchAnyKeyPhrase error %d", rc);
    }

    return rc;
  }

  JNIEXPORT jint JNICALL
  Java_com_qualcomm_qti_biometrics_voiceprint_service_Native_matchAnySecure(
    JNIEnv* env,
    jobject thiz,
    jlong context,
    jobject receiver,
    jstring user) {
    logfile_print_d("nativeMatchAnySecure");

    sp<QvopJniCallback> jnicb = new QvopJniCallback();
    jnicb->setListeningCallback(receiver);

    jint rc = GetService(context)->matchAnySecure(jnicb,
              GetJavaString(env, user));


    if (rc != NO_ERROR) {
      logfile_print_e("nativeMatchAnySecure error %d", rc);
    }

    return rc;
  }


  JNIEXPORT jint JNICALL
  Java_com_qualcomm_qti_biometrics_voiceprint_service_Native_cancel(
    JNIEnv* env,
    jobject thiz,
    jlong context) {
    logfile_print_d("nativeCancel");

    jint rc = GetService(context)->cancel();
    if (rc != NO_ERROR) {
      logfile_print_e("nativeCancel error %d", rc);
    }

    return rc;
  }

  JNIEXPORT jint JNICALL
  Java_com_qualcomm_qti_biometrics_voiceprint_service_Native_delete(
    JNIEnv* env,
    jobject thiz,
    jlong context,
    jobject receiver,
    jstring user) {
    logfile_print_d("nativeDelete");

    sp<QvopJniCallback> jnicb = new QvopJniCallback();
    jnicb->setListeningCallback(receiver);

    // remove all voiceprint templates for user
    jint rc = GetService(context)->deleteUser(jnicb,
              GetJavaString(env, user));

    if (rc != NO_ERROR) {
      logfile_print_e("nativeDelete error %d", rc);
    }

    return rc;
  }

  JNIEXPORT jint JNICALL
  Java_com_qualcomm_qti_biometrics_voiceprint_service_Native_deleteWithId(
    JNIEnv* env,
    jobject thiz,
    jlong context,
    jobject receiver,
    jstring user,
    jstring identifier) {
    logfile_print_d("nativeDeleteWithId");

    sp<QvopJniCallback> jnicb = new QvopJniCallback();
    jnicb->setListeningCallback(receiver);

    // remove all voiceprint templates for user
    jint rc = GetService(context)->deleteWithId(jnicb,
              GetJavaString(env, user),
              GetJavaString(env, identifier));

    if (rc != NO_ERROR) {
      logfile_print_e("nativeDeleteWithId error %d", rc);
    }

    return rc;
  }


  JNIEXPORT jint JNICALL
  Java_com_qualcomm_qti_biometrics_voiceprint_service_Native_deleteAll(
    JNIEnv* env,
    jobject thiz,
    jlong context,
    jobject receiver,
    jstring user) {
    logfile_print_d("nativeDeleteAll");

    sp<QvopJniCallback> jnicb = new QvopJniCallback();
    jnicb->setListeningCallback(receiver);

    // remove all voiceprint templates for user
    jint rc = GetService(context)->deleteAll(jnicb,
              GetJavaString(env, user));

    if (rc != NO_ERROR) {
      logfile_print_e("nativeDeleteAll error %d", rc);
    }

    return rc;
  }

  JNIEXPORT jint JNICALL
  Java_com_qualcomm_qti_biometrics_voiceprint_service_Native_processFrame(
    JNIEnv *env,
    jobject thiz,
    jlong context,
    jobject receiver,
    jlong timeStamp,
    jint bufferLen,
    jshortArray frame)
  {
    logfile_print_v("nativeProcessFrame bufferLen=%d", bufferLen);
    sp<QvopJniCallback> jnicb = new QvopJniCallback();
    jnicb->setListeningCallback(receiver);

    jshort* buffer = env->GetShortArrayElements(frame, 0);

#ifdef QVOP_DEBUG
    logfile_print_v("Frame[0]: %d", buffer[0]);
    logfile_print_v("Frame[1]: %d", buffer[1]);
    logfile_print_v("Frame[%d]: %d", bufferLen - 1, buffer[bufferLen - 1]);
#endif
    jint rc = GetService(context)->processFrame(jnicb,
              timeStamp, bufferLen, reinterpret_cast<int16_t const*>(buffer));

    env->ReleaseShortArrayElements(frame, buffer, JNI_ABORT /* no copy-back */);

    if (rc != NO_ERROR) {
      logfile_print_e("nativeProcessFrame error %d", rc);
    }

    return rc;
  }


  JNIEXPORT jint JNICALL
  Java_com_qualcomm_qti_biometrics_voiceprint_service_Native_processFrameWithId(
    JNIEnv *env,
    jobject thiz,
    jlong context,
    jobject receiver,
    jlong timeStamp,
    jlong frameId,
    jint bufferLen,
    jshortArray frame)
  {
    logfile_print_v("nativeProcessFrameWithId bufferLen=%d", bufferLen);
    sp<QvopJniCallback> jnicb = new QvopJniCallback();
    jnicb->setListeningCallback(receiver);


    jshort* buffer = env->GetShortArrayElements(frame, 0);
#ifdef QVOP_DEBUG
    logfile_print_v("FrameId: %d", frameId);
    logfile_print_v("Frame[0]: %d", buffer[0]);
    logfile_print_v("Frame[1]: %d", buffer[1]);
    logfile_print_v("Frame[%d]: %d", bufferLen, buffer[bufferLen - 1]);
#endif

    jint rc = GetService(context)->processFrameWithId(jnicb,
              timeStamp, frameId, bufferLen,  reinterpret_cast<int16_t const*>(buffer));

    env->ReleaseShortArrayElements(frame, buffer, JNI_ABORT /* no copy-back */);

    if (rc != NO_ERROR) {
      logfile_print_e("nativeProcessFrameWithId error %d", rc);
    }

    return rc;
  }


  JNIEXPORT jint JNICALL
  Java_com_qualcomm_qti_biometrics_voiceprint_service_Native_onStartAudioCapture(
    JNIEnv *env,
    jobject thiz,
    jlong context,
    jobject receiver,
    jint sampleRate,
    jint numberOfChannels,
    jint audioFormat)
  {
    logfile_print_v("nativeOnStartAudioCapture");
    sp<QvopJniCallback> jnicb = new QvopJniCallback();
    jnicb->setListeningCallback(receiver);

    jint rc = GetService(context)->onStartAudioCapture(jnicb,
              sampleRate, numberOfChannels, audioFormat);

    if (rc != NO_ERROR) {
      logfile_print_e("nativeOnStartAudioCapture error %d", rc);
    }

    return rc;
  }

  JNIEXPORT jint JNICALL
  Java_com_qualcomm_qti_biometrics_voiceprint_service_Native_onStopAudioCapture(
    JNIEnv *env,
    jobject thiz,
    jlong context,
    jobject receiver)
  {
    logfile_print_v("nativeOnStopAudioCapture");
    sp<QvopJniCallback> jnicb = new QvopJniCallback();
    jnicb->setListeningCallback(receiver);

    jint rc = GetService(context)->onStopAudioCapture(jnicb);

    if (rc != NO_ERROR) {
      logfile_print_e("nativeOnStopAudioCapture error %d", rc);
    }

    return rc;
  }

  JNIEXPORT jint JNICALL
  Java_com_qualcomm_qti_biometrics_voiceprint_service_Native_enrollCaptureStart(
    JNIEnv *env,
    jobject thiz,
    jlong context,
    jobject receiver)
  {
    logfile_print_v("nativeEnrollCaptureStart");
    sp<QvopJniCallback> jnicb = new QvopJniCallback();
    jnicb->setListeningCallback(receiver);

    jint rc = GetService(context)->enrollCaptureStart(jnicb);

    if (rc != NO_ERROR) {
      logfile_print_e("nativeEnrollCaptureStart error %d", rc);
    }

    return rc;
  }


  JNIEXPORT jint JNICALL
  Java_com_qualcomm_qti_biometrics_voiceprint_service_Native_enrollCaptureComplete(
    JNIEnv *env,
    jobject thiz,
    jlong context,
    jobject receiver)
  {
    logfile_print_v("nativeEnrollCaptureComplete");
    sp<QvopJniCallback> jnicb = new QvopJniCallback();
    jnicb->setListeningCallback(receiver);

    jint rc = GetService(context)->enrollCaptureComplete(jnicb);

    if (rc != NO_ERROR) {
      logfile_print_e("nativeEnrollCaptureComplete error %d", rc);
    }

    return rc;
  }

  JNIEXPORT jint JNICALL
  Java_com_qualcomm_qti_biometrics_voiceprint_service_Native_enrollCommit(
    JNIEnv *env,
    jobject thiz,
    jlong context,
    jobject receiver)
  {
    logfile_print_v("nativeEnrollCommit");
    sp<QvopJniCallback> jnicb = new QvopJniCallback();
    jnicb->setListeningCallback(receiver);

    jint rc = GetService(context)->enrollCommit(jnicb);

    if (rc != NO_ERROR) {
      logfile_print_e("nativeEnrollCommit error %d", rc);
    }

    return rc;
  }

  JNIEXPORT jint JNICALL
  Java_com_qualcomm_qti_biometrics_voiceprint_service_Native_matchCaptureComplete(
    JNIEnv *env,
    jobject thiz,
    jlong context,
    jobject receiver)
  {
    logfile_print_v("nativeMatchCaptureComplete");
    sp<QvopJniCallback> jnicb = new QvopJniCallback();
    jnicb->setListeningCallback(receiver);

    jint rc = GetService(context)->matchCaptureComplete(jnicb);

    if (rc != NO_ERROR) {
      logfile_print_e("nativeMatchCaptureComplete error %d", rc);
    }

    return rc;
  }


  JNIEXPORT jint JNICALL
  Java_com_qualcomm_qti_biometrics_voiceprint_service_Native_rename(
    JNIEnv *env,
    jobject thiz,
    jlong context,
    jobject receiver,
    jstring user,
    jstring old_id,
    jstring new_id
  )
  {
    logfile_print_v("nativeRename");
    sp<QvopJniCallback> jnicb = new QvopJniCallback();
    jnicb->setListeningCallback(receiver);

    jint rc = GetService(context)->rename(jnicb,
                                          GetJavaString(env, user),
                                          GetJavaString(env, old_id),
                                          GetJavaString(env, new_id));

    if (rc != NO_ERROR) {
      logfile_print_e("nativeRename error %d", rc);
    }

    return rc;
  }


#ifdef QVOP_DEBUG

// Add any debug APIs here

#endif


} // extern "C"
