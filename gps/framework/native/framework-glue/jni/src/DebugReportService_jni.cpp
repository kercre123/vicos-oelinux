/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
  Copyright (c) 2017 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.
=============================================================================*/


#define LOG_TAG "DebugReportService_jni"
#define LOG_NDEBUG 0
#define LOCATION_CLASS_NAME "android/location/Location"

#include "JNIHelp.h"
#include "jni.h"
#include "android_runtime/AndroidRuntime.h"
#include "android_runtime/Log.h"
#include "LocDualContext.h"
#include "SystemStatus.h"

using namespace std;
using namespace android;
using namespace loc_core;

const SystemStatus* gSystemStatus;

static jclass arrayListClass;
static jclass IzatEphmerisDebugReportClass;
static jclass IzatFixStatusDebugReportClass;
static jclass IzatLocationReportClass;
static jclass IzatGpsTimeReportClass;
static jclass IzatXoStateReportClass;
static jclass IzatRfStateReportClass;
static jclass IzatUtcSpecClass;
static jclass IzatPDRReportClass;
static jclass IzatSVHealthReportClass;
static jclass IzatXTRAReportClass;
static jclass IzatErrorRecoveryClass;

static jmethodID method_addToList;
static jmethodID method_ephmerisDebugReportCtor;
static jmethodID method_fixStatusDebugReportCtor;
static jmethodID method_locationReportCtor;
static jmethodID method_gpsTimeReportCtor;
static jmethodID method_xoStateReportCtor;
static jmethodID method_rfStateReportCtor;
static jmethodID method_utcSpecCtor;
static jmethodID method_pdrReportCtor;
static jmethodID method_svhealthReportCtor;
static jmethodID method_xtraReportCtor;
static jmethodID method_errorRecoveryCtor;

static void DebugReportService_class_init(JNIEnv* env, jclass clazz)
{
    ALOGD("DebugReportService_class_init");

    // get references to the Java method
    jclass arrayListClassLocal;
    jclass IzatEphmerisClassLocal;
    jclass IzatFixStatusClassLocal;
    jclass IzatLocationReportClassLocal;
    jclass IzatGpsTimeReportClassLocal;
    jclass IzatXoStateReportClassLocal;
    jclass IzatRfStateReportClassLocal;
    jclass IzatUtcSpecClassLocal;
    jclass IzatPDRReportClassLocal;
    jclass IzatSVHealthReportClassLocal;
    jclass IzatXTRAReportClassLocal;
    jclass IzatErrorRecoveryClassLocal;

    arrayListClassLocal = env->FindClass("java/util/ArrayList");
    method_addToList  = env->GetMethodID(arrayListClassLocal, "add", "(Ljava/lang/Object;)Z");
    arrayListClass = (jclass) env->NewGlobalRef(arrayListClassLocal);
    env->DeleteLocalRef(arrayListClassLocal);

    IzatEphmerisClassLocal = env->FindClass("com/qti/debugreport/IZatEphmerisDebugReport");
    if (IzatEphmerisClassLocal == nullptr) {
        ALOGE("Cannot find IzatEphmerisDebugReportClass class");
    } else {
        IzatEphmerisDebugReportClass = (jclass) env->NewGlobalRef(IzatEphmerisClassLocal);
        method_ephmerisDebugReportCtor = env->GetMethodID(IzatEphmerisDebugReportClass, "<init>",
            "(Lcom/qti/debugreport/IZatUtcSpec;Lcom/qti/debugreport/IZatUtcSpec;IIJJB)V");
        if (method_ephmerisDebugReportCtor == 0) {
            ALOGE("Failed to get constructor method for IzatEphmerisDebugReportClass");
        }
        env->DeleteLocalRef(IzatEphmerisClassLocal);
    }

    IzatFixStatusClassLocal = env->FindClass("com/qti/debugreport/IZatFixStatusDebugReport");
    if (IzatFixStatusClassLocal == nullptr) {
        ALOGE("Cannot find IZatFixStatusDebugReport class");
    } else {
        IzatFixStatusDebugReportClass = (jclass)  env->NewGlobalRef(IzatFixStatusClassLocal);
        method_fixStatusDebugReportCtor = env->GetMethodID(IzatFixStatusDebugReportClass, "<init>",
            "(Lcom/qti/debugreport/IZatUtcSpec;Lcom/qti/debugreport/IZatUtcSpec;IJ)V");
        if (method_fixStatusDebugReportCtor == 0) {
            ALOGE("Failed to get constructor method for IZatFixStatusDebugReport");
        }
        env->DeleteLocalRef(IzatFixStatusClassLocal);
    }

    IzatLocationReportClassLocal = env->FindClass("com/qti/debugreport/IZatLocationReport");
    if (IzatLocationReportClassLocal == nullptr) {
        ALOGE("Cannot find IZatLocationReport class");
    } else {
        IzatLocationReportClass = (jclass) env->NewGlobalRef(IzatLocationReportClassLocal);
        method_locationReportCtor = env->GetMethodID(IzatLocationReportClass, "<init>",
            "(Lcom/qti/debugreport/IZatUtcSpec;Lcom/qti/debugreport/IZatUtcSpec;IDDFDFI)V");
        if (method_locationReportCtor == 0) {
            ALOGE("Failed to get constructor method for IZatLocationReport");
        }
        env->DeleteLocalRef(IzatLocationReportClassLocal);
    }

    IzatGpsTimeReportClassLocal = env->FindClass("com/qti/debugreport/IZatGpsTimeDebugReport");
    if (IzatGpsTimeReportClassLocal == nullptr) {
        ALOGE("Cannot find IZatGpsTimeDebugReport class");
    } else {
        IzatGpsTimeReportClass = (jclass) env->NewGlobalRef(IzatGpsTimeReportClassLocal);
        method_gpsTimeReportCtor = env->GetMethodID(IzatGpsTimeReportClass, "<init>",
            "(Lcom/qti/debugreport/IZatUtcSpec;Lcom/qti/debugreport/IZatUtcSpec;IJZIIII)V");
        if (method_gpsTimeReportCtor == 0) {
            ALOGE("Failed to get constructor method for IZatDebugReport");
        }
        env->DeleteLocalRef(IzatGpsTimeReportClassLocal);
    }

    IzatXoStateReportClassLocal = env->FindClass("com/qti/debugreport/IZatXoStateDebugReport");
    if (IzatXoStateReportClassLocal == nullptr) {
        ALOGE("Cannot find IZatXoStateDebugReport class");
    } else {
        IzatXoStateReportClass = (jclass) env->NewGlobalRef(IzatXoStateReportClassLocal);
        method_xoStateReportCtor = env->GetMethodID(IzatXoStateReportClass, "<init>",
            "(Lcom/qti/debugreport/IZatUtcSpec;Lcom/qti/debugreport/IZatUtcSpec;I)V");
        if (method_xoStateReportCtor == 0) {
            ALOGE("Failed to get constructor method for IZatXoStateDebugReport");
        }
        env->DeleteLocalRef(IzatXoStateReportClassLocal);
    }

    IzatRfStateReportClassLocal = env->FindClass("com/qti/debugreport/IZatRfStateDebugReport");
    if (IzatRfStateReportClassLocal == nullptr) {
        ALOGE("Cannot find IZatRfStateDebugReport class");
    } else {
        IzatRfStateReportClass = (jclass) env->NewGlobalRef(IzatRfStateReportClassLocal);
        method_rfStateReportCtor = env->GetMethodID(IzatRfStateReportClass, "<init>",
            "(Lcom/qti/debugreport/IZatUtcSpec;Lcom/qti/debugreport/IZatUtcSpec;IJJJJJJJJ)V");
        if (method_rfStateReportCtor == 0) {
            ALOGE("Failed to get constructor method for IZatRfStateDebugReport");
        }
        env->DeleteLocalRef(IzatRfStateReportClassLocal);
    }

    IzatUtcSpecClassLocal = env->FindClass("com/qti/debugreport/IZatUtcSpec");
    if (IzatUtcSpecClassLocal == nullptr) {
        ALOGE("Cannot find IZatUtcSpec class");
    } else {
        IzatUtcSpecClass = (jclass) env->NewGlobalRef(IzatUtcSpecClassLocal);
        method_utcSpecCtor = env->GetMethodID(IzatUtcSpecClass, "<init>", "(JJ)V");
        if (method_utcSpecCtor == 0) {
            ALOGE("Failed to get constructor method for IZatUtcSpec");
        }
        env->DeleteLocalRef(IzatUtcSpecClassLocal);
    }

    IzatPDRReportClassLocal = env->FindClass("com/qti/debugreport/IZatPDRDebugReport");
    if (IzatPDRReportClassLocal == nullptr) {
        ALOGE("Cannot find IZatPDRDebugReport class");
    } else {
        IzatPDRReportClass = (jclass) env->NewGlobalRef(IzatPDRReportClassLocal);
        method_pdrReportCtor = env->GetMethodID(IzatPDRReportClass, "<init>",
            "(Lcom/qti/debugreport/IZatUtcSpec;Lcom/qti/debugreport/IZatUtcSpec;I)V");
        if (method_pdrReportCtor == 0) {
            ALOGE("Failed to get constructor method for IZatPDRDebugReport");
        }
        env->DeleteLocalRef(IzatPDRReportClassLocal);
    }

    IzatSVHealthReportClassLocal = env->FindClass("com/qti/debugreport/IZatSVHealthDebugReport");
    if (IzatSVHealthReportClassLocal == nullptr) {
        ALOGE("Cannot find IZatSVHealthDebugReport class");
    } else {
        IzatSVHealthReportClass = (jclass) env->NewGlobalRef(IzatSVHealthReportClassLocal);
        method_svhealthReportCtor = env->GetMethodID(IzatSVHealthReportClass, "<init>",
            "(Lcom/qti/debugreport/IZatUtcSpec;Lcom/qti/debugreport/IZatUtcSpec;IIIIIIJJJJJJBBB)V");
        if (method_svhealthReportCtor == 0) {
            ALOGE("Failed to get constructor method for IZatSVHealthDebugReport");
        }
        env->DeleteLocalRef(IzatSVHealthReportClassLocal);
    }

    IzatXTRAReportClassLocal = env->FindClass("com/qti/debugreport/IZatXTRADebugReport");
    if (IzatXTRAReportClassLocal == nullptr) {
        ALOGE("Cannot find IZatXTRADebugReport class");
    } else {
        IzatXTRAReportClass = (jclass) env->NewGlobalRef(IzatXTRAReportClassLocal);
        method_xtraReportCtor = env->GetMethodID(IzatXTRAReportClass, "<init>",
            "(Lcom/qti/debugreport/IZatUtcSpec;Lcom/qti/debugreport/IZatUtcSpec;BIIIIJIJIBI)V");
        if (method_xtraReportCtor == 0) {
            ALOGE("Failed to get constructor method for IZatXTRADebugReport");
        }
        env->DeleteLocalRef(IzatXTRAReportClassLocal);
    }

    IzatErrorRecoveryClassLocal = env->FindClass("com/qti/debugreport/IZatErrorRecoveryReport");
    if (IzatErrorRecoveryClassLocal == nullptr) {
        ALOGE("Cannot find IZatErrorRecoveryReport class");
    } else {
        IzatErrorRecoveryClass = (jclass) env->NewGlobalRef(IzatErrorRecoveryClassLocal);
        method_errorRecoveryCtor = env->GetMethodID(IzatErrorRecoveryClass, "<init>",
            "(Lcom/qti/debugreport/IZatUtcSpec;Lcom/qti/debugreport/IZatUtcSpec;)V");
        if (method_errorRecoveryCtor == 0) {
            ALOGE("Failed to get constructor method for IZatErrorRecoveryReport");
        }
        env->DeleteLocalRef(IzatErrorRecoveryClassLocal);
    }
}

static void DebugReportService_init(JNIEnv* env, jobject obj)
{
    ALOGD("DebugReportService_init");
    gSystemStatus = LocDualContext::getSystemStatus();
    if (gSystemStatus == NULL) {
        ALOGE("Failed to get SystemStatus");
    }
}

static void DebugReportService_deinit(JNIEnv* env, jobject obj)
{
    ALOGD("DebugReportService_deinit");

    env->DeleteGlobalRef(arrayListClass);
    env->DeleteGlobalRef(IzatEphmerisDebugReportClass);
    env->DeleteGlobalRef(IzatFixStatusDebugReportClass);
    env->DeleteGlobalRef(IzatLocationReportClass);
    env->DeleteGlobalRef(IzatGpsTimeReportClass);
    env->DeleteGlobalRef(IzatXoStateReportClass);
    env->DeleteGlobalRef(IzatRfStateReportClass);
    env->DeleteGlobalRef(IzatUtcSpecClass);
    env->DeleteGlobalRef(IzatPDRReportClass);
    env->DeleteGlobalRef(IzatSVHealthReportClass);
    env->DeleteGlobalRef(IzatXTRAReportClass);
}

jobject createUtcTimeObject(JNIEnv* env, jlong utcSecondsPart, jlong utcNanoSecPart) {
    jobject utcObj = env->NewObject(IzatUtcSpecClass, method_utcSpecCtor,
        utcSecondsPart, utcNanoSecPart);
    if (utcObj == nullptr) {
        ALOGE("Failed to create IZatUtcSpec object");
    }

    return utcObj;
}

 static void DebugReportService_getReport(JNIEnv* env, jobject obj,  jint maxReports, jobject ephmerisStatusListObj,
                                          jobject fixStatusListObj, jobject epiReportListObj, jobject bestLocationListObj,
                                          jobject gpsTimeReportListObj, jobject xoStateReportListObj, jobject rfStateReportListObj,
                                          jobject errorRecoveriesListObj, jobject pdrReportListObj, jobject svHealthReportListObj,
                                          jobject xtraReportListObj)
{
    if (maxReports == 1) {
        ALOGD("DebugReportService_getReport, Get Latest 1 sec data.");
    } else if (maxReports != 1) {
        ALOGD("DebugReportService_getReport, Get latest all available data.");
    }

    if (maxReports > 0) {
        SystemStatusReports allReports;
        gSystemStatus->getReport(allReports, maxReports == 1);

        #define HAS_HORIZONTAL_COMPONENT 1
        #define HAS_VERTICAL_COMPONENT 2
        #define HAS_SOURCE 4

        // fill in the ephmeris report
        for (auto p = make_pair(0, allReports.mEphemeris.begin());
             (p.first < maxReports) && (p.second != allReports.mEphemeris.end());
             p.first++, ++p.second) {
            jlong utcSecondsPartLastUpdated = (*p.second).mUtcTime.tv_sec;
            jlong utcNanoSecPartLastUpdated = (*p.second).mUtcTime.tv_nsec;

            jlong utcSecondsPartLastReported = (*p.second).mUtcReported.tv_sec;
            jlong utcNanoSecPartLastReported = (*p.second).mUtcReported.tv_nsec;

            jint gpsEpheValidity = (*p.second).mGpsEpheValid;
            jint glonassEpheValidity = (*p.second).mGloEpheValid;
            jlong bdsEpheValidity = (*p.second).mBdsEpheValid;
            jlong galEpheValidity = (*p.second).mGalEpheValid;
            jbyte qzssEpheValidity = (*p.second).mQzssEpheValid;

            jobject utcLastUpdatedObj = createUtcTimeObject(env,
                utcSecondsPartLastUpdated, utcNanoSecPartLastUpdated);

            jobject utcLastReportedObj = createUtcTimeObject(env,
                utcSecondsPartLastReported, utcNanoSecPartLastReported);

            jobject ephmerisReportObj = env->NewObject(IzatEphmerisDebugReportClass, method_ephmerisDebugReportCtor,
                                                       utcLastUpdatedObj, utcLastReportedObj, gpsEpheValidity, glonassEpheValidity,
                                                       bdsEpheValidity, galEpheValidity, qzssEpheValidity);
            if (ephmerisReportObj == nullptr) {
                ALOGE("Failed to create IzatEphmerisDebugReport object");
            } else {
                jboolean jb = env->CallBooleanMethod(ephmerisStatusListObj,
                                                     method_addToList,
                                                     ephmerisReportObj);
                if (jb  == 0) {
                    ALOGE("Failed to add IzatEphmerisDebugReport object to list");
                }
            }
        }

        // fill in FixStatus Report
        for (auto p = make_pair(0, allReports.mPositionFailure.begin());
             (p.first < maxReports) && (p.second != allReports.mPositionFailure.end());
             p.first++, ++p.second) {
             jlong utcSecondsPartLastUpdated = (*p.second).mUtcTime.tv_sec;
             jlong utcNanoSecPartLastUpdated = (*p.second).mUtcTime.tv_nsec;

            jlong utcSecondsPartLastReported = (*p.second).mUtcReported.tv_sec;
            jlong utcNanoSecPartLastReported = (*p.second).mUtcReported.tv_nsec;

            jint fixInfoMask = (*p.second).mFixInfoMask;
            jlong hepeLimit = (*p.second).mHepeLimit;

            jobject utcLastUpdatedObj = createUtcTimeObject(env,
                utcSecondsPartLastUpdated, utcNanoSecPartLastUpdated);

            jobject utcLastReportedObj = createUtcTimeObject(env,
                utcSecondsPartLastReported, utcNanoSecPartLastReported);

            jobject fixReportObj = env->NewObject(IzatFixStatusDebugReportClass,
                                                  method_fixStatusDebugReportCtor,
                                                  utcLastUpdatedObj, utcLastReportedObj,
                                                  fixInfoMask, hepeLimit);
            if (fixReportObj == nullptr) {
                ALOGE("Failed to create IZatFixStatusDebugReport object");
            } else {
                jboolean jb = env->CallBooleanMethod(fixStatusListObj,
                                                     method_addToList,
                                                     fixReportObj);
                if (jb  == 0) {
                    ALOGE("Failed to add IZatFixStatusDebugReport object");
                }
            }
        }


        // fill in the External poisition injected report
        for (auto p = make_pair(0, allReports.mInjectedPosition.begin());
             (p.first < maxReports) && (p.second != allReports.mInjectedPosition.end());
             p.first++, ++p.second) {
            jlong utcSecondsPartLastUpdated = (*p.second).mUtcTime.tv_sec;
            jlong utcNanoSecPartLastUpdated = (*p.second).mUtcTime.tv_nsec;

            jlong utcSecondsPartLastReported = (*p.second).mUtcReported.tv_sec;
            jlong utcNanoSecPartLastReported = (*p.second).mUtcReported.tv_nsec;

            // EPI always has source field
            jint validityMask = (*p.second).mEpiValidity;
            validityMask |= HAS_SOURCE;

            jdouble lat = (*p.second).mEpiLat;
            jdouble lon = (*p.second).mEpiLon;
            jfloat horzAccuracy = (*p.second).mEpiHepe;
            jdouble alt= (*p.second).mEpiAlt;
            jfloat altUnc = (*p.second).mEpiAltUnc;
            jint source = (*p.second).mEpiSrc;

            jobject utcLastUpdatedObj = createUtcTimeObject(env,
                utcSecondsPartLastUpdated, utcNanoSecPartLastUpdated);

            jobject utcLastReportedObj = createUtcTimeObject(env,
                utcSecondsPartLastReported, utcNanoSecPartLastReported);

            jobject epiReportObj = env->NewObject(IzatLocationReportClass, method_locationReportCtor,
                                                  utcLastUpdatedObj, utcLastReportedObj, validityMask, lat,
                                                  lon, horzAccuracy, alt, altUnc, source);
            if (epiReportObj == nullptr) {
                ALOGE("Failed to create IzatLocationReport object for EPI");
            } else {
                jboolean jb = env->CallBooleanMethod(epiReportListObj,
                                                     method_addToList, epiReportObj);
                if (jb  == 0) {
                    ALOGE("Failed to add IzatLocationReport object for EPI to list");
                }
            }
        }

        // fill in the Best position report
        for (auto p = make_pair(0, allReports.mBestPosition.begin());
             (p.first < maxReports) && (p.second != allReports.mBestPosition.end());
             p.first++, ++p.second) {
            jlong utcSecondsPartLastUpdated = (*p.second).mUtcTime.tv_sec;
            jlong utcNanoSecPartLastUpdated = (*p.second).mUtcTime.tv_nsec;

            jlong utcSecondsPartLastReported = (*p.second).mUtcReported.tv_sec;
            jlong utcNanoSecPartLastReported = (*p.second).mUtcReported.tv_nsec;

            jint validityMask = (HAS_HORIZONTAL_COMPONENT | HAS_VERTICAL_COMPONENT);
            jdouble lat = (*p.second).mBestLat;
            jdouble lon = (*p.second).mBestLon;
            jfloat horzAccuracy = (*p.second).mBestHepe;
            jdouble alt= (*p.second).mBestAlt;
            jfloat altUnc = (*p.second).mBestAltUnc;

            jobject utcLastUpdatedObj = createUtcTimeObject(env,
                utcSecondsPartLastUpdated, utcNanoSecPartLastUpdated);

            jobject utcLastReportedObj = createUtcTimeObject(env,
                utcSecondsPartLastReported, utcNanoSecPartLastReported);

            jobject bestLocReportObj = env->NewObject(IzatLocationReportClass,
                                                      method_locationReportCtor,
                                                      utcLastUpdatedObj, utcLastReportedObj,
                                                      validityMask, lat, lon, horzAccuracy,
                                                      alt, altUnc, 0);
            if (bestLocReportObj == nullptr) {
                ALOGE("Failed to create IzatLocationReport object for Best Location");
            } else {
                jboolean jb = env->CallBooleanMethod(bestLocationListObj,
                                                     method_addToList, bestLocReportObj);
                if (jb  == 0) {
                    ALOGE("Failed to add IzatLocationReport object for BestLocation to list");
                }
            }
        }

        // fill in the GPS time report
        for(auto p = make_pair(0, allReports.mTimeAndClock.begin());
            (p.first < maxReports) && (p.second != allReports.mTimeAndClock.end());
            p.first++, ++p.second) {
            jlong utcSecondsPartLastUpdated = (*p.second).mUtcTime.tv_sec;
            jlong utcNanoSecPartLastUpdated = (*p.second).mUtcTime.tv_nsec;

            jlong utcSecondsPartLastReported = (*p.second).mUtcReported.tv_sec;
            jlong utcNanoSecPartLastReported = (*p.second).mUtcReported.tv_nsec;

            jint gpsWeek = (*p.second).mGpsWeek;
            jlong gpsTimeOfWeekInMs = (*p.second).mGpsTowMs;
            jboolean timeValid = ((*p.second).mTimeValid == 1 ? JNI_TRUE : JNI_FALSE);
            jint  timeSource = (*p.second).mTimeSource;
            jint timeUncertainity = (*p.second).mTimeUnc;
            jint clockFreqBias =  (*p.second).mClockFreqBias;
            jint clockFreBiasUnc = (*p.second).mClockFreqBiasUnc;

            jobject utcLastUpdatedObj = createUtcTimeObject(env,
                utcSecondsPartLastUpdated, utcNanoSecPartLastUpdated);

            jobject utcLastReportedObj = createUtcTimeObject(env,
                utcSecondsPartLastReported, utcNanoSecPartLastReported);

            jobject timeReportObj = env->NewObject(IzatGpsTimeReportClass, method_gpsTimeReportCtor,
                                                   utcLastUpdatedObj, utcLastReportedObj, gpsWeek,
                                                   gpsTimeOfWeekInMs, timeValid, timeSource,
                                                   timeUncertainity, clockFreqBias, clockFreBiasUnc);

            if (timeReportObj == nullptr) {
                ALOGE("Failed to create IzatGpsTimeDebugReport object");
            } else {
                jboolean jb = env->CallBooleanMethod(gpsTimeReportListObj, method_addToList,
                                                     timeReportObj);
                if (jb == JNI_FALSE) {
                    ALOGE("Failed to add IzatGpsTimeDebugReport object");
                }
            }
        }

        for (auto p = make_pair(0, allReports.mXoState.begin());
             (p.first < maxReports) && (p.second != allReports.mXoState.end());
             p.first++, ++p.second) {
            jlong utcSecondsPartLastUpdated = (*p.second).mUtcTime.tv_sec;
            jlong utcNanoSecPartLastUpdated = (*p.second).mUtcTime.tv_nsec;

            jlong utcSecondsPartLastReported = (*p.second).mUtcReported.tv_sec;
            jlong utcNanoSecPartLastReported = (*p.second).mUtcReported.tv_nsec;

            jint xostate = (*p.second).mXoState;

            jobject utcLastUpdatedObj = createUtcTimeObject(env,
                utcSecondsPartLastUpdated, utcNanoSecPartLastUpdated);

            jobject utcLastReportedObj = createUtcTimeObject(env,
                utcSecondsPartLastReported, utcNanoSecPartLastReported);

            jobject xoReportObj = env->NewObject(IzatXoStateReportClass, method_xoStateReportCtor,
                                                 utcLastUpdatedObj, utcLastReportedObj, xostate);

            if (xoReportObj == nullptr) {
                ALOGE("Failed to create IzatXoStateDebugReport object");
            } else {
                jboolean jb = env->CallBooleanMethod(xoStateReportListObj, method_addToList,
                                                     xoReportObj);
                if (jb == JNI_FALSE) {
                    ALOGE("Failed to add IzatXoStateDebugReport object");
                }
            }
        }

        for (auto p = make_pair(0, allReports.mRfAndParams.begin());
             (p.first < maxReports) && (p.second != allReports.mRfAndParams.end());
             p.first++, ++p.second) {
             jlong utcSecondsPartLastUpdated = (*p.second).mUtcTime.tv_sec;
             jlong utcNanoSecPartLastUpdated = (*p.second).mUtcTime.tv_nsec;

            jlong utcSecondsPartLastReported = (*p.second).mUtcReported.tv_sec;
            jlong utcNanoSecPartLastReported = (*p.second).mUtcReported.tv_nsec;

            jint pgaGain = (*p.second).mPgaGain;
            jlong gpsBpAmplI = (*p.second).mGpsBpAmpI;
            jlong gpsBpAmplQ = (*p.second).mGpsBpAmpQ;
            jlong adcI = (*p.second).mAdcI;
            jlong adcQ = (*p.second).mAdcQ;
            jlong jammerMetricGps =(*p.second).mJammerGps;
            jlong jammerMetricGlonass = (*p.second).mJammerGlo;
            jlong jammerMetricBds = (*p.second).mJammerBds;
            jlong jammerMetricGal = (*p.second).mJammerGal;

            jobject utcLastUpdatedObj = createUtcTimeObject(env,
                utcSecondsPartLastUpdated, utcNanoSecPartLastUpdated);

            jobject utcLastReportedObj = createUtcTimeObject(env,
                utcSecondsPartLastReported, utcNanoSecPartLastReported);

            jobject rfReportObj = env->NewObject(IzatRfStateReportClass, method_rfStateReportCtor,
                                                 utcLastUpdatedObj, utcLastReportedObj, pgaGain, gpsBpAmplI,
                                                 gpsBpAmplQ, adcI, adcQ, jammerMetricGps, jammerMetricGlonass,
                                                 jammerMetricBds, jammerMetricGal);
            if (rfReportObj == nullptr) {
                ALOGE("Failed to create IzatRfStateDebugReport object");
            } else {
                jboolean jb = env->CallBooleanMethod(rfStateReportListObj, method_addToList, rfReportObj);
                if (jb  == 0) {
                    ALOGE("Failed to add IzatRfStateDebugReport object");
                }
            }
        }

        for (auto p = make_pair(0, allReports.mErrRecovery.begin());
             (p.first < maxReports) && (p.second != allReports.mErrRecovery.end());
             p.first++, ++p.second) {
            jlong utcSecondsPartLastUpdated = (*p.second).mUtcTime.tv_sec;
            jlong utcNanoSecPartLastUpdated = (*p.second).mUtcTime.tv_nsec;

            jlong utcSecondsPartLastReported = (*p.second).mUtcReported.tv_sec;
            jlong utcNanoSecPartLastReported = (*p.second).mUtcReported.tv_nsec;

            jobject utcLastUpdatedObj = createUtcTimeObject(env,
                utcSecondsPartLastUpdated, utcNanoSecPartLastUpdated);

            jobject utcLastReportedObj = createUtcTimeObject(env,
                utcSecondsPartLastReported, utcNanoSecPartLastReported);

            jobject errorRecoveryReportObj = env->NewObject(IzatErrorRecoveryClass, method_errorRecoveryCtor,
                                                  utcLastUpdatedObj, utcLastReportedObj);
            if (errorRecoveryReportObj == nullptr) {
                ALOGE("Failed to create IzatErrorRecoveryReport object");
            } else {
                jboolean jb = env->CallBooleanMethod(errorRecoveriesListObj, method_addToList, errorRecoveryReportObj);
                if (jb  == 0) {
                    ALOGE("Failed to add IzatErrorRecoveryReport object");
                }
            }
        }

        for (auto p = make_pair(0, allReports.mPdr.begin());
             (p.first < maxReports) && (p.second != allReports.mPdr.end());
             p.first++, ++p.second) {
            jlong utcSecondsPartLastUpdated = (*p.second).mUtcTime.tv_sec;
            jlong utcNanoSecPartLastUpdated = (*p.second).mUtcTime.tv_nsec;

            jlong utcSecondsPartLastReported = (*p.second).mUtcReported.tv_sec;
            jlong utcNanoSecPartLastReported = (*p.second).mUtcReported.tv_nsec;

            jint pdrMask = (*p.second).mFixInfoMask;

            jobject utcLastUpdatedObj = createUtcTimeObject(env,
                utcSecondsPartLastUpdated, utcNanoSecPartLastUpdated);

            jobject utcLastReportedObj = createUtcTimeObject(env,
                utcSecondsPartLastReported, utcNanoSecPartLastReported);

            jobject pdrReportObj = env->NewObject(IzatPDRReportClass, method_pdrReportCtor,
                                                  utcLastUpdatedObj, utcLastReportedObj, pdrMask);
            if (pdrReportObj == nullptr) {
                ALOGE("Failed to create IzatPDRDebugReport object");
            } else {
                jboolean jb = env->CallBooleanMethod(pdrReportListObj, method_addToList, pdrReportObj);
                if (jb  == 0) {
                    ALOGE("Failed to add IzatPDRDebugReport object");
                }
            }
        }

        for (auto p = make_pair(0, allReports.mSvHealth.begin());
             (p.first < maxReports) && (p.second != allReports.mSvHealth.end());
             p.first++, ++p.second) {
            jlong utcSecondsPartLastUpdated = (*p.second).mUtcTime.tv_sec;
            jlong utcNanoSecPartLastUpdated = (*p.second).mUtcTime.tv_nsec;

            jlong utcSecondsPartLastReported = (*p.second).mUtcReported.tv_sec;
            jlong utcNanoSecPartLastReported = (*p.second).mUtcReported.tv_nsec;

            jint gpsGoodMask = (*p.second).mGpsGoodMask;
            jint glonassGoodMask = (*p.second).mGloGoodMask;
            jlong bdsGoodMask = (*p.second).mBdsGoodMask;
            jlong galGoodMask = (*p.second).mGalGoodMask;
            jbyte qzssGoodMask = (*p.second).mQzssGoodMask;

            jint gpsBadMask = (*p.second).mGpsBadMask;
            jint glonassBadMask = (*p.second).mGloBadMask;
            jlong bdsBadMask = (*p.second).mBdsBadMask;
            jlong galBadMask = (*p.second).mGalBadMask;
            jbyte qzssBadMask = (*p.second).mQzssBadMask;

            jint gpsUnknownMask = (*p.second).mGpsUnknownMask;
            jint glonassUnknownMask = (*p.second).mGloUnknownMask;
            jlong bdsUnknownMask = (*p.second).mBdsUnknownMask;
            jlong galUnknownMask = (*p.second).mGalUnknownMask;
            jbyte qzssUnknownMask = (*p.second).mQzssUnknownMask;

            jobject utcLastUpdatedObj = createUtcTimeObject(env,
                utcSecondsPartLastUpdated, utcNanoSecPartLastUpdated);

            jobject utcLastReportedObj = createUtcTimeObject(env,
                utcSecondsPartLastReported, utcNanoSecPartLastReported);

            jobject svHealthReportObj = env->NewObject(IzatSVHealthReportClass, method_svhealthReportCtor,
                                                       utcLastUpdatedObj, utcLastReportedObj, gpsGoodMask, gpsBadMask,
                                                       gpsUnknownMask, glonassGoodMask, glonassBadMask,
                                                       glonassUnknownMask, bdsGoodMask, bdsBadMask, bdsUnknownMask,
                                                       galGoodMask, galBadMask, galUnknownMask, qzssGoodMask,
                                                       qzssBadMask, qzssUnknownMask);
            if (svHealthReportObj == nullptr) {
                ALOGE("Failed to create IzatSVHealthDebugReport object");
            } else {
                jboolean jb = env->CallBooleanMethod(svHealthReportListObj, method_addToList, svHealthReportObj);
                if (jb  == 0) {
                    ALOGE("Failed to add IzatSVHealthDebugReport object");
                }
            }
        }

        for (auto p = make_pair(0, allReports.mXtra.begin());
             (p.first < maxReports) && (p.second != allReports.mXtra.end());
             p.first++, ++p.second) {
            jlong utcSecondsPartLastUpdated = (*p.second).mUtcTime.tv_sec;
            jlong utcNanoSecPartLastUpdated = (*p.second).mUtcTime.tv_nsec;

            jlong utcSecondsPartLastReported = (*p.second).mUtcReported.tv_sec;
            jlong utcNanoSecPartLastReported = (*p.second).mUtcReported.tv_nsec;

            jbyte validityBit =(*p.second).mXtraValidMask;
            jint gpsXtraAge = (*p.second).mGpsXtraAge;
            jint gloXtraAge= (*p.second).mGloXtraAge;
            jint bdsXtraAge = (*p.second).mBdsXtraAge;
            jint galXtraAge = (*p.second).mGalXtraAge;
            jint qzssXtraAge = (*p.second).mQzssXtraAge;
            jint gpsXtraValidity = (*p.second).mGpsXtraValid;
            jint gloXtraValidity = (*p.second).mGloXtraValid;
            jlong bdsXtraValidity = (*p.second).mBdsXtraValid;
            jlong galXtraValidity =(*p.second).mGalXtraValid;
            jbyte qzssXtraValidity = (*p.second).mQzssXtraValid;

            jobject utcLastUpdatedObj = createUtcTimeObject(env,
                utcSecondsPartLastUpdated, utcNanoSecPartLastUpdated);

            jobject utcLastReportedObj = createUtcTimeObject(env,
                utcSecondsPartLastReported, utcNanoSecPartLastReported);

            jobject xtraReportObj = env->NewObject(IzatXTRAReportClass, method_xtraReportCtor,
                                                   utcLastUpdatedObj, utcLastReportedObj,
                                                   validityBit, gpsXtraValidity, gpsXtraAge,
                                                   gloXtraValidity, gloXtraAge, bdsXtraValidity, bdsXtraAge,
                                                   galXtraValidity, galXtraAge, qzssXtraValidity, qzssXtraAge);
            if (xtraReportObj == nullptr) {
                ALOGE("Failed to create IzatXTRADebugReport object");
            } else {
                jboolean jb = env->CallBooleanMethod(xtraReportListObj, method_addToList, xtraReportObj);
                if (jb  == 0) {
                    ALOGE("Failed to add IzatXTRADebugReport object");
                }
            }
        }
    }
}


static JNINativeMethod sMethods[] = {
    /* name, signature, funcPtr */
    {"native_debugreport_class_init",
     "()V",
     reinterpret_cast<void*>(DebugReportService_class_init)},
    {"native_debugreport_init",
     "()V",
     reinterpret_cast<void*>(DebugReportService_init)},
    {"native_debugreport_deinit",
     "()V",
     reinterpret_cast<void*>(DebugReportService_deinit)},
    {"native_debugreport_getReport",
     "(ILjava/util/List;Ljava/util/List;Ljava/util/List;Ljava/util/List;Ljava/util/List;Ljava/util/List;Ljava/util/List;Ljava/util/List;Ljava/util/List;Ljava/util/List;Ljava/util/List;)V",
     reinterpret_cast<void*>(DebugReportService_getReport)}
};

int register_DebugReportService(JNIEnv* env)
{
    ALOGD("register_DebugReportService");
    return jniRegisterNativeMethods(env,
                                    "com/qualcomm/location/izat/debugreport/DebugReportService",
                                    sMethods,
                                    NELEM(sMethods));
}

