/*
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#define LOG_TAG "WiFiDBReceiver_jni"
#define LOG_NDEBUG 0
#define MAX_NUMBER_APS 500

#include "JNIHelp.h"
#include "jni.h"
#include "android_runtime/AndroidRuntime.h"
#include "android_runtime/Log.h"
#include "izat_wifi_db_receiver.h"
#include "utils_jni.h"

using namespace android;

static void* sWiFiDBReceiverHandle = NULL;
static JavaVM* sWiFiDBRecCbJavaVm = NULL;
static jobject sCallbacksObj = NULL;

static jmethodID method_onAPListAvailable = NULL;
static jmethodID method_onStatusUpdate = NULL;
static jmethodID method_onServiceRequest = NULL;

static jclass class_APInfo = NULL;
static jclass class_APLocationData = NULL;
static jclass class_APSpecialInfo = NULL;

static uint64_t convertMacStrto48R(const char *mac) {
    if (0 != mac) {

        int hi24, lo24;
        sscanf(mac, "%06X%06X", &lo24, &hi24);
        uint64_t temp = (unsigned)hi24;
        temp <<= 24;
        temp |= (uint64_t)lo24;
        return temp;
    }
    return 0;
}

static void convertMac48RtoStr(char mac_str[], int str_size, uint64_t mac) {
    snprintf(mac_str, str_size, "%06X%06X",
             (unsigned int)(mac & 0xFFFFFF), (unsigned int)(mac >> 24));
}

static int TranslateFromObject(const jobject locationObject, APLocationData_s *&location_data) {
    int result = 0;
    JNIEnv* env = NULL;
    if (sWiFiDBRecCbJavaVm) {
        sWiFiDBRecCbJavaVm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6);
    }

    if (NULL == env) {
        result = 400;
    } else {

        static jfieldID jFieldAPLocationData_mac =
            env->GetFieldID(class_APLocationData, "mMacAddress",
                            "Ljava/lang/String;");
        static jfieldID jFieldAPLocationData_lat =
            env->GetFieldID(class_APLocationData, "mLatitude",
                            "F");
        static jfieldID jFieldAPLocationData_lon =
            env->GetFieldID(class_APLocationData, "mLongitude",
                            "F");
        static jfieldID jFieldAPLocationData_mar =
            env->GetFieldID(class_APLocationData, "mMaxAntenaRange",
                            "F");
        static jfieldID jFieldAPLocationData_he =
            env->GetFieldID(class_APLocationData, "mHorizontalError",
                            "F");
        static jfieldID jFieldAPLocationData_reliability =
            env->GetFieldID(class_APLocationData, "mReliability",
                            "I");
        static jfieldID jFieldAPLocationData_valid =
            env->GetFieldID(class_APLocationData, "mValidBits",
                            "I");

        jstring strMac = (jstring)env->GetObjectField(locationObject,
                                             jFieldAPLocationData_mac);
        if (NULL == strMac) {
            result = 401;
        } else {

            jfloat lat  = env->GetFloatField(locationObject,
                                             jFieldAPLocationData_lat);
            jfloat lon  = env->GetFloatField(locationObject,
                                             jFieldAPLocationData_lon);
            jfloat mar  = env->GetFloatField(locationObject,
                                             jFieldAPLocationData_mar);
            jfloat hepe = env->GetFloatField(locationObject,
                                             jFieldAPLocationData_he);
            jint rel    = env->GetIntField(locationObject,
                                           jFieldAPLocationData_reliability);
            jint valid  = env->GetIntField(locationObject,
                                           jFieldAPLocationData_valid);

            location_data = new APLocationData_s;
            if (NULL == location_data) {
                result = 402;
            } else {

                const char *mac = env->GetStringUTFChars(strMac, NULL);
                if (NULL == mac) {
                    result = 403;
                } else {
                    location_data->mac_R48b = convertMacStrto48R(mac);
                    env->ReleaseStringUTFChars(strMac, NULL);

                    location_data->latitude = lat;
                    location_data->longitude = lon;
                    location_data->max_antena_range = mar;
                    location_data->horizontal_error = hepe;
                    location_data->reliability = rel;
                    location_data->valid_bits = valid;
                }
            }
        }
    }

    return result;
}

static int TranslateFromObject(const jobject splecialInfoObject, APSpecialInfo_s *&special_info) {
    int result = 0;
    JNIEnv* env = NULL;
    if (sWiFiDBRecCbJavaVm) {
        sWiFiDBRecCbJavaVm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6);
    }

    if (NULL == env) {
        result = 700;
    } else {

        static jfieldID jFieldAPSpecialInfo_mac =
            env->GetFieldID(class_APSpecialInfo, "mMacAddress",
                            "Ljava/lang/String;");
        static jfieldID jFieldAPSpecialInfo_info =
            env->GetFieldID(class_APSpecialInfo, "mInfo", "I");
        jstring strMac = (jstring)env->GetObjectField(splecialInfoObject,
                                             jFieldAPSpecialInfo_mac);
        if (NULL == strMac) {
            result = 701;
        } else {

            jint info  = env->GetIntField(splecialInfoObject,
                                          jFieldAPSpecialInfo_info);

            special_info = new APSpecialInfo_s;
            if (NULL == special_info) {
                result = 702;
            } else {

                const char *mac = env->GetStringUTFChars(strMac, NULL);
                if (NULL == mac) {
                    result = 703;
                } else {
                    special_info->mac_R48b = convertMacStrto48R(mac);
                    env->ReleaseStringUTFChars(strMac, NULL);

                    special_info->info = info;
                }
            }
        }
    }
    return result;
}

static void TranslateToObject(const APInfo_s *ap, jobject& apInfoObject) {
    JNIEnv* env = NULL;
    if (sWiFiDBRecCbJavaVm) {
        sWiFiDBRecCbJavaVm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6);
    }

    if (env) {

        if (class_APInfo) {
            jmethodID jConstructorAPInfo = env->GetMethodID(class_APInfo,
                                                            "<init>",
                                                            "()V");
            static jfieldID jFieldMac = env->GetFieldID(class_APInfo,
                                                        "mMacAddress",
                                                        "Ljava/lang/String;");
            static jfieldID jFieldCellType = env->GetFieldID(class_APInfo,
                                                             "mCellType", "I");
            static jfieldID jFieldCell1 = env->GetFieldID(class_APInfo,
                                                          "mCellRegionID1", "I");
            static jfieldID jFieldCell2 = env->GetFieldID(class_APInfo,
                                                          "mCellRegionID2", "I");;
            static jfieldID jFieldCell3 = env->GetFieldID(class_APInfo,
                                                          "mCellRegionID3", "I");
            static jfieldID jFieldCell4 = env->GetFieldID(class_APInfo,
                                                          "mCellRegionID4", "I");
            static jfieldID jFieldSSID = env->GetFieldID(class_APInfo,
                                                         "mSSID", "[B");
            apInfoObject = env->NewObject(class_APInfo, jConstructorAPInfo, NULL);
            if (apInfoObject) {
                char macStr[13];
                convertMac48RtoStr(macStr, 13, ap->mac_R48b);
                jstring jStrMac = env->NewStringUTF(macStr);
                if (jStrMac) {
                    env->SetObjectField(apInfoObject, jFieldMac, jStrMac);
                    env->SetIntField(apInfoObject, jFieldCellType, ap->cell_type);
                    env->SetIntField(apInfoObject, jFieldCell1, ap->cell_id1);
                    env->SetIntField(apInfoObject, jFieldCell2, ap->cell_id2);
                    env->SetIntField(apInfoObject, jFieldCell3, ap->cell_id3);
                    env->SetIntField(apInfoObject, jFieldCell4, ap->cell_id4);
                    jbyteArray jssid = env->NewByteArray(ap->ssid_valid_byte_count);
                    jbyte* jbssid = new jbyte[ap->ssid_valid_byte_count];
                    for (int ii = 0; ii < ap->ssid_valid_byte_count; ii++) {
                        jbssid[ii] = (jbyte)ap->ssid[ii];
                    }
                    if (jssid && jbssid) {
                        env->SetByteArrayRegion(jssid, 0, ap->ssid_valid_byte_count, jbssid);
                        delete jbssid;
                        env->SetObjectField(apInfoObject, jFieldSSID, jssid);
                        // Managing overwhelming number of requests
                        env->DeleteLocalRef(jssid);
                    }
                }
                // Managing overwhelming number of requests
                env->DeleteLocalRef(jStrMac);
            }
        }
    }
}

static void wifi_receiver_ap_list_update_callback(APInfo_s *ap_list[],
                                                  size_t ap_list_size,
                                                  void * /*clientData*/) {

    JNIEnv* env = NULL;
    if (sWiFiDBRecCbJavaVm) {
        sWiFiDBRecCbJavaVm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6);
    }

    ALOGD("wifi_receiver_ap_list_update_callback: list size %d\n", ap_list_size);
    if (env && sCallbacksObj && method_onAPListAvailable) {
        if (class_APInfo) {
            jobjectArray jAPList = (jobjectArray)env->NewObjectArray(ap_list_size,
                                                                     class_APInfo,
                                                                     NULL);
            if (jAPList) {
                for (size_t ii = 0; ii < ap_list_size; ii++) {
                    jobject jObjAPInfo = NULL;
                    TranslateToObject(ap_list[ii], jObjAPInfo);
                    if (jObjAPInfo) {
                        env->SetObjectArrayElement(jAPList, ii, jObjAPInfo);
                    }
                    // Managing overwhelming number of requests
                    env->DeleteLocalRef(jObjAPInfo);
                }
                env->CallVoidMethod(sCallbacksObj,
                                    method_onAPListAvailable,
                                    jAPList);
                // Managing overwhelming number of requests
                env->DeleteLocalRef(jAPList);
                checkAndClearExceptionsFromCallback(env, __FUNCTION__);
            }
        }
    }
};

static void wifi_receiver_status_update_callback(bool isSuccess,
                                                 const char* errStr,
                                                 void* /*clientData*/)
{
    JNIEnv* env = NULL;
    if (sWiFiDBRecCbJavaVm) {
        sWiFiDBRecCbJavaVm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6);
    }
    ALOGD("wifi_receiver_status_update_callback: %d\n", isSuccess);

    if (env && sCallbacksObj && method_onStatusUpdate) {
        jstring jErrStr = env->NewStringUTF(errStr);
        env->CallVoidMethod(sCallbacksObj,
                            method_onStatusUpdate,
                            isSuccess,
                            jErrStr);
        checkAndClearExceptionsFromCallback(env, __FUNCTION__);
    }
};

static void wifi_receiver_service_request_callback(void* /*clientData*/)
{
    JNIEnv* env = NULL;
    if (sWiFiDBRecCbJavaVm) {
        sWiFiDBRecCbJavaVm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6);
    }
    ALOGD("wifi_receiver_service_request_callback\n");
    if (env && sCallbacksObj && method_onServiceRequest) {
        env->CallVoidMethod(sCallbacksObj,
                            method_onServiceRequest);
        checkAndClearExceptionsFromCallback(env, __FUNCTION__);
    }
};

static void wifi_receiver_attach_vm_on_callback_env(void* /*clientData*/)
{
    ALOGD("wifi_receiver_attach_vm_on_callback_env\n");
    sWiFiDBRecCbJavaVm = AndroidRuntime::getJavaVM();
    JNIEnv *callbackEnv = NULL;
    JavaVMAttachArgs args = {
                JNI_VERSION_1_6,
                "WFD Service Callback Thread",
                NULL
            };
    jint attachResult = sWiFiDBRecCbJavaVm->AttachCurrentThread(&callbackEnv, &args);
    if (attachResult != 0) {
        ALOGE("Callback thread attachment error: %d", attachResult);
        callbackEnv = NULL;
    }
};

static void WiFiDBReceiver_class_init(JNIEnv* env, jclass clazz)
{
    ALOGD("WiFiDBReceiver_class_init");
    // get references to the Java methods
    method_onAPListAvailable = env->GetMethodID(clazz,
                                                "onAPListAvailable",
                                                "([Lcom/qti/wifidbreceiver/APInfo;)V");
    method_onStatusUpdate = env->GetMethodID(clazz, "onStatusUpdate", "(ZLjava/lang/String;)V");
    method_onServiceRequest = env->GetMethodID(clazz, "onServiceRequest", "()V");


}

static void WiFiDBReceiver_init(JNIEnv *env, jobject obj)
{
    ALOGD("WiFiDBReceiver_init");
    if (sCallbacksObj == NULL) {
        sCallbacksObj = env->NewGlobalRef(obj);
    }
    if (class_APInfo == NULL) {
        jclass class_APInfo_temp = env->FindClass("com/qti/wifidbreceiver/APInfo");
        if (!class_APInfo_temp) {
            ALOGE("Class not found : com/qti/wifidbreceiver/APInfo");
        } else {
            class_APInfo = (jclass)env->NewGlobalRef(class_APInfo_temp);
        }
    }
    if (class_APLocationData == NULL) {
        jclass class_APLocationData_temp = env->FindClass("com/qti/wifidbreceiver/APLocationData");
        if (!class_APLocationData_temp) {
            ALOGE("Class not found : com/qti/wifidbreceiver/APLocationData");
        } else {
            class_APLocationData = (jclass)env->NewGlobalRef(class_APLocationData_temp);
        }
    }
    if (class_APSpecialInfo == NULL) {
        jclass class_APSpecialInfo_temp = env->FindClass("com/qti/wifidbreceiver/APSpecialInfo");
        if (!class_APSpecialInfo_temp) {
            ALOGE("Class not found : com/qti/wifidbreceiver/APSpecialInfo");
        } else {
            class_APSpecialInfo = (jclass)env->NewGlobalRef(class_APSpecialInfo_temp);
        }
    }
    if (sWiFiDBReceiverHandle == NULL) {
        sWiFiDBReceiverHandle = registerWiFiDBUpdater(wifi_receiver_ap_list_update_callback,
                                                      wifi_receiver_status_update_callback,
                                                      wifi_receiver_service_request_callback,
                                                      wifi_receiver_attach_vm_on_callback_env,
                                                      (void*)0xDEADBEEF);
    }
}

static void WiFiDBReceiver_deinit(JNIEnv *env, jobject /*obj*/)
{
    // clear Global References if any
    if (sCallbacksObj) env->DeleteGlobalRef(sCallbacksObj);
    sCallbacksObj = NULL;

    if (class_APInfo) env->DeleteGlobalRef(class_APInfo);
    class_APInfo = NULL;

    if (class_APLocationData) env->DeleteGlobalRef(class_APLocationData);
    class_APLocationData = NULL;

    if (class_APSpecialInfo) env->DeleteGlobalRef(class_APSpecialInfo);
    class_APSpecialInfo = NULL;

    if (sWiFiDBReceiverHandle) unregisterWiFiDBUpdater(sWiFiDBReceiverHandle);
    sWiFiDBReceiverHandle = NULL;
}


static jint WiFiDBReceiver_requestAPList(JNIEnv* /*env*/, jobject /*obj*/,
                                         int expire_in_days) {
    jint result = 0;
    if (sWiFiDBReceiverHandle) {
        sendAPListRequest(expire_in_days, sWiFiDBReceiverHandle);
    }
    else {
        result = 1;
    }
    return result;
}

static jint WiFiDBReceiver_pushWiFiDB(JNIEnv* env, jobject /*obj*/,
                                      jobjectArray j_loc_list,
                                      jobjectArray j_spl_list,
                                      int days_valid) {
    int result = 0;

    APLocationData_s **loc_list = NULL;
    jsize j_loc_list_len = 0;
    APSpecialInfo_s** spl_list = NULL;
    jsize j_spl_list_len = 0;

    if (NULL == sWiFiDBReceiverHandle) {
        result = 1;
    } else {
        ALOGD("DBRecJNI: pushDB\n");

        if (j_loc_list) {
            j_loc_list_len = env->GetArrayLength(j_loc_list);
            if ((int)j_loc_list_len > MAX_NUMBER_APS)
            {
              ALOGD("JNI pushWiFiDB: Max location Data size exceeded %d", (int)j_loc_list_len);
              result = 4;
            }
            else
            {
              ALOGD("JNI pushWiFiDB: Location Data size %d", (int)j_loc_list_len);
              for (size_t ii = 0; ii <(size_t)j_loc_list_len; ii++) {
                if (NULL == loc_list) {
                    loc_list = new APLocationData_s *[j_loc_list_len];
                }
                if (NULL == loc_list) {
                    result = 2;
                    break;
                }
                jobject objAPLocationData = env->GetObjectArrayElement(j_loc_list, ii);
                if (NULL == objAPLocationData) {
                    result = 3;
                    break;
                }

                result = TranslateFromObject(objAPLocationData, loc_list[ii]);
                env->DeleteLocalRef(objAPLocationData);
                if (0 != result) {
                    break;
                }
              }
            }
        } else {
            ALOGD("pushWiFiDB: NO Location Data");
        }

        if (0 == result) {

            if (j_spl_list) {
                j_spl_list_len = env->GetArrayLength(j_spl_list);
                for (size_t ii = 0; ii < (size_t)j_spl_list_len; ii++) {
                    if (NULL == spl_list) {
                        spl_list = new APSpecialInfo_s *[j_spl_list_len];
                    }
                    if (NULL == spl_list) {
                        result = 5;
                        break;
                    }
                    jobject objAPSpecialInfo = env->GetObjectArrayElement(j_spl_list, ii);
                    if (NULL == objAPSpecialInfo) {
                        result = 6;
                        break;
                    }

                    result = TranslateFromObject(objAPSpecialInfo, spl_list[ii]);
                    env->DeleteLocalRef(objAPSpecialInfo);
                    if (0 != result) {
                        break;
                    }
                }
            } else {
                ALOGD("pushWiFiDB: Fail to push location Data");
            }

            if (0 == result) {
                pushWiFiDB(loc_list, j_loc_list_len, spl_list,
                           j_spl_list_len, days_valid,
                           sWiFiDBReceiverHandle);
            }

        }

        // Clean up
        if (NULL != loc_list)
        {
            for (size_t ii = 0; loc_list && ii < (size_t)j_loc_list_len; ii++) {
              if (NULL != loc_list[ii])
              {
                delete loc_list[ii];
              }
            }
            delete[] loc_list;
        }
        if (NULL != spl_list)
        {
            for (size_t ii = 0; spl_list && ii < (size_t)j_spl_list_len; ii++) {
              if (NULL != spl_list[ii])
              {
                delete spl_list[ii];
              }
            }
            delete[] spl_list;
        }

    }
    return result;

}

static JNINativeMethod sMethods[] = {
    /* name, signature, funcPtr */
    { "native_wifi_db_receiver_class_init",
     "()V",
     reinterpret_cast<void*>(WiFiDBReceiver_class_init)},
    {"native_wifi_db_receiver_init",
     "()V",
     reinterpret_cast<void*>(WiFiDBReceiver_init)},
    {"native_wifi_db_receiver_deinit",
     "()V",
     reinterpret_cast<void*>(WiFiDBReceiver_deinit)},
    {"native_wifi_db_receiver_request_ap_list",
     "(I)I",
     reinterpret_cast<void*>(WiFiDBReceiver_requestAPList)},
    {"native_wifi_db_receiver_push_ad",
     "([Lcom/qti/wifidbreceiver/APLocationData;[Lcom/qti/wifidbreceiver/APSpecialInfo;I)I",
     reinterpret_cast<void*>(WiFiDBReceiver_pushWiFiDB)}
};

int register_WiFiDBReceiver(JNIEnv* env)
{
    ALOGD("register_WiFiDBReceiver");

    return jniRegisterNativeMethods(env,
                                    "com/qualcomm/location/izat/wifidbreceiver/WiFiDBReceiver",
                                    sMethods,
                                    NELEM(sMethods));
}
