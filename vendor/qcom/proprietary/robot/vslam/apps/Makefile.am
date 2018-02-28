ACLOCAL_AMFLAGS = -I m4

bin_PROGRAMS = mv_vwslam

# mv_vwslam
# =========================================================
mv_vwslam_SOURCES := \
     src/main.cpp \
     src/mainThread.cpp \
     src/Camera_VSLAM.cpp \
     src/Motion.cpp \
     src/MoveAlongPathPattern.cpp \
     src/ReadImages.cpp \
     src/ReadConfigurationFile.cpp \
     src/ReadConfigurationFile_Internal.cpp \
     src/ReadGrayBMP.cpp \
     src/ScaleEstimation.cpp \
     src/VirtualWheel.cpp \
     src/Visualization.cpp \
     src/Visualization_NonRos.cpp \
     src/UndistortionEngine.cpp \
     src/VSLAM.cpp \
     src/WEF.cpp  \
     src/VSLAMScheduler.cpp \
     src/MapFocuser.cpp

mv_vwslam_CPPFLAGS = -O2
mv_vwslam_CPPFLAGS += -g
mv_vwslam_CPPFLAGS += -Wno-all
mv_vwslam_CPPFLAGS += -Wno-error
mv_vwslam_CPPFLAGS += -Wno-unused-parameter
mv_vwslam_CPPFLAGS += -I .
mv_vwslam_CPPFLAGS += -Iinc
mv_vwslam_CPPFLAGS += -I../prebuilt/inc
mv_vwslam_CPPFLAGS += -std=c++11
mv_vwslam_CPPFLAGS += -DARM_BASED

mv_vwslam_LDADD = $(GLIB_LIBS)
mv_vwslam_LDADD += ../prebuilt/libs/libmv0.so
mv_vwslam_LDADD += ../prebuilt/libs/libmv1.so
mv_vwslam_LDADD += ../prebuilt/libs/libmv3.so
mv_vwslam_LDADD += -ldl -lcutils -lcamera

