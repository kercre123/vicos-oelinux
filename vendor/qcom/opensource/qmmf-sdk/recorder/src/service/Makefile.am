source_files = qmmf_recorder_service.cc
source_files += qmmf_recorder_impl.cc
source_files += qmmf_recorder_ion.cc
source_files += qmmf_remote_cb.cc
source_files += qmmf_camera_source.cc
source_files += qmmf_camera_context.cc
if ENABLE_360
source_files += qmmf_multicamera_manager.cc
endif
source_files += qmmf_encoder_core.cc
source_files += qmmf_audio_source.cc
source_files += qmmf_audio_raw_track_source.cc
source_files += qmmf_audio_encoded_track_source.cc
source_files += qmmf_audio_encoder_core.cc
if ENABLE_360
source_files += qmmf_jpeg_encoder.cc
source_files += qmmf_camera_jpeg.cc
source_files += qmmf_exif_generator.cc
endif

source_files += qmmf_camera_reprocess_impl.cc
source_files += post-process/factory/qmmf_postproc_factory.cc
source_files += post-process/node/qmmf_postproc_node.cc
source_files += post-process/memory/qmmf_postproc_memory_pool.cc
source_files += post-process/pipe/qmmf_postproc_pipe.cc
source_files += post-process/common/qmmf_postproc_thread.cc
source_files += qmmf_camera_rescaler.cc

# Build qmmf recorder service library
# libqmmf_recorder_service.so
lib_LTLIBRARIES = libqmmf_recorder_service.la

libqmmf_recorder_service_la_SOURCES = $(source_files)

libqmmf_recorder_service_la_CPPFLAGS = -I$(top_srcdir)/include
libqmmf_recorder_service_la_CPPFLAGS += -I$(top_srcdir)/
libqmmf_recorder_service_la_CPPFLAGS += -I$(LIBGRALLOC)
libqmmf_recorder_service_la_CPPFLAGS += -I$(MMCORE_CFLAGS)
libqmmf_recorder_service_la_CPPFLAGS += -I$(CAMERAHAL_CFLAGS)
libqmmf_recorder_service_la_CPPFLAGS += -I$(CAMERASTACKCOMMON_CFLAGS)
libqmmf_recorder_service_la_CPPFLAGS += -I$(CAMERAINTERFACE_CFLAGS)
libqmmf_recorder_service_la_CPPFLAGS += -I$(EXIF_CFLAGS)
libqmmf_recorder_service_la_CPPFLAGS += -I$(OMXCORE_CFLAGS)
libqmmf_recorder_service_la_CPPFLAGS += -I$(OPENMAX_CFLAGS)
if ENABLE_360
libqmmf_recorder_service_la_CPPFLAGS += -DENABLE_360
endif

libqmmf_recorder_service_la_LDFLAGS  = -llog -lbinder -lutils -lcutils
libqmmf_recorder_service_la_LDFLAGS  += -lcamera_client -lcamera_metadata
libqmmf_recorder_service_la_LDFLAGS  += -lhardware -lfastcvopt -lC2D2

libqmmf_recorder_service_la_LIBADD   = ../client/libqmmf_recorder_client.la
libqmmf_recorder_service_la_LIBADD   += ../../../common/cameraadaptor/libcamera_adaptor.la
libqmmf_recorder_service_la_LIBADD   += ../../../common/codecadaptor/libcodec_adaptor.la
libqmmf_recorder_service_la_LIBADD   += ../../../common/audio/src/client/libqmmf_audio_client.la
libqmmf_recorder_service_la_LIBADD   += ../../../common/overlay/liboverlay.la
libqmmf_recorder_service_la_LIBADD   += ../../../display/src/client/libqmmf_display_client.la
libqmmf_recorder_service_la_LIBADD   += ../service/post-process/modules/camera-hal-jpeg/libqmmf_camera_hal_jpeg.la
libqmmf_recorder_service_la_LIBADD   += ../service/post-process/modules/camera-hal-reproc/libqmmf_camera_hal_reproc.la
libqmmf_recorder_service_la_LIBADD   += ../service/post-process/modules/jpeg-encoder/libqmmf_jpeg.la
libqmmf_recorder_service_la_LIBADD   += ../service/post-process/modules/test/libqmmf_postproc_test.la
libqmmf_recorder_service_la_LIBADD   += ../service/post-process/modules/algo/libqmmf_postproc_algo.la
