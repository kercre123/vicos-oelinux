include_directories(
		${AVB_OSAL_DIR}/tl
		${AVB_OSAL_DIR}/endpoint
		${AVB_SRC_DIR}/util
		${AVB_SRC_DIR}/tl
		${AVB_SRC_DIR}/srp
		)

MESSAGE ("PCI_SUPPORT_INCLUDED in avb_host     : ${PCI_SUPPORT_INCLUDED}")
# Rules to build the AVB host
if (AVB_FEATURE_GSTREAMER)
	set(GST_INTF openavb_intf_mpeg2ts_gst openavb_intf_mjpeg_gst openavb_intf_h264_gst)
	include_directories( ${GLIB_PKG_INCLUDE_DIRS} ${GST_PKG_INCLUDE_DIRS} )
endif()

if (AVB_FEATURE_INTF_ALSA2)
	set (ALSA_INTF openavb_intf_alsa)
endif()

if (PCI_SUPPORT_INCLUDED)
	set (PCI_PKG_LIBRARIES pci)
endif()

add_executable ( openavb_host openavb_host.c )
link_directories( ${SYSROOT}/usr/lib ${SYSROOT}/usr/lib64 )
target_link_libraries( openavb_host
	openavb
	openavb_map_ctrl
	openavb_map_mjpeg
	openavb_map_mpeg2ts
	openavb_map_null
	openavb_map_pipe
	openavb_map_aaf_audio
	openavb_map_uncmp_audio
	openavb_map_h264
	openavb_map_clk_ref
	openavb_intf_ctrl
	openavb_intf_echo
	openavb_intf_logger
	openavb_intf_null
	openavb_intf_tonegen
	openavb_intf_viewer
	openavb_intf_alsa
	openavb_intf_mpeg2ts_file
	openavb_intf_mjpeg_file
	openavb_intf_h264_file
	openavb_intf_wav_file
	openavb_intf_clk_ref
	${GST_INTF}
	${PLATFORM_LINK_LIBRARIES}
	${ALSA_LIBRARIES}
	${ALSA_INTF}
	${GSTRTP_PKG_LIBRARIES}
	${GLIB_PKG_LIBRARIES}
	${GST_PKG_LIBRARIES}
	pthread
	rt
	dl
	${PCI_PKG_LIBRARIES})

# Rules to build the AVB harness
add_executable ( openavb_harness openavb_harness.c )
link_directories( ${SYSROOT}/usr/lib ${SYSROOT}/usr/lib64 )
target_link_libraries( openavb_harness
	openavb
	openavb_map_ctrl
	openavb_map_mjpeg
	openavb_map_mpeg2ts
	openavb_map_null
	openavb_map_pipe
	openavb_map_aaf_audio
	openavb_map_uncmp_audio
	openavb_map_h264
	openavb_map_clk_ref
	openavb_intf_ctrl
	openavb_intf_echo
	openavb_intf_logger
	openavb_intf_null
	openavb_intf_tonegen
	openavb_intf_viewer
	openavb_intf_alsa
	openavb_intf_mpeg2ts_file
	openavb_intf_mjpeg_file
	openavb_intf_h264_file
	openavb_intf_wav_file
	openavb_intf_clk_ref
	${GST_INTF}
	${PLATFORM_LINK_LIBRARIES}
	${ALSA_LIBRARIES}
	${ALSA_INTF}
	${GSTRTP_PKG_LIBRARIES}
	${GLIB_PKG_LIBRARIES}
	${GST_PKG_LIBRARIES}
	pthread
	rt
	dl
	${PCI_PKG_LIBRARIES})

# Install rules
install ( TARGETS openavb_host RUNTIME DESTINATION ${AVB_INSTALL_BIN_DIR} )
install ( TARGETS openavb_harness RUNTIME DESTINATION ${AVB_INSTALL_BIN_DIR} )
