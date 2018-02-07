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
	set(GST_INTF intf_mpeg2ts_gst intf_mjpeg_gst intf_h264_gst)
endif()

if (AVB_FEATURE_INTF_ALSA2)
	set (ALSA_INTF alsa_intf)
endif()

if (PCI_SUPPORT_INCLUDED)
	set (PCI_PKG_LIBRARIES pci)
endif()

add_executable ( openavb_host openavb_host.c )
target_link_libraries( openavb_host
	map_ctrl
	map_mjpeg
	map_mpeg2ts
	map_null
	map_pipe
	map_aaf_audio 
	map_uncmp_audio 
	map_h264
	map_clk_ref 
	intf_ctrl
	intf_echo
	intf_logger
	intf_null
	intf_tonegen
	intf_viewer
	intf_alsa
	intf_mpeg2ts_file
	intf_mjpeg_file
	intf_h264_file
	intf_wav_file
        intf_clk_ref
	${GST_INTF}
	avbTl
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
target_link_libraries( openavb_harness 
	map_ctrl
	map_mjpeg
	map_mpeg2ts
	map_null
	map_pipe
	map_aaf_audio 
	map_uncmp_audio 
	map_h264
	map_clk_ref 
	intf_ctrl
	intf_echo
	intf_logger
	intf_null
	intf_tonegen
	intf_viewer
	intf_alsa
	intf_mpeg2ts_file
	intf_mjpeg_file
	intf_h264_file
	intf_wav_file
        intf_clk_ref
	${GST_INTF}
	avbTl
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

if (AVB_FEATURE_GSTREAMER)
include_directories( ${GLIB_PKG_INCLUDE_DIRS} ${GST_PKG_INCLUDE_DIRS} )
target_link_libraries( openavb_host ${GLIB_PKG_LIBRARIES} ${GST_PKG_LIBRARIES} ${PLATFORM_LINK_LIBRARIES} )
target_link_libraries( openavb_harness ${GLIB_PKG_LIBRARIES} ${GST_PKG_LIBRARIES} ${PLATFORM_LINK_LIBRARIES} )
endif ()
