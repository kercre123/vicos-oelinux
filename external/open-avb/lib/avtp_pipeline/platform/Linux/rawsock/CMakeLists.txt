SET (LOCAL_SRC_FILES
	${AVB_OSAL_DIR}/rawsock/openavb_rawsock.c
	${AVB_OSAL_DIR}/rawsock/simple_rawsock.c
	${AVB_OSAL_DIR}/rawsock/ring_rawsock.c
)

if ( AVB_FEATURE_PCAP )
  find_package ( PCAP REQUIRED )
  include_directories ( ${PCAP_INCLUDE_DIR} )
  SET (LOCAL_SRC_FILES ${LOCAL_SRC_FILES} ${AVB_OSAL_DIR}/rawsock/pcap_rawsock.c)
endif ()

SET (SRC_FILES ${SRC_FILES} ${LOCAL_SRC_FILES} PARENT_SCOPE)