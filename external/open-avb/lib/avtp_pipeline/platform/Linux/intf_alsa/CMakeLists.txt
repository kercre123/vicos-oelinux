SET (SRC_FILES ${SRC_FILES}
	${AVB_OSAL_DIR}/intf_alsa/openavb_intf_alsa.c
	PARENT_SCOPE
)

if (AVB_FEATURE_INTF_ALSA2)
	SET (SRC_FILES ${SRC_FILES}
		${AVB_OSAL_DIR}/intf_alsa/openavb_intf_alsa.c
		${AVB_OSAL_DIR}/intf_alsa/openavb_intf_alsa2.c
		PARENT_SCOPE
	)
endif()

# Need include and link directories for ALSA
SET (INTF_INCLUDE_DIR ${INTF_INCLUDE_DIR} ${ALSA_INCLUDE_DIRS} PARENT_SCOPE)
SET (INTF_LIBRARY_DIR ${INTF_LIBRARY_DIR} ${ALSA_LIBRARY_DIRS} PARENT_SCOPE)
SET (INTF_LIBRARY ${ALSA_LIBRARIES} pthread rt PARENT_SCOPE)

