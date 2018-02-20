# and another kernel sources
#set ( LINUX_KERNEL_DIR "/usr/src/kernel" )

# build configuration
set ( OPENAVB_HAL      "arm_ntn" )
set ( OPENAVB_OSAL     "Linux" )
set ( OPENAVB_TCAL     "GNU" )
set ( OPENAVB_PLATFORM "${OPENAVB_HAL}-${OPENAVB_OSAL}" )

# Platform Additions
set ( PLATFORM_INCLUDE_DIRECTORIES
	${CMAKE_SOURCE_DIR}/platform/generic
	${CMAKE_SOURCE_DIR}/../neutrino
	${CMAKE_SOURCE_DIR}/openavb_common
	${CMAKE_SOURCE_DIR}/../../daemons/common
	${CMAKE_SOURCE_DIR}/../../daemons/mrpd
)

set ( PLATFORM_LINK_DIRECTORIES
	${CMAKE_SOURCE_DIR}/../neutrino
)

set ( PLATFORM_LINK_LIBRARIES
	neutrino
)

set ( AVB_FEATURE_PCAP 1 )
