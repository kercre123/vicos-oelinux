#!/bin/sh
#==============================================================================
# FILE: ipa_fws.sh
#
# DESCRIPTION:
# Indicate to IPA driver that FWs are available from user space for fetching
# and loading
#
# Copyright (c) 2017 Qualcomm Technologies, Inc.
# All Rights Reserved.
# Confidential and Proprietary - Qualcomm Technologies, Inc.
#==============================================================================
# A write to /dev/ipa will indicate that user space is available and the
# FWs can be fetched.
echo 1 > /dev/ipa
