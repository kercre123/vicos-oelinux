/*
 * Copyright (c) 2016-2017, Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#ifndef _VASIM_H_
#define _VASIM_H_

#include <string>
#include <vector>
#include <fstream>
#include "VASimConfig.h"
#include "VASimFRConfig.h"
#include "VAMJSONRuleConfig.h"
#include "VAMExtRuleConfig.h"
#include "VASimDecoder.h"
#include "VASimRenderer.h"

struct VASimParams
{
    bool isStop;
    VASimConfig config;
    VASimFRConfig FRConfig;
    VAMJSONRuleConfig JSONRuleConfig;
    VAMExtRuleConfig extRuleConfig;
    std::vector<VideoState*> videoStateList;
    std::ofstream metadataFileWriter;
    std::ofstream eventFileWriter;
    bool isNewEventFile;
    bool isNewMetadataFile;
    bool isRawVideo;

    VASimParams();
    ~VASimParams();
    int init(int argc, char **argv);
};

#endif // #define _VASIM_H_
