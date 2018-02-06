/*
 * Copyright (c) 2016-2017, Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#include <iostream>
#include <fstream>
#include <sstream>

#include "VASimFRConfig.h"
#include "VASimError.h"


void VASimFRConfig_enrollmentInfo::print()
{
    printf("\tuuid: %s\n", uuid.c_str());
    printf("\tdisplayName: %s\n", displayName.c_str());
    printf("\timageName: %s\n", imageName.c_str());
}

VASimFRConfig::VASimFRConfig()
{
    //std::cout << "VASimFRConfig:VASimFRConfig()" << std::endl;
}

void VASimFRConfig::print()
{
    std::cout << "===VASimFRConfig=============================begin" << std::endl;
    for(size_t idx=0; idx<enrollmentList.size(); idx++)
    {
        printf("  enrollment %d:\n", (int)idx);
        enrollmentList[idx].print();
    }

    std::cout << "===VASimFRConfig=============================end" << std::endl;
    std::cout << std::endl;
}

int VASimFRConfig::_processItems(std::ifstream *pConfigFile, std::string str)
{
    VASimFRConfig_enrollmentInfo info;
    std::string *s[3] = {
        &info.uuid,
        &info.displayName,
        &info.imageName};
    size_t idx = 0;

    str = trimWhiteSpace(str);
    std::size_t found = str.find(',');
    while(found != std::string::npos && idx < 3)
    {
        std::string field = str.substr(0, found);
        field = trimWhiteSpace(field);
        //printf("%s\n", field.c_str());

        *s[idx++] = field;

        str = str.substr(found+1);
        found=str.find(',');
    }

    if(idx < 3)
    {
        str = trimWhiteSpace(str);
        //printf("%s\n", str.c_str());
        *s[idx] = str;
    }

    enrollmentList.push_back(info);

    return VS_OK;
}

