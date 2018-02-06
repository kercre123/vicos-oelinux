/*
 * Copyright (c) 2016-2017, Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#ifndef _CONFIG_BASE_H_
#define _CONFIG_BASE_H_

#include <string>
#include <map>

#include <VAM/vaapi.h>

class ConfigBase
{
public:
    ConfigBase();
    virtual int readConfig(std::string configFileName, int debugLevel=0);
    virtual void print()=0;

    static std::string EventTypeToStr(vaapi_event_type eventType);
    virtual ~ConfigBase() {};

protected:
    virtual int _processItems(std::ifstream *pConfigFile, std::string tagStr) { return 0; }
    virtual int _validateConfig() { return 0; }

    std::string trimWhiteSpace(const std::string& str,
            const std::string& whitespace = " \t");

    inline bool isBlank(const std::string &str)
    { return str.size()==0; }

    inline bool isComment(const std::string &str)
    { return str.size() > 0 && str[0] == '#'; }

private:
    bool _isInitialized;
};

#endif // #define _CONFIG_BASE_H_
