/*
 * Copyright (c) 2016-2017, Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>

#include "VAMJSONRuleConfig.h"
#include "VASimError.h"

extern "C"
{
#include "configuration_parser_apis.h"
}

static int parseJSONZones( JSONVAZone *pJZones, int zoneCount, vaapi_zone *pZones);
static int parseJSONRule( JSONVAAtomicRule *pJSONAtomicRule, vaapi_rule *pRule, vaapi_zone *pZones, int zoneSize);

VAMJSONRuleConfig::VAMJSONRuleConfig()
{
    memset(&vaapiConfig, 0, sizeof(vaapiConfig));
}

VAMJSONRuleConfig::~VAMJSONRuleConfig() {}

int VAMJSONRuleConfig::readConfig(std::string configFileName, int debugLevel)
{
    if(configFileName.size() == 0 || configFileName == "")
        return VAM_OK;

    void *pJSONInstance = NULL;
    enum JSONVAStatus ret = configuration_parser_init(&pJSONInstance);
    struct JSONConfiguration *pJSONConfig = NULL;
    ret = configuration_parser_parse_file(pJSONInstance, configFileName.c_str(), &pJSONConfig);

    if(pJSONConfig == NULL)
    {
        printf("failed to parse config file %s\n", configFileName.c_str());
        return 0;
    }

    struct JSONConfiguration *p = pJSONConfig;
    snprintf(vaapiConfig.id, VAAPI_UUID_LEN, "%s", p->id);
    snprintf(vaapiConfig.version, VAAPI_NAME_LEN, "%s", p->version);

    //switch(p->general_configuration.scene_type)
    //{
    //case SCENE_TYPE_INDOOR:
        //vaapiConfig.scene_type = vaapi_scene_indoor;
    //case SCENE_TYPE_OUTDOOR:
        //vaapiConfig.scene_type = vaapi_scene_outdoor;
        //break;
    //default:
        vaapiConfig.scene_type = vaapi_scene_unknown;
        //break;
    //}

    vaapi_zone pVAAPIZones[VAAPI_ZONE_MAX];
    size_t zone_count = MIN(p->cnt_zones, VAAPI_ZONE_MAX);
    if(zone_count > 0)
    {
        parseJSONZones( p->zones, zone_count, pVAAPIZones);
    }

    int totalEnabledRule = 0;
    for(int i=0; i<ATOMIC_RULE_MAX; i++)
        if(p->atomic_rules[i].status == VA_ENABLED)
            totalEnabledRule++;


    size_t rule_count = MIN(totalEnabledRule, VAAPI_RULE_MAX);
    vaapiConfig.rule_size = rule_count;
    int ruleIDX = 0;
    for(uint32_t i=0; i< rule_count; i++)
    {
        JSONVAAtomicRule &ar = p->atomic_rules[i];
        if(ar.status != VA_ENABLED)
            continue;

        int ret = parseJSONRule(&ar, &(vaapiConfig.rules[ruleIDX]), pVAAPIZones, zone_count);
        if(ret != VAM_OK)
            continue;
        ruleIDX++;
    }

    ret = configuration_parser_deinit(pJSONInstance);
    pJSONInstance = NULL;

    return VAM_OK;
}

void VAMJSONRuleConfig::print()
{
    std::cout << "===VAMJSONRuleConfig============================begin" << std::endl;

    vaapi_configuration &c = vaapiConfig;

    printf("  configuration_id: %s\n", c.id);
    printf("  version: %s\n", c.version);
    printf("  scene_type: %d\n", c.scene_type);
    //vaapi_scene_calibration calibration;

    for(uint32_t i=0; i<c.rule_size; i++)
    {
        vaapi_rule &r = c.rules[i];

        printf("    <rule %d>\n", i);
        printf("      id: %s\n", r.id);
        printf("      name: %s\n", r.name);
        printf("      type: %d (%s)\n", r.type,
                EventTypeToStr(r.type).c_str());
        printf("      sensitivity: %d\n", (int)r.sensitivity);
        printf("      minimum_size: %d\n", (int)r.minimum_size);
        printf("      scene_type: %d (%s)\n", r.scene_type,
                EventTypeToStr(r.scene_type).c_str());

        printf("      reserve: ");
        for(int i=0; i<5; i++)
            printf("%lld, ", r.reserve[i]);
        printf("\n");

        printf("      reserve_str: ");
        for(int i=0; i<5; i++)
            printf("%s, ", r.reserve_str[i]);
        printf("\n");

        for(uint32_t zoneIDX=0; zoneIDX<r.zone_size; zoneIDX++)
        {
            vaapi_zone &z = r.zones[zoneIDX];
            printf("      <zone %d>\n", zoneIDX);
            printf("        id:%s\n", z.id);
            if(z.point_size > 0)
                printf("        point: ");
            for(uint32_t pointIDX=0; pointIDX<z.point_size; pointIDX++)
                printf("(%d, %d) ", z.points[pointIDX].x, z.points[pointIDX].y);
            if(z.point_size > 0)
                printf("\n");
            printf("        zone_type:%d\n", z.zone_type);
        }

        //int reserve[5];
        //char reserveStr[5][VAAPI_NAME_LEN];
    }

    std::cout << "===VAMJSONRuleConfig============================end" << std::endl;
    std::cout << std::endl;
}

static int parseJSONZones( JSONVAZone *pJZ, int zoneCount, vaapi_zone *pZones)
{
    if(pJZ == NULL || pZones == NULL || zoneCount == 0)
        return VAM_NULLPTR;

    for(int i=0; i<zoneCount; i++)
    {
        snprintf(pZones[i].id, VAAPI_UUID_LEN, "%s", pJZ[i].id);
        size_t point_count = MIN(pJZ[i].cnt_points, VAAPI_POINTS_MAX);
        pZones[i].point_size = point_count;
        memset(pZones[i].points, 0, sizeof(vaapi_point)*point_count);
        for(uint32_t j=0; j<point_count; j++)
        {
            pZones[i].points[j].x = pJZ[i].points[j].x;
            pZones[i].points[j].y = pJZ[i].points[j].y;
        }

        switch(pJZ[i].type)
        {
        case ZONE_TYPE_INCLUDE: pZones[i].zone_type = vaapi_include_zone; break;
        case ZONE_TYPE_EXCLUDE: pZones[i].zone_type = vaapi_exclude_zone; break;
        case LINE_TYPE_BOTH_DIR: pZones[i].zone_type = vaapi_line; break;
        case LINE_TYPE_IN: pZones[i].zone_type = vaapi_line_in; break;
        case LINE_TYPE_OUT: pZones[i].zone_type = vaapi_line_out; break;
        default:
            printf("VASim warning: parseJSONZones found unsupported zone type %d in json file\n",
                    pJZ[i].type);
            break;
        };
    }

    return VAM_OK;
}

static int parseJSONRule( JSONVAAtomicRule *pJSONAtomicRule, vaapi_rule *pRule, vaapi_zone *pZones, int zoneSize)
{
    if(pJSONAtomicRule == NULL || pRule == NULL)
        return VAM_NULLPTR;

    JSONVAAtomicRule &ar = *pJSONAtomicRule;
    if(ar.status != VA_ENABLED)
        return VAM_OK;

    snprintf(pRule->id, VAAPI_UUID_LEN, "%s", ar.id);
    snprintf(pRule->name, VAAPI_NAME_LEN, "%s", ar.name);
    pRule->sensitivity = ar.sensitivity;
    pRule->minimum_size = ar.min_size;
    pRule->scene_type = ar.scene_type;

    //////////////// zones - start
    if(ar.cnt_zones > 0)
    {
        pRule->zone_size = MIN(ar.cnt_zones, VAAPI_ZONE_MAX);
        memset(pRule->zones, 0, sizeof(vaapi_zone)*pRule->zone_size);
    }

    for(uint32_t i=0; i<ar.cnt_zones; i++)
    {
        bool isMatched = false;
        for(int j=0; j<zoneSize; j++)
        {
            //printf("ar.zone_ids[i]:%s, pZones[j].id:%s\n", ar.zone_ids[i], pZones[j].id);
            if(0==strcmp(ar.zone_ids[i], pZones[j].id))
            {
                isMatched = true;
                //printf("(matched!)\n");
                memcpy(&pRule->zones[i], &pZones[j], sizeof(vaapi_zone));
            }
        }

        if( isMatched == false &&
                0==strcmp(ar.zone_ids[i], "00000000-0000-0000-0000-000000000001"))
        {
            //printf("(matched 000001!)\n");
            snprintf(pRule->zones[i].id, VAAPI_UUID_LEN, "00000000-0000-0000-0000-000000000001");
            pRule->zones[i].point_size = 4;
            pRule->zones[i].points[0].x = 0;
            pRule->zones[i].points[0].y = 0;
            pRule->zones[i].points[1].x = 9999;
            pRule->zones[i].points[1].y = 0;
            pRule->zones[i].points[2].x = 9999;
            pRule->zones[i].points[2].y = 9999;
            pRule->zones[i].points[3].x = 0;
            pRule->zones[i].points[3].y = 9999;
            pRule->zones[i].zone_type = vaapi_include_zone;
        }
    }
    //////////////// zones - end

    for(int i=0; i<5; i++)
    {
        pRule->reserve[i] = ar.reserve[i];
        snprintf(pRule->reserve_str[i], VAAPI_NAME_LEN, "%s", ar.reserve_str[i]);
    }

    //TODO: deal with defaults for all reserves

    switch(ar.event_type)
    {
    case METADATA_EVENT_CAMERA_TAMPER_DETECTED:
        pRule->type = vaapi_event_camera_tamper_detected;
        break;
    case METADATA_EVENT_MOTION_DETECTED:
        pRule->type = vaapi_event_motion_detected;
        break;
    case METADATA_EVENT_LINECROSSED:
        pRule->type = vaapi_event_line_crossed;
        break;
    case METADATA_EVENT_INTRUSION_DETECTED:
        pRule->type = vaapi_event_intrusion_detected;
        break;
    case METADATA_EVENT_OBJECT_COUNTED:
        pRule->type = vaapi_event_object_counted;
        break;
    case METADATA_EVENT_LOITERING_DETECTED:
        pRule->type = vaapi_event_loitering_detected;
        break;
    case METADATA_EVENT_FACE_DETECTED:
        pRule->type = vaapi_event_face_detected;
        break;
    case METADATA_EVENT_FACE_RECOGNIZED:
        pRule->type = vaapi_event_face_recognized;
        break;
    case METADATA_EVENT_OBJECT_CLASSIFIED:
        pRule->type = vaapi_event_object_classified;
        break;
    case METADATA_EVENT_OBJECT_DETECTED:
        pRule->type = vaapi_event_object_detected;
        break;
    case METADATA_EVENT_COMPOUND_EVENT:
    case METADATA_EVENT_EXTENSION_RESERVED:
    default:
        printf("VAMJSONRuleConfig::parseJSONRule error: found unsupported event: %d\n", ar.event_type);
        break;
    }

    return VAM_OK;
}

