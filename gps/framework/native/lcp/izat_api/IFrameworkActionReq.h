/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*

GENERAL DESCRIPTION
  IFrameworkActionReq

  Copyright  (c) 2017 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.
=============================================================================*/

#ifndef __IZAT_MANAGER_IFRAMEWORKACTIONREQ_H__
#define __IZAT_MANAGER_IFRAMEWORKACTIONREQ_H__

#include  <DataItemId.h>

namespace izat_manager
{

/**
 * @brief IFrameworkActionReq interface
 * @details IFrameworkActionReq interface;
 *          Defines an interface for operations such as turnOn, turnOff a
 *          framework module described by the data item. Framework module
 *          could be bluetooth, wifi etc.
 *          Must be implemented by OS dependent code.
 *
 */
class IFrameworkActionReq {

public:
    /**
     * @brief Turn on the framework module described by the data item.
     * @details  Turn on the framework module described by the data item;
     *          An IFrameworkActionReq implementer invokes this method to
     *          turn on the framework module described by the data item.
     *          Framework module could be bluetooth, wifi etc.
     *
     * @param dit DataItemId
     * @param timeout Timeout after which to turn off the framework module.
     */
    virtual void turnOn (DataItemId dit, int timeOut = 0) = 0;

    /**
     * @brief Turn off the framework module described by the data item.
     * @details  Turn off the framework module described by the data item;
     *          An IFrameworkActionReq implementer invokes this method to
     *          turn off the framework module described by the data item.
     *          Framework module could be bluetooth, wifi etc.
     *
     * @param dit DataItemId
     */
    virtual void turnOff (DataItemId dit) = 0;

    /**
     * @brief Destructor
     * @details Destructor
     */
    virtual ~IFrameworkActionReq () {}
};

} // namespace izat_manager

#endif // #ifndef __IZAT_MANAGER_IFRAMEWORKACTIONREQ_H__

