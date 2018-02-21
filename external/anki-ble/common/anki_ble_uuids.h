/**
 * File: anki_ble_uuids.h
 *
 * Author: seichert
 * Created: 2/9/2018
 *
 * Description: Anki BLE UUIDs
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#pragma once

#include <string>

// This header defines constants specific to the Anki BLE GATT Service.

namespace Anki {

const std::string kAnkiBLEService_128_BIT_UUID("D55E356B-59CC-4265-9D5F-3C61E9DFD70F");
const std::string kAppWriteCharacteristicUUID("7D2A4BDA-D29B-4152-B725-2491478C5CD7");
const std::string kAppReadCharacteristicUUID("30619F2D-0F54-41BD-A65A-7588D8C85B45");
const std::string kAppWriteEncryptedCharacteristicUUID("045C8155-3D7B-41BC-9DA0-0ED27D0C8A61");
const std::string kAppReadEncryptedCharacteristicUUID("28C35E4C-B218-43CB-9718-3D7EDE9B5316");
const std::string kCCCDescriptorUUID("00002902-0000-1000-8000-00805F9B34FB");

} // namespace Anki
