#ifndef _TOPIC_H_
#define _TOPIC_H_


            /***********************************************************************/
            /*************************Node Environment******************************/
            /***********************************************************************/
#define node_environment_properties_pH_payload            "mandevices/thap_cho_ca/device_0001/environment/pH"
#define node_environment_properties_pH_threshold          "mandevices/thap_cho_ca/device_0001/environment/pH/$threshold"
#define node_environment_properties_pH_threshold_set      "mandevices/thap_cho_ca/device_0001/environment/pH/$threshold/set"

#define node_environment_properties_DO_payload            "mandevices/thap_cho_ca/device_0001/environment/o2_gas"
#define node_environment_properties_DO_threshold          "mandevices/thap_cho_ca/device_0001/environment/o2_gas/$threshold"
#define node_environment_properties_DO_threshold_set          "mandevices/thap_cho_ca/device_0001/environment/o2_gas/$threshold/set"

#define node_environment_properties_Temperature_payload   "mandevices/thap_cho_ca/device_0001/environment/temperature"
#define node_environment_properties_Temperature_threshold "mandevices/thap_cho_ca/device_0001/environment/temperature/$threshold"
#define node_environment_properties_Temperature_threshold_set "mandevices/thap_cho_ca/device_0001/environment/temperature/$threshold/set"


            /***********************************************************************/
            /*****************************Node Device*******************************/
            /***********************************************************************/

#define node_device_properties_foot_can_payload     "mandevices/thap_cho_ca/device_0001/device/foot_can"
#define node_device_properties_foot_can_cmdset      "mandevices/thap_cho_ca/device_0001/device/foot_can/set"
#define node_device_properties_foot_can_settable    "mandevices/thap_cho_ca/device_0001/device/foot_can/$settable"

#define node_device_properties_foot_tray_payload    "mandevices/thap_cho_ca/device_0001/device/foot_tray"
#define node_device_properties_foot_tray_cmdset     "mandevices/thap_cho_ca/device_0001/device/foot_tray/set"
#define node_device_properties_foot_tray_settable   "mandevices/thap_cho_ca/device_0001/device/foot_tray/$settable"

#define node_device_properties_fan_payload          "mandevices/thap_cho_ca/device_0001/device/fan"
#define node_device_properties_fan_cmdset           "mandevices/thap_cho_ca/device_0001/device/fan/set"
#define node_device_properties_fan_settable         "mandevices/thap_cho_ca/device_0001/device/fan/$settable"

#define node_device_properties_cylinder_payload          "mandevices/thap_cho_ca/device_0001/device/cylinder"
#define node_device_properties_cylinder_cmdset           "mandevices/thap_cho_ca/device_0001/device/cylinder/set"
#define node_device_properties_cylinder_settable         "mandevices/thap_cho_ca/device_0001/device/cylinder/$settable"

#endif