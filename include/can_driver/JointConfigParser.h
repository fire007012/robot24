#ifndef CAN_DRIVER_JOINT_CONFIG_PARSER_H
#define CAN_DRIVER_JOINT_CONFIG_PARSER_H

#include "can_driver/CanType.h"
#include "can_driver/MotorID.h"

#include <xmlrpcpp/XmlRpcValue.h>

#include <string>
#include <vector>

namespace joint_config_parser {

struct ParsedJointConfig {
    std::string name;
    std::string canDevice;
    std::string controlMode;
    MotorID motorId{MotorID::LeftWheel};
    CanType protocol{CanType::MT};
    double positionScale{1.0};
    double velocityScale{1.0};
};

bool parse(const XmlRpc::XmlRpcValue &jointList,
           std::vector<ParsedJointConfig> &out,
           std::string &errorMsg);

bool parseMotorId(const XmlRpc::XmlRpcValue &value,
                  const std::string &jointName,
                  MotorID &out,
                  std::string &errorMsg);

} // namespace joint_config_parser

#endif // CAN_DRIVER_JOINT_CONFIG_PARSER_H
