#include <gtest/gtest.h>

#include "can_driver/JointConfigParser.h"

#include <string>

using joint_config_parser::parseMotorId;

TEST(JointConfigParser, ValidIntMotorId)
{
    XmlRpc::XmlRpcValue v(42);
    MotorID out;
    std::string err;
    EXPECT_TRUE(parseMotorId(v, "test", out, err));
    EXPECT_EQ(static_cast<uint16_t>(out), 42);
}

TEST(JointConfigParser, ValidHexStringMotorId)
{
    XmlRpc::XmlRpcValue v(std::string("0x1A"));
    MotorID out;
    std::string err;
    EXPECT_TRUE(parseMotorId(v, "test", out, err));
    EXPECT_EQ(static_cast<uint16_t>(out), 0x1A);
}

TEST(JointConfigParser, NegativeMotorIdFails)
{
    XmlRpc::XmlRpcValue v(-1);
    MotorID out;
    std::string err;
    EXPECT_FALSE(parseMotorId(v, "test", out, err));
    EXPECT_FALSE(err.empty());
}

TEST(JointConfigParser, OverflowMotorIdFails)
{
    XmlRpc::XmlRpcValue v(70000);
    MotorID out;
    std::string err;
    EXPECT_FALSE(parseMotorId(v, "test", out, err));
}

TEST(JointConfigParser, InvalidStringMotorIdFails)
{
    XmlRpc::XmlRpcValue v(std::string("not_a_number"));
    MotorID out;
    std::string err;
    EXPECT_FALSE(parseMotorId(v, "test", out, err));
}
