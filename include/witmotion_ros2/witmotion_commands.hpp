#ifndef WITMOTION_COMMANDS_HPP
#define WITMOTION_COMMANDS_HPP

#include <vector>
#include <cstdint>

namespace WitMotionCommands
{
    const std::vector<uint8_t> UNLOCK_IMU_CMD = {0xff, 0xaa, 0x69, 0x88, 0xb5};
    const std::vector<uint8_t> RESET_MAGX_OFFSET_CMD = {0xff, 0xaa, 0x0b, 0x00, 0x00};
    const std::vector<uint8_t> RESET_MAGY_OFFSET_CMD = {0xff, 0xaa, 0x0c, 0x00, 0x00};
    const std::vector<uint8_t> RESET_MAGZ_OFFSET_CMD = {0xff, 0xaa, 0x0d, 0x00, 0x00};
    const std::vector<uint8_t> RESET_MAG_PARAM_CMD = {0xff, 0xaa, 0x01, 0x07, 0x00};
    const std::vector<uint8_t> ENTER_MAG_CALI_CMD = {0xff, 0xaa, 0x01, 0x09, 0x00};
    const std::vector<uint8_t> EXIT_CALI_CMD = {0xff, 0xaa, 0x01, 0x00, 0x00};
    const std::vector<uint8_t> SAVE_PARAM_CMD = {0xff, 0xaa, 0x00, 0x00, 0x00};
    const std::vector<uint8_t> READ_MAG_OFFSET_CMD = {0xff, 0xaa, 0x27, 0x0b, 0x00};
    const std::vector<uint8_t> READ_MAG_RANGE_CMD = {0xff, 0xaa, 0x27, 0x1c, 0x00};
    const std::vector<uint8_t> REBOOT_CMD = {0xff, 0xaa, 0x00, 0xff, 0x00};
    const std::vector<uint8_t> SET_RSW_DEMO_CMD = {0xff, 0xaa, 0x02, 0x1f, 0x00};
}

#endif // WITMOTION_COMMANDS_HPP
