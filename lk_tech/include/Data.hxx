#ifndef __DATA_HXX__
#define __DATA_HXX__

#include <cstdint>
#include <vector>
#include <string>

struct Data{

    std::vector<std::uint8_t> model_cmd_;
    std::vector<std::uint8_t> model_response_;

    std::vector<std::uint8_t> read_pid_cmd_;
    std::vector<std::uint8_t> read_pid_response_;

    std::vector<std::uint8_t> write_pid_ram_cmd_;
    std::vector<std::uint8_t> write_pid_rom_cmd_;

    std::vector<std::uint8_t> read_acc_cmd_;
    std::vector<std::uint8_t> read_acc_response_;

    std::vector<std::uint8_t> write_acc_cmd_;

    std::vector<std::uint8_t> read_enc_cmd_;
    std::vector<std::uint8_t> read_enc_response_;

    std::vector<std::uint8_t> write_temp_enc_offset_cmd_;
    std::vector<std::uint8_t> write_perm_enc_offset_cmd_;
    std::uint16_t encoder_mask_{(1 << 14) - 1};

    std::vector<std::uint8_t> read_multi_turn_cmd_;
    std::vector<std::uint8_t> read_multi_turn_response_;
    
    std::vector<std::uint8_t> read_single_turn_cmd_;
    std::vector<std::uint8_t> read_single_turn_response_;
    
    std::vector<std::uint8_t> read_temperature_and_voltage_cmd_;
    std::vector<std::uint8_t> read_temperature_and_voltage_response_;

    std::vector<std::uint8_t> clear_error_cmd_;
    std::vector<std::uint8_t> clear_error_response_;

    std::vector<std::uint8_t> read_temperature_and_torque_speed_pose_cmd_;
    std::vector<std::uint8_t> read_temperature_and_torque_speed_pose_response_;

    std::vector<std::uint8_t> read_temperature_and_phase_current_cmd_;
    std::vector<std::uint8_t> read_temperature_and_phase_current_response_;

    std::vector<std::uint8_t> shutdown_cmd_;
    std::vector<std::uint8_t> shutdown_response_;

    std::vector<std::uint8_t> stop_cmd_;
    std::vector<std::uint8_t> stop_response_;

    std::vector<std::uint8_t> turn_on_cmd_;
    std::vector<std::uint8_t> turn_on_response_;

    std::vector<std::uint8_t> open_loop_torque_cmd_;
    std::vector<std::uint8_t> open_loop_torque_response_;

    std::vector<std::uint8_t> closed_loop_torque_cmd_;
    std::vector<std::uint8_t> closed_loop_torque_response_;

    std::vector<std::uint8_t> closed_loop_speed_cmd_;
    std::vector<std::uint8_t> closed_loop_speed_response_;

    std::vector<std::uint8_t> closed_loop_multi_turn_position_1_cmd_;
    std::vector<std::uint8_t> closed_loop_multi_turn_position_1_response_;

    std::vector<std::uint8_t> closed_loop_multi_turn_position_2_cmd_;
    std::vector<std::uint8_t> closed_loop_multi_turn_position_2_response_;

    std::vector<std::uint8_t> closed_loop_single_turn_position_1_cmd_;
    std::vector<std::uint8_t> closed_loop_single_turn_position_1_response_;

    std::vector<std::uint8_t> closed_loop_single_turn_position_2_cmd_;
    std::vector<std::uint8_t> closed_loop_single_turn_position_2_response_;

    /*NOT WORKING*/
    // std::vector<std::uint8_t> closed_loop_incremental_turn_position_1_cmd_;
    // std::vector<std::uint8_t> closed_loop_incremental_turn_position_1_response_;

    std::vector<std::uint8_t> closed_loop_incremental_turn_position_2_cmd_;
    std::vector<std::uint8_t> closed_loop_incremental_turn_position_2_response_;
};


#endif