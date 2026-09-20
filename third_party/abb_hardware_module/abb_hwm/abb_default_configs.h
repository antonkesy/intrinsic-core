#ifndef ABB_HARDWARE_MODULE_ABB_HWM_ABB_DEFAULT_CONFIGS_H_
#define ABB_HARDWARE_MODULE_ABB_HWM_ABB_DEFAULT_CONFIGS_H_

namespace abb_hardware_module {

// This is the default EGM config that should be used for all robots which
// specify custom advanced control parameters. Of particular note is
// `run_pos_corr_gain` which is set to zero which means that convergence to
// position setpoints is handled by the control loop in the hardware module.
constexpr char kDefaultEgmWithPidConfig[] =
    R"pb(
  setup_uc_use_filtering: false
  activate_cond_min_max: 1e-10
  activate_lp_filter: 100
  activate_sample_time: 4
  activate_max_speed_deviation: 300
  run_cond_time: 1e10
  run_ramp_in_time: 0.005
  run_pos_corr_gain: 0.0
  stop_ramp_out_time: 0.005
    )pb";

// These are the control parameters that were tested on an IRB 1100-4/0.58
// robot. These parameters were tuned to reach a maximum joint velocity of 3.14
// rad/s for a given joint position setpoint of 3.14 rad.
constexpr char kIrb1100_4_058_DefaultPidConfig[] =
    R"pb(
  cycle_time_seconds: 0.004
  k_p: 3
  k_p: 3
  k_p: 3
  k_p: 3
  k_p: 3
  k_p: 3
  k_i: 0
  k_i: 0
  k_i: 0
  k_i: 0
  k_i: 0
  k_i: 0
  k_d: 0.5
  k_d: 0.5
  k_d: 0.5
  k_d: 0.5
  k_d: 0.5
  k_d: 0.5
  k_ff: 0.85
  k_ff: 0.85
  k_ff: 0.9
  k_ff: 0.9
  k_ff: 0.9
  k_ff: 0.9
  max_integral_control: 0.001
  max_integral_control: 0.001
  max_integral_control: 0.001
  max_integral_control: 0.001
  max_integral_control: 0.001
  max_integral_control: 0.001
  max_velocity_command: 8.02
  max_velocity_command: 6.28
  max_velocity_command: 4.88
  max_velocity_command: 9.77
  max_velocity_command: 7.33
  max_velocity_command: 13.08
  position_filter_cuttoff_frequency_hz: 35
  velocity_filter_cuttoff_frequency_hz: 25
    )pb";

// These are the control parameters that were tested on an IRB 1300-10/1.15
// robot.
constexpr char kIrb1300_10_115_DefaultPidConfig[] =
    R"pb(
  cycle_time_seconds: 0.004
  k_p: 3
  k_p: 3
  k_p: 3
  k_p: 3
  k_p: 3
  k_p: 3
  k_i: 0
  k_i: 0
  k_i: 0
  k_i: 0
  k_i: 0
  k_i: 0
  k_d: 0.45
  k_d: 0.45
  k_d: 0.45
  k_d: 0.45
  k_d: 0.45
  k_d: 0.45
  k_ff: 0.9
  k_ff: 0.9
  k_ff: 0.9
  k_ff: 0.9
  k_ff: 0.9
  k_ff: 0.9
  max_integral_control: 0.001
  max_integral_control: 0.001
  max_integral_control: 0.001
  max_integral_control: 0.001
  max_integral_control: 0.001
  max_integral_control: 0.001
  max_velocity_command: 4.88
  max_velocity_command: 3.97
  max_velocity_command: 5.86
  max_velocity_command: 8.72
  max_velocity_command: 7.24
  max_velocity_command: 12.56
  position_filter_cuttoff_frequency_hz: 35
  velocity_filter_cuttoff_frequency_hz: 25
    )pb";

}  // namespace abb_hardware_module

#endif  // ABB_HARDWARE_MODULE_ABB_HWM_ABB_DEFAULT_CONFIGS_H_
