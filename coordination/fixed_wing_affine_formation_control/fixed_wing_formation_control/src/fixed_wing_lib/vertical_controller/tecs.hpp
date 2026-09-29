/****************************************************************************
 *
 *   Copyright (c) 2017 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file tecs.cpp
 *
 * @author Paul Riseborough
 * 
 */

//***可以换用更靠近XTDrone版本的TECS参数，效果应该会更好
#pragma once

#include "../mathlib.hpp"
using namespace std;

class TECS
{
public:
	TECS() = default;
	~TECS() = default;

	// 无副本，分配，移动，移动分配
	TECS(const TECS &) = delete;
	TECS &operator=(const TECS &) = delete;
	TECS(TECS &&) = delete;
	TECS &operator=(TECS &&) = delete;

	/**
	 * 获得当前空速状态
	 *
	 * @return 如果启用的空速控制则返回true
	 */
	bool airspeed_sensor_enabled() { return _airspeed_enabled; }

	/**
	 * 设置空速启用状态
	 */
	void enable_airspeed(bool enabled) { _airspeed_enabled = enabled; }

	/**
	 * 更新僚机运动学状态估计:
	 * 垂直位置，速度，加速度.
	 * 速度导数
	 * 必须在更新TECS控制循环之前调用
	 * 必须在50Hz或更高的频率时调用
	 */
	void update_vehicle_state_estimates(float airspeed, const float rotMat[3][3],
										const float accel_body[3], bool altitude_lock, bool in_air,
										float altitude, bool vz_valid, float vz, float az);

	/**
	 * 更新控制循环计算
	 */
	void update_pitch_throttle(const float rotMat[3][3], float pitch, float baro_altitude, float hgt_setpoint,
							   float EAS_setpoint, float indicated_airspeed, float eas_to_tas, bool climb_out_setpoint, float pitch_min_climbout,
							   float throttle_min, float throttle_setpoint_max, float throttle_cruise,
							   float pitch_limit_min, float pitch_limit_max);
	/**
	 * 控制器输出-获得归一化的需求油门
	 */
	float get_throttle_setpoint(void) { return _throttle_setpoint; }
	
	/**
	 * @brief 控制器输出-获得俯仰角度需求（弧度）
	 * 
	 * @return float 俯仰角度需求（弧度）
	 */
	float get_pitch_setpoint() { return _pitch_setpoint; }

	/**
	 * @brief 获得用于计算俯仰角的速度控制权重
	 * 
	 * @return float 速度控制权重
	 */
	float get_speed_weight() { return _pitch_speed_weight; }
	
	/**
	 * @brief 重置TECS状态
	 * 
	 */
	void reset_state() { _states_initalized = false; }

	enum ECL_TECS_MODE //定义枚举类型和枚举常量表
	{
		ECL_TECS_MODE_NORMAL = 0,
		ECL_TECS_MODE_UNDERSPEED,
		ECL_TECS_MODE_BAD_DESCENT,
		ECL_TECS_MODE_CLIMBOUT
	};

	void set_detect_underspeed_enabled(bool enabled) { _detect_underspeed_enabled = enabled; }

	// 制定控制器参数
	void set_time_const(float time_const) { _pitch_time_constant = time_const; }
	void set_integrator_gain(float gain) { _integrator_gain = gain; }

	void set_min_sink_rate(float rate) { _min_sink_rate = rate; }
	void set_max_sink_rate(float sink_rate) { _max_sink_rate = sink_rate; }
	void set_max_climb_rate(float climb_rate) { _max_climb_rate = climb_rate; }

	void set_height_comp_filter_omega(float omega) { _hgt_estimate_freq = omega; }
	void set_heightrate_ff(float heightrate_ff) { _height_setpoint_gain_ff = heightrate_ff; }
	void set_heightrate_p(float heightrate_p) { _height_error_gain = heightrate_p; }

	void set_indicated_airspeed_max(float airspeed) { _indicated_airspeed_max = airspeed; }
	void set_indicated_airspeed_min(float airspeed) { _indicated_airspeed_min = airspeed; }

	void set_pitch_damping(float damping) { _pitch_damping_gain = damping; }
	void set_vertical_accel_limit(float limit) { _vert_accel_limit = limit; }

	void set_speed_comp_filter_omega(float omega) { _tas_estimate_freq = omega; }
	void set_speed_weight(float weight) { _pitch_speed_weight = weight; }
	void set_speedrate_p(float speedrate_p) { _speed_error_gain = speedrate_p; }

	void set_time_const_throt(float time_const_throt) { _throttle_time_constant = time_const_throt; }
	void set_throttle_damp(float throttle_damp) { _throttle_damping_gain = throttle_damp; }
	void set_throttle_slewrate(float slewrate) { _throttle_slewrate = slewrate; }

	void set_roll_throttle_compensation(float compensation) { _load_factor_correction = compensation; }

	// TECS 状态
	uint64_t timestamp() { return _pitch_update_timestamp; }
	ECL_TECS_MODE tecs_mode() { return _tecs_mode; }

	float hgt_setpoint_adj() { return _hgt_setpoint_adj; }
	float vert_pos_state() { return _vert_pos_state; }

	float TAS_setpoint_adj() { return _TAS_setpoint_adj; }
	float tas_state() { return _tas_state; }

	float hgt_rate_setpoint() { return _hgt_rate_setpoint; }
	float vert_vel_state() { return _vert_vel_state; }

	float TAS_rate_setpoint() { return _TAS_rate_setpoint; }
	float speed_derivative() { return _speed_derivative; }

	float STE_error() { return _STE_error; }
	float STE_rate_error() { return _STE_rate_error; }

	float SEB_error() { return _SEB_error; }
	float SEB_rate_error() { return _SEB_rate_error; }

	float throttle_integ_state() { return _throttle_integ_state; }
	float pitch_integ_state() { return _pitch_integ_state; }

	/**
	 * 处理高度重置
	 *
	 * 如果估计系统在一个离散步骤中重置高度，则会随着时间的推移优雅地平衡重置。
	 */
	void handle_alt_step(float delta_alt, float altitude)
	{
		// 将高度重置增量添加到所涉及的所有变量
		// 滤波所需的高度
		_hgt_setpoint_in_prev += delta_alt;
		_hgt_setpoint_prev += delta_alt;
		_hgt_setpoint_adj_prev += delta_alt;

		// 重置高度状态
		_vert_pos_state = altitude;
		_vert_accel_state = 0.0f;
		_vert_vel_state = 0.0f;
	}

private:
	enum ECL_TECS_MODE _tecs_mode
	{
		ECL_TECS_MODE_NORMAL
	};

	// 时间戳
	uint64_t _state_update_timestamp{0}; ///< 50 Hz函数调用的最后时间戳
	uint64_t _speed_update_timestamp{0}; ///< 速度函数调用的最后时间戳
	uint64_t _pitch_update_timestamp{0}; ///< 音高函数调用的最后时间戳

	// 控制器参数
	float _hgt_estimate_freq{3.0f};		  ///< 高度互补滤波器（RAD / SEC）的交叉频率
	float _tas_estimate_freq{2.0f};		  ///< 真正的空速互补滤波器的交叉频率（RAD / SEC）
	float _max_climb_rate{5.0f};		  ///< 允许最大油门（M / SEC）产生的爬升
	float _min_sink_rate{2.0f};			  ///< 由最小允许油门产生的下降率 (m/sec)
	float _max_sink_rate{5.0f};			  ///< 最大安全下降率 (m/sec)
	float _pitch_time_constant{5.0f};	 ///< 音高需求计算使用的控制时间常数 (sec)
	float _throttle_time_constant{8.0f};  ///< 计算油门的控制时间常数 (sec)
	float _pitch_damping_gain{0.3f};	  ///< 阻尼距离计算计算的增益 (sec)   0.0f 这里会影响俯仰角变化快吗？
	float _throttle_damping_gain{0.5f};   ///< 计算油门当衰减增益 (sec)
	float _integrator_gain{0.1f};		  ///< 计算油门和俯仰角使用的积分器增益
	float _vert_accel_limit{10.0f};		  ///< 允许最大垂直加速度的幅度 (m/sec**2)
	float _load_factor_correction{15.0f}; ///< 从正常负载因子增加到总能量率的增益 (m**2/sec**3)
	float _pitch_speed_weight{1.0f};	  ///< 计算俯仰角使用的速度控制加权
	float _height_error_gain{0.05f};	  ///< 从高度误差到所需爬升率的增益 (1/sec)
	float _height_setpoint_gain_ff{0.8f}; ///< 从高度需求衍生物到需求爬升率的增益
	float _speed_error_gain{0.02f};		  ///< 从速度误差到需求速率的增益 (1/sec)
	float _indicated_airspeed_min{3.0f};  ///< 等效空速需求下限(m/sec)
	float _indicated_airspeed_max{30.0f}; ///< 等效空速需求上限 (m/sec)
	float _throttle_slewrate{0.0f};		  ///< 油门转换速率限制的需求 (1/sec)

	// 控制器输出
	float _throttle_setpoint{0.0f}; ///< 归一化的油门需求 (0..1)
	float _pitch_setpoint{0.0f};	///< 俯仰角度需求（弧度）

	// 互补滤波状态量
	float _vert_accel_state{0.0f}; ///< 互补滤波器状态 - 高度第二衍生物 (m/sec**2)
	float _vert_vel_state{0.0f};   ///< 互补滤波器状态 - 高度率 (m/sec)
	float _vert_pos_state{0.0f};   ///< 互补滤波器状态 - 高度 (m)
	float _tas_rate_state{0.0f};   ///< 互补滤波器状态 - 真正的Airspeed首次衍生 (m/sec**2)
	float _tas_state{0.0f};		   ///< 互补滤波器状态 - 真正的空速 (m/sec)

	// 控制器状态
	float _throttle_integ_state{0.0f};   ///< 油门积分器状态
	float _pitch_integ_state{0.0f};		 ///< 俯仰积分器状态（RAD）
	float _last_throttle_setpoint{0.0f}; ///< 油门需求率限制器状态 (1/sec)
	float _last_pitch_setpoint{0.0f};	///< 俯仰角需求利率限制器状态 (rad/sec)
	float _speed_derivative{0.0f};		 ///< 沿X轴的速度变化率 (m/sec**2)

	// 速度需求计算
	float _EAS{0.0f};				///< 当量空速 (m/sec)
	float _TAS_max{30.0f};			///< 真正的空速需求上限 (m/sec)
	float _TAS_min{3.0f};			///< 真正的空速需求下限 (m/sec)
	float _TAS_setpoint{0.0f};		///< 当前需求空速 (m/sec)
	float _TAS_setpoint_last{0.0f}; ///< 以前真正的Airpeed需求 (m/sec)
	float _EAS_setpoint{0.0f};		///< 等效空速需求 (m/sec)
	float _TAS_setpoint_adj{0.0f};  ///< TECS算法跟踪真正的空速需求 (m/sec)
	float _TAS_rate_setpoint{0.0f}; ///< TECS算法跟踪真正的空速率需求 (m/sec**2)

	// 高度需求计算
	float _hgt_setpoint{0.0f};			///< TECS算法跟踪的要求高度 (m)
	float _hgt_setpoint_in_prev{0.0f};  ///< 噪声过滤后的_hgt_setpoint的先前值 (m)
	float _hgt_setpoint_prev{0.0f};		///< 噪声过滤和速率限制后的_hgt_setpoint的先前值 (m)
	float _hgt_setpoint_adj{0.0f};		///< 所有滤波后控制循环使用的要求高度 (m)
	float _hgt_setpoint_adj_prev{0.0f}; ///< _hgt_setpoint_adj中的值来自上一帧的值 (m)
	float _hgt_rate_setpoint{0.0f};		///< TECS算法跟踪的要求攀升速率

	// 无人机物理限制
	float _pitch_setpoint_unc{0.0f};	///< 限制之前的俯仰角需求 (rad)
	float _STE_rate_max{0.0f};			///< 当油门位于_throttle_setpoint_max时达到的具体的总能量率上限 (m**2/sec**3)
	float _STE_rate_min{0.0f};			///< 当油门位于_throttle_setpoint_min时达到的具体的总能量率下限  (m**2/sec**3)
	float _throttle_setpoint_max{0.0f}; ///< 归一化的油门上限
	float _throttle_setpoint_min{0.0f}; ///< 归一化的油门下限
	float _pitch_setpoint_max{0.5f};	///< 需求俯仰角上限 (rad)
	float _pitch_setpoint_min{-0.5f};   ///< 需求俯仰角下限 (rad)

	// 比能量参数
	float _SPE_setpoint{0.0f};		///< 特定的潜在能源需求 (m**2/sec**2)
	float _SKE_setpoint{0.0f};		///< 特定的动能需求 (m**2/sec**2)
	float _SPE_rate_setpoint{0.0f}; ///< 特定的潜在能量率需求 (m**2/sec**3)
	float _SKE_rate_setpoint{0.0f}; ///< 特定的动能需求 (m**2/sec**3)
	float _SPE_estimate{0.0f};		///< 特定的潜在能量估计 (m**2/sec**2)
	float _SKE_estimate{0.0f};		///< 特定的动能估计 (m**2/sec**2)
	float _SPE_rate{0.0f};			///< 特定的潜在能量率估计 (m**2/sec**3)
	float _SKE_rate{0.0f};			///< 特定的动能率估计 (m**2/sec**3)

	// 特定能量误差数量
	float _STE_error{0.0f};		 ///< 特定的总能量误差 (m**2/sec**2)
	float _STE_rate_error{0.0f}; ///< 特定总能量率误差 (m**2/sec**3)
	float _SEB_error{0.0f};		 ///< 特定能量平衡误差 (m**2/sec**2)
	float _SEB_rate_error{0.0f}; ///< 特定能量平衡率误差 (m**2/sec**3)

	// 时间间隔（非固定）
	float _dt{DT_DEFAULT};					   ///< 自上次更新主要TECS循环以来的时间 (sec)
	static constexpr float DT_DEFAULT = 0.02f; ///< _dt的默认值 (sec)

	// 控制器模式逻辑
	bool _underspeed_detected{false};		   ///< 当检测到低速工况是是true 
	bool _detect_underspeed_enabled{true};	 ///< 当低速检测启用时是true
	bool _uncommanded_descent_recovery{false}; ///< 当已经检测到由无法实现的空速需求引起的连续下降时是true
	bool _climbout_mode_active{false};		   ///< 在爬行模式下是true
	bool _airspeed_enabled{false};			   ///< 当启用Airspeed时是true
	bool _states_initalized{false};			   ///< 当TECS状态已初始化时true
	bool _in_air{false};					   ///< 当无人机起飞后是true

	/**
	 * 使用二阶互补滤波器更新空速内部状态
	 */
	void _update_speed_states(float airspeed_setpoint, float indicated_airspeed, float eas_to_tas);

	/**
	 * 更新期望空速
	 */
	void _update_speed_setpoint();

	/**
	 * 更新期望高度
	 */
	void _update_height_setpoint(float desired, float state);

	/**
	 * 检测系统是否不能维持空速
	 */
	void _detect_underspeed();

	/**
	 * 更新特定的能量
	 */
	void _update_energy_estimates();

	/**
	 * 更新油门设定值
	 */
	void _update_throttle_setpoint(float throttle_cruise, const float rotMat[3][3]);

	/**
	 * 检测未经命令的下降
	 */
	void _detect_uncommanded_descent();

	/**
	 * 更新俯仰角设定值
	 */
	void _update_pitch_setpoint();

	/**
	 * 初始化控制器
	 */
	void _initialize_states(float pitch, float throttle_cruise, float baro_altitude, float pitch_min_climbout,
							float eas_to_tas);

	/**
	 * 计算特定的总能量率限制
	 */
	void _update_STE_rate_lim();
};
