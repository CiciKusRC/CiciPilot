/****************************************************************************
 *
 *   Copyright 2024 The MathWorks, Inc.
 *   This file is used to define the added Parameter metadata.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/sensor_combined.h>

using namespace time_literals;

extern "C" __EXPORT int px4_simulink_utils_main(int argc, char *argv[]);

class PX4SimulinkUtilityModule : public ModuleBase<PX4SimulinkUtilityModule>, public ModuleParams
{
public:
	PX4SimulinkUtilityModule(int example_param, bool example_flag);

	virtual ~PX4SimulinkUtilityModule() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static PX4SimulinkUtilityModule *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);


	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SL_FLOAT_PARAM>) _param_sl_float_param,
	        (ParamInt<px4::params::SL_INT32_PARAM>) _param_sl_int32_param
	)

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

};

