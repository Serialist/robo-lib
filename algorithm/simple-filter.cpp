/**
 * @file simple-filter.cpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-05-01
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#include "simple-filter.hpp"

vgd::algorithm::LowPass_Order_1::LowPass_Order_1(float cutoff_frequency, float sample_rate)
{
	this->cutoff_frequency = cutoff_frequency;
	this->sample_rate = sample_rate;

	alpha = 2.0f * cutoff_frequency / sample_rate;

	output = 0;
}

vgd::algorithm::LowPass_Order_1::LowPass_Order_1(float alpha)
{
	this->alpha = alpha;

	cutoff_frequency = 0;
	sample_rate = 0;
	output = 0;
}

float vgd::algorithm::LowPass_Order_1::update(float input)
{
	output = alpha * input + (1.0f - alpha) * output;
	return output;
}

vgd::algorithm::ZeroOrder_Holder::ZeroOrder_Holder(void)
{
	prev_value = 0;
}

float vgd::algorithm::ZeroOrder_Holder::update(float input)
{
	if (input == 0)
	{
		return prev_value;
	}
	else
	{
		prev_value = input;
		return input;
	}
}
