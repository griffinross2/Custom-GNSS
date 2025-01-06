#include "filters.h"

FirstOrderPLL::FirstOrderPLL(double noise_bandwidth)
{
    w_0 = noise_bandwidth / 0.25;
}

double FirstOrderPLL::update(double input)
{
    // Update the accumulator and produce the output
    double code_error = w_0 * input;

    return code_error;
}

void FirstOrderPLL::set_bandwidth(double noise_bandwidth)
{
    w_0 = noise_bandwidth / 0.25;
}

SecondOrderPLL::SecondOrderPLL(double noise_bandwidth, double acc0)
{
    w_0 = noise_bandwidth / 0.53;
    w_0_2 = w_0 * w_0;
    a_2 = 1.414;
    acc = acc0;
}

double SecondOrderPLL::update(double input, double int_time)
{
    // Update the accumulator and produce the output
    double code_error;
    double new_acc = input * w_0_2 * int_time + acc;
    code_error = (new_acc + acc) * 0.5 + (a_2 * w_0 * input);
    acc = new_acc;

    return code_error;
}

void SecondOrderPLL::set_bandwidth(double noise_bandwidth)
{
    w_0 = noise_bandwidth / 0.53;
    w_0_2 = w_0 * w_0;
}

ThirdOrderPLL::ThirdOrderPLL(double noise_bandwidth, double acc0)
{
    w_0 = noise_bandwidth / 0.7845;
    w_0_2 = w_0 * w_0;
    w_0_3 = w_0_2 * w_0;
    a_3 = 1.1;
    b_3 = 2.4;
    acc1 = 0.0;
    acc2 = acc0;
}

double ThirdOrderPLL::update(double input, double int_time)
{
    // Update the accumulator and produce the output
    double code_error;
    double new_acc_1 = input * w_0_3 * int_time + acc1;
    double new_acc_2 = ((new_acc_1 + acc1) * 0.5 + (a_3 * w_0_2 * input)) * int_time + acc2;
    code_error = (new_acc_2 + acc2) * 0.5 + (b_3 * w_0 * input);
    acc1 = new_acc_1;
    acc2 = new_acc_2;

    return code_error;
}

void ThirdOrderPLL::set_bandwidth(double noise_bandwidth)
{
    w_0 = noise_bandwidth / 0.7845;
    w_0_2 = w_0 * w_0;
    w_0_3 = w_0_2 * w_0;
}

ThirdOrderFLLAssistedPLL::ThirdOrderFLLAssistedPLL(double noise_bandwidth_fll, double noise_bandwidth_pll, double acc0)
{
    w_0p = noise_bandwidth_pll / 0.7845;
    w_0_2p = w_0p * w_0p;
    w_0_3p = w_0_2p * w_0p;
    w_0f = noise_bandwidth_fll / 0.53;
    w_0_2f = w_0f * w_0f;
    a_3 = 1.1;
    b_3 = 2.4;
    a_2 = 1.414;
    acc1 = 0.0;
    acc2 = acc0;
}

double ThirdOrderFLLAssistedPLL::update(double fll_input, double pll_input, double int_time)
{
    // Update the accumulator and produce the output
    double code_error;
    double new_acc_1 = (pll_input * w_0_3p * int_time) + (fll_input * w_0_2f * int_time) + acc1;
    double new_acc_2 = (((new_acc_1 + acc1) * 0.5 + (a_3 * w_0_2p * pll_input) + (fll_input * a_2 * w_0f)) * int_time) + acc2;
    code_error = (new_acc_2 + acc2) * 0.5 + (b_3 * w_0p * pll_input);
    acc1 = new_acc_1;
    acc2 = new_acc_2;

    return code_error;
}

void ThirdOrderFLLAssistedPLL::set_bandwidth(double noise_bandwidth_fll, double noise_bandwidth_pll)
{
    w_0p = noise_bandwidth_pll / 0.7845;
    w_0_2p = w_0p * w_0p;
    w_0_3p = w_0_2p * w_0p;
    w_0f = noise_bandwidth_fll / 0.53;
    w_0_2f = w_0f * w_0f;
}