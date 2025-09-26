#ifndef FILTERS_H
#define FILTERS_H

// Reference: Understanding GPS/GNSS Principles and Applications by Elliott D. Kaplan (3rd Edition)

class PLL
{
public:
    PLL() {}
    virtual double update(double input) { return 0; }
    virtual double update(double input, double int_time) { return 0; }
    virtual double update(double fll_input, double pll_input, double int_time) { return 0; }
    virtual void set_bandwidth(double noise_bandwidth) {}
    virtual void set_bandwidth(double noise_bandwidth_fll, double noise_bandwidth_pll) {}
};

class FirstOrderPLL : public PLL
{
public:
    FirstOrderPLL(double noise_bandwidth);

    double update(double input);
    void set_bandwidth(double noise_bandwidth);

private:
    double w_0; // Natural frequency
};

class SecondOrderPLL : public PLL
{
public:
    SecondOrderPLL(double noise_bandwidth, double acc0 = 0.0);

    double update(double input, double int_time);
    void set_bandwidth(double noise_bandwidth);

private:
    double w_0;   // Natural frequency
    double w_0_2; // Squared natural frequency
    double a_2;   // Coefficient
    double acc;   // Accumulator
};

class ThirdOrderPLL : public PLL
{
public:
    ThirdOrderPLL(double noise_bandwidth, double acc0 = 0.0);

    double update(double input, double int_time);
    void set_bandwidth(double noise_bandwidth);

private:
    double w_0;   // Natural frequency
    double w_0_2; // Squared natural frequency
    double w_0_3; // Cubed natural frequency
    double a_3;   // Coefficient
    double b_3;   // Coefficient
    double acc1;  // First Accumulator
    double acc2;  // Second Accumulator
};

class ThirdOrderFLLAssistedPLL : public PLL
{
public:
    ThirdOrderFLLAssistedPLL(double noise_bandwidth_fll, double noise_bandwidth_pll, double acc0 = 0.0);

    double update(double fll_input, double pll_input, double int_time);
    void set_bandwidth(double noise_bandwidth_fll, double noise_bandwidth_pll);

private:
    double w_0p;   // PLL Natural frequency
    double w_0_2p; // PLL Squared natural frequency
    double w_0_3p; // PLL Cubed natural frequency
    double w_0f;   // FLL Natural frequency
    double w_0_2f; // FLL Squared natural frequency
    double a_3;    // Coefficient
    double b_3;    // Coefficient
    double a_2;    // Coefficient
    double acc1;   // First Accumulator
    double acc2;   // Second Accumulator
};

class SecondOrderFLLAssistedPLL : public PLL
{
public:
    SecondOrderFLLAssistedPLL(double noise_bandwidth_fll, double noise_bandwidth_pll, double acc0 = 0.0);

    double update(double fll_input, double pll_input, double int_time);
    void set_bandwidth(double noise_bandwidth_fll, double noise_bandwidth_pll);

private:
    double w_0p;   // PLL Natural frequency
    double w_0_2p; // PLL Squared natural frequency
    double w_0f;   // FLL Natural frequency
    double a_2;    // Coefficient
    double acc1;   // First Accumulator
};

#endif // FILTERS_H
