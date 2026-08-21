#ifndef LOW_PASS_FILTER_HPP
#define LOW_PASS_FILTER_HPP


#include "PolarFOC/filters/filter.hpp"


class LowPassFilter : public Filter
{

    public:

        LowPassFilter(double alpha);

        LowPassFilter(double alpha, double beta);

        void set_alpha(double alpha, bool clamp_to_one = true);

        void set_beta(double beta, bool clamp_to_one = true);

        void update(double incoming_data) override;

        double get() override;

    private:

        double m_alpha = 0;
        double m_beta = 0;

        double m_prev_filtered = 0;
        double m_current_filtered = 0;

}; // class LowPassFilter : public Filter


#endif // LOW_PASS_FILTER_HPP