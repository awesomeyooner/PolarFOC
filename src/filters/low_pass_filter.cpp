#include "PolarFOC/filters/low_pass_filter.hpp"


LowPassFilter::LowPassFilter(double alpha)
{
    set_alpha(alpha);

} // end of "LowPassFilter(double)"


LowPassFilter::LowPassFilter(double alpha, double beta)
{
    m_alpha = alpha;
    m_beta = beta;

} // end of "LowPassFilter(double, double)"


void LowPassFilter::set_alpha(double alpha, bool clamp_to_one)
{
    m_alpha = alpha;

    if(clamp_to_one)
        m_beta = 1 - alpha;

} // end of "set_alpha(double, bool = true)"


void LowPassFilter::set_beta(double beta, bool clamp_to_one)
{
    m_beta = beta;

    if(clamp_to_one)
        m_alpha = 1 - beta;

} // end of "set_beta(double, bool = true)"


void LowPassFilter::update(double incoming_data)
{
    m_prev_filtered = m_current_filtered;

    m_current_filtered = (m_alpha * m_prev_filtered) + (m_beta * incoming_data);

} // end of "update(double)"


double LowPassFilter::get()
{
    return m_current_filtered;

} // end of "get()"