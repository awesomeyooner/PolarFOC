#include "PolarFOC/devices/current_sensors/adc_current_sensor.hpp"


using namespace status_utils;


ADCCurrentSensor::ADCCurrentSensor(ADC_HandleTypeDef* adc, double resistor, double mirror_gain) :
    m_adc(adc, 2) // 2 channels, A and B
{
    m_resistor = resistor;
    m_mirror_gain = mirror_gain;

} // end of "ADCCurrentSensor(ADC_HandleTypeDef*, double, double)"


StatusCode ADCCurrentSensor::init()
{
    return m_adc.start_DMA();
    
} // end of "init()"


Vector2d ADCCurrentSensor::get_phase_currents(double signumA, double signumB)
{
    // I_out = V_IPROPI / (A_IPROPI * R_IPROPI)
    Vector2d currents = get_raw_voltages().divided_by(m_mirror_gain * m_resistor);

    // Apply signs
    // currents.times(math::sign(signumA), 0); // Phase A
    // currents.times(math::sign(signumB), 1); // Phase B

    return currents
                .times(math::sign(signumA), 0)  // Phase A
                .times(math::sign(signumB), 1); // Phase B

} // end of "get_phase_currents()"


Vector2d ADCCurrentSensor::get_dq_currents(double el_angle)
{
    return get_phase_currents().inverse_rotate(el_angle);

} // end of "get_phase_currents(double)"


Vector2d ADCCurrentSensor::get_raw_voltages()
{
    return {m_adc.get_voltage(0), m_adc.get_voltage(1)};

} // end of "get_raw_voltages()"


ADCDevice* ADCCurrentSensor::get_adc()
{
    return &m_adc;

} // end of "get_adc()"