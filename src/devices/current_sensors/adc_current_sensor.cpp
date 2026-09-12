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


Vector2d ADCCurrentSensor::get_currents()
{
    // I_out = V_IPROPI / (A_IPROPI * R_IPROPI)
    return get_raw_voltages().divided_by(m_mirror_gain * m_resistor);

} // end of "ADCCurrentSensor()"


Vector2d ADCCurrentSensor::get_raw_voltages()
{
    return {m_adc.get_voltage(0), m_adc.get_voltage(1)};

} // end of "get_raw_voltages()"