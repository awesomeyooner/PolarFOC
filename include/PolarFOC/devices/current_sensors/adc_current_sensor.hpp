#ifndef ADC_CURRENT_SENSOR
#define ADC_CURRENT_SENSOR


#include "EmbeddedLib/status.hpp"

#include "EmbeddedLib/devices/adc_device.hpp"

#include "EmbeddedLib/math/vector2d.hpp"
#include "EmbeddedLib/math/math_util.hpp"


/**
 * @brief This class represents dual Lowside Current sensor for a stepper motor,
 * one for Phase A and the other for Phase B. 
 * 
 * `init()` Must be called 
 * 
 */
class ADCCurrentSensor
{

    public:

        ADCCurrentSensor(ADC_HandleTypeDef* adc, double resistor, double mirror_gain);

        status_utils::StatusCode init();

        Vector2d get_phase_currents(double signumA = 1, double signumB = 1);

        Vector2d get_dq_currents(double el_angle);

        Vector2d get_raw_voltages();

        ADCDevice* get_adc();

    private:

        ADCDevice m_adc;

        double m_resistor = 1;

        double m_mirror_gain = 1;


}; // class ADCCurrentSensor


#endif // ADC_CURRENT_SENSOR