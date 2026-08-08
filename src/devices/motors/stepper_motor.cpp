#include "PolarFOC/devices/motors/stepper_motor.hpp"


using namespace std;
using namespace status_utils;


void StepperMotor::link_drivers(L298N* phase_A, L298N* phase_B)
{
    m_phase_A = phase_A;
    m_phase_B = phase_B;
     
} // end of "link_drivers(L298N*, L298N*)"


void StepperMotor::link_encoder(AS5047* encoder)
{
    m_encoder = encoder;

} // end of "link_encoder(AS5047*)"


StatusCode StepperMotor::init()
{
    StatusCode init_A = m_phase_A->init();
    StatusCode init_B = m_phase_B->init();

    StatusCode init_encoder = m_encoder->init();
    
    return combine_statuses({
        init_A,
        init_B,
        init_encoder
    });
     
} // end of "init()"


void StepperMotor::set_input_voltage(double input_voltage)
{
    m_phase_A->set_input_voltage(input_voltage);
    m_phase_B->set_input_voltage(input_voltage);
     
} // end of "set_input_voltage(double)"


void StepperMotor::stop()
{
    m_phase_A->stop();
    m_phase_B->stop();

} // end of "stop()"


void StepperMotor::set_voltages(double vA, double vB)
{
    m_phase_A->set_voltage(vA);
    m_phase_B->set_voltage(vB);
 
} // end of "set_voltages(double, double)"


void StepperMotor::set_percents(double A, double B)
{
    m_phase_A->set_percent(A);
    m_phase_B->set_percent(B);
 
} // end of "set_percents(double, double)"


void StepperMotor::step(double voltage, double delay_ms)
{
    // 0 Degrees
    m_phase_A->set_voltage(voltage);
    m_phase_B->set_voltage(0);
    HAL_Delay(delay_ms);

    // 45 Degrees
    m_phase_A->set_voltage(voltage);
    m_phase_B->set_voltage(voltage);
    HAL_Delay(delay_ms);

    // 90 Degrees
    m_phase_A->set_voltage(0);
    m_phase_B->set_voltage(voltage);
    HAL_Delay(delay_ms);

    // 135 Degrees
    m_phase_A->set_voltage(-voltage);
    m_phase_B->set_voltage(voltage);
    HAL_Delay(delay_ms);

    // 180 Degrees
    m_phase_A->set_voltage(-voltage);
    m_phase_B->set_voltage(0);
    HAL_Delay(delay_ms);

    // 235 Degrees
    m_phase_A->set_voltage(-voltage);
    m_phase_B->set_voltage(-voltage);
    HAL_Delay(delay_ms);

    // 270 Degrees
    m_phase_A->set_voltage(0);
    m_phase_B->set_voltage(-voltage);
    HAL_Delay(delay_ms);

    // 315 Degrees
    m_phase_A->set_voltage(voltage);
    m_phase_B->set_voltage(-voltage);
    HAL_Delay(delay_ms);
}