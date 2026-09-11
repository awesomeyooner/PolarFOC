#include "PolarFOC/devices/motors/stepper_motor.hpp"


using namespace std;
using namespace status_utils;


StepperMotor::StepperMotor(double pole_pairs)
{
    set_pole_pairs(pole_pairs);

} // end of "StepperMotor(double = 50)"


void StepperMotor::set_pole_pairs(double pole_pairs)
{
    m_pole_pairs = pole_pairs;

} // end of "set_pole_pairs(int)"


double StepperMotor::get_pole_pairs()
{
    return m_pole_pairs;

} // end of "get_pole_pairs()"


void StepperMotor::set_input_voltage(double input_voltage)
{
    m_phase_A->set_input_voltage(input_voltage);
    m_phase_B->set_input_voltage(input_voltage);
     
} // end of "set_input_voltage(double)"


double StepperMotor::get_input_voltage()
{
    return m_phase_A->get_input_voltage();

} // end of "get_input_voltage()"


void StepperMotor::link_drivers(DualPWMDriver* phase_A, DualPWMDriver* phase_B)
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


void StepperMotor::stop()
{
    m_phase_A->stop();
    m_phase_B->stop();

} // end of "stop()"


void StepperMotor::refresh()
{
    m_encoder->refresh();

} // end of "refresh()";


void StepperMotor::move()
{
    inverse_park(
        m_target_voltage_d,
        m_target_voltage_q
    );

} // end of "move()"


void StepperMotor::set_target_voltage(double vq)
{
    m_target_voltage_q = vq;

} // end of "set_target_voltage(double)"


void StepperMotor::set_target_voltages(double vd, double vq)
{
    m_target_voltage_d = vd;
    m_target_voltage_q = vq;
}

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


void StepperMotor::inverse_park(double vq)
{
    inverse_park(0, vq);

} // end of "inverse_park(double)"


void StepperMotor::inverse_park(double vd, double vq)
{
    double el_angle = get_electrical_angle();

    Vector2d phase_voltages = Vector2d({vd, vq}).rotate(el_angle);

    double vA = phase_voltages.at(0);
    double vB = phase_voltages.at(1);

    set_voltages(vA, vB);

} // end of "inverse_park(double, double)"


void StepperMotor::inverse_park(double vd, double vq, double mechanical_angle)
{
    double el_angle = mechanical_angle * m_pole_pairs;

    Vector2d phase_voltages = Vector2d({vd, vq}).rotate(el_angle);

    double vA = phase_voltages.at(0);
    double vB = phase_voltages.at(1);

    set_voltages(vA, vB);

} // end of "inverse_park(double, double, double)"


void StepperMotor::inverse_park_openloop(double vq, double rads_per_sec)
{
    inverse_park_openloop(0, vq, rads_per_sec);
}


void StepperMotor::inverse_park_openloop(double vd, double vq, double rads_per_sec)
{
    double now = System::get_seconds(true);
    double dt = now - m_openloop_timestamp;

    m_openloop_angle += rads_per_sec * dt;

    double mechanical_angle = m_openloop_angle;

    inverse_park(vd, vq, mechanical_angle);

    m_openloop_timestamp = now;
}


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

} // end of "step(double, double = 20)"


double StepperMotor::get_electrical_angle()
{
    return m_encoder->get_angle() * m_pole_pairs;

} // end of "get_electrical_angle()"


void StepperMotor::calibrate_angle_offset(double voltage)
{
    m_encoder->set_offset(
        get_angle_offset(voltage)
    );

} // end of "calibrate_angle_offset(double)"


double StepperMotor::get_angle_offset(double voltage)
{
    HAL_Delay(500);

    m_phase_A->set_voltage(voltage);
    m_phase_B->set_voltage(0);

    HAL_Delay(1000);

    m_encoder->refresh();

    HAL_Delay(500);

    stop();

    return m_encoder->get_angle();

} // end of "get_angle_offset(double)"


AS5047* StepperMotor::get_encoder()
{
    return m_encoder;

} // end of "get_encoder()"


DualPWMDriver* StepperMotor::get_phase_A()
{
    return m_phase_A;

} // end of "get_phase_A()"


DualPWMDriver* StepperMotor::get_phase_B()
{
    return m_phase_B;

} // end of "get_phase_B()"