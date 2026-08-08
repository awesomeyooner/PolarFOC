#ifndef STEPPER_MOTOR_HPP
#define STEPPER_MOTOR_HPP


#include "stm32f4xx_hal.h"

#include "PolarFOC/devices/drivers/l298n.hpp"
#include "PolarFOC/devices/encoders/as5047.hpp"


class StepperMotor
{

    public:

        void link_drivers(L298N* phase_A, L298N* phase_B);

        void link_encoder(AS5047* encoder);

        status_utils::StatusCode init();

        void set_input_voltage(double input_voltage);

        void stop();

        void set_voltages(double vA, double vB);

        void set_percents(double A, double B);

        void step(double voltage, double delay_ms = 20);

    private:

        AS5047* m_encoder = nullptr;
        L298N* m_phase_A = nullptr;
        L298N* m_phase_B = nullptr;


}; // class StepperMotor


#endif // STEPPER_MOTOR_HPP