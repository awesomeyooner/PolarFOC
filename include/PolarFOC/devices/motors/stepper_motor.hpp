#ifndef STEPPER_MOTOR_HPP
#define STEPPER_MOTOR_HPP


#include "stm32f4xx_hal.h"

#include "PolarFOC/devices/drivers/l298n.hpp"
#include "PolarFOC/devices/encoders/as5047.hpp"

#include "EmbeddedLib/math/vector2d.hpp"


class StepperMotor
{

    public:

        StepperMotor(double pole_pairs = 50);

        void set_pole_pairs(double pole_pairs);

        double get_pole_pairs();

        void set_input_voltage(double input_voltage);

        double get_input_voltage();

        void link_drivers(L298N* phase_A, L298N* phase_B);

        void link_encoder(AS5047* encoder);

        status_utils::StatusCode init();

        void stop();

        void refresh();

        void move();

        void set_target_voltage(double vq);

        void set_target_voltages(double vd, double vq);

        void set_voltages(double vA, double vB);

        void set_percents(double A, double B);

        void inverse_park(double vq);

        void inverse_park(double vd, double vq);

        void inverse_park(double vd, double vq, double mechanical_angle);

        void inverse_park_openloop(double vq, double rads_per_sec);

        void inverse_park_openloop(double vd, double vq, double rads_per_sec);

        void step(double voltage, double delay_ms = 20);

        double get_electrical_angle();

        void calibrate_angle_offset(double voltage);

        double get_angle_offset(double voltage);

        AS5047* get_encoder();

        L298N* get_phase_A();

        L298N* get_phase_B();

        double m_openloop_angle = 0;

    private:

        AS5047* m_encoder = nullptr;
        L298N* m_phase_A = nullptr;
        L298N* m_phase_B = nullptr;

        // The number of pole pairs the stepper has. Typical 1.8 deg steppers have 50
        double m_pole_pairs = 50;

        /* 
        The target D-Axis voltage to apply during `inverse_park`. This is aligned with the rotor's magnetic flux,
        used for field weakening. This is typically set to 0 to have the best torque performance
        */        
        int m_target_voltage_d = 0;

        /*
        The target Q-Axis voltage to apply during `inverse_park`. This is orthogonal to the rotor's magnetic field,
        used for controlling torque
        */
        int m_target_voltage_q = 0;

        // double m_openloop_angle = 0;
        double m_openloop_timestamp = 0;


}; // class StepperMotor


#endif // STEPPER_MOTOR_HPP