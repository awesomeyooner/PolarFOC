// https://look.ams-osram.com/m/d05ee39221f9857/original/AS5047P-DS000324.pdf

#ifndef AS5047_HPP
#define AS5047_HPP

#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

#include <vector>
#include <cmath>
#include <stdint.h>
#include <stm32f446xx.h>


// #define CPR 16384 // 14 bits
// #define UNCOMPENSATED_ANGLE_REGISTER 0x3FFE
// #define COMPENSATED_ANGLE_REGISTER 0x3FFF
// #define MAGNETIC_MAGNITUDE_REGISTER 0x3FFD
// #define PARITY 0x8000
// #define READ_WRITE 0x4000
// #define RESULT_MASK 0x3FFF


class AS5047
{
    
    public:

        /**
         * @brief Creates a new AS5047 using SPI
         * 
         * @param spi `SPI_HandleTypeDef*` Which SPI to use, ie `SPI1 = &hspi1`
         * @param GPIO_family `GPIO_TypeDef*` The GPIO family of the CS pin. ie `PA11 = GPIOA` 
         * @param pin_num `int` The GPIO pin number for the CS pin. ie `PA11 = GPIO_PIN_11`
         * @param timeout `int = 100` The SPI transaction timeout in milliseconds. Default is `100ms`
         */
        AS5047(SPI_HandleTypeDef* spi, GPIO_TypeDef* GPIO_family, int pin_num, int timeout = 100);

        /**
         * @brief Initializes the SPI interface with the correct settings (`Mode SPI1`).
         * This function is not yet implemented, so no need to call it. Just make sure you
         * correctly setup SPI to be SPI1 mode.
         * 
         * The max baudrate is 10 MHz, or 10 MBits/s
         * 
         * `Frame Format = Motorola`
         * 
         * `Data Size = 16 Bits`
         * 
         * `First Bit = MSB First`
         * 
         * `Clock Parity (CPOL) = Low`
         * 
         * `Clock Phase (CPHA) = 2 Edge`
         * 
         * `CRC Calculation = Disabled`
         * 
         * `NSS Signal Type` is up to the user
         * 
         */
        void init();

        /**
         * @brief Gets the encoder counts [ 0, 16384 ) in compensated mode 
         * 
         * @return `int` 
         */
        int get_compensated_counts();
        
        /**
         * @brief Gets the encoder counts [ 0, 16384 ) in uncompensated mode 
         * 
         * @return `int` 
         */
        int get_uncompensated_counts();

        /**
         * @brief Gets the encoder counts [0, 16384) in compensated or uncompensated mode
         * depending on the boolean passed in.
         * 
         * @param compensated `bool = false` Set to `true` to return compensated values
         * @return `int` 
         */
        int get_raw_counts(bool compensated = false);

        /**
         * @brief Gets the encoder rotation [0, 1) in compensated or uncompensated mode
         * depending on the boolean passed in.
         * 
         * @param compensated `bool = false` Set to `true` to return compensated values
         * @return `double` 
         */
        double get_raw_rotations(bool compensated = false);

        /**
         * @brief Gets the encoder rotation in radians [0, 2Pi) in compensated or uncompensated mode
         * depending on the boolean passed in.
         * 
         * @param compensated `bool = false` Set to `true` to return compensated values
         * @return `double` 
         */
        double get_raw_angle(bool compensated = false);

        /**
         * @brief Gets the unbounded encoder rotations in radians in compensated or uncompensated mode
         * depending on the boolean passed in.
         * 
         * @param compensated `bool = false` Set to `true` to return compensated values
         * @return `double` 
         */
        double get_angle(bool compensated = false);

        /**
         * @brief Gets the strength of the magnet in arbitrary units, higher is better.
         * Values from 4000 to 10000 are good.
         * 
         * @return `int` 
         */
        int get_magnetic_magnitude();

        /**
         * @brief Transmit a No-Operation command
         * 
         */
        void NOP();

        /**
         * @brief Send a command to the sensor using the protocol in the datasheet
         * 
         * @param address `uint16_t` The address / register to send the data to
         * @param read_or_write `uint8_t` The data to transmit
         */
        void send_command(uint16_t address, uint8_t read_or_write);

        /**
         * @brief Read data from the sensor using the protocol in the datasheet
         * 
         * @param address `uint16_t` The address / register to send the data to
         * @return `uint16_t` 
         */
        uint16_t read_data(uint16_t address);

        /**
         * @brief Transmit a 16 bit number to the sensor via SPI
         * 
         * @param data `uint16_t`
         */
        void transmit(uint16_t data);

        /**
         * @brief Recieve a 16 bit number from the sensor via SPI
         * 
         * @return `uint16_t` 
         */
        uint16_t recieve();

    private:

        // Counts per revolution
        static constexpr int CPR = 16384; // 14 bits

        static constexpr uint16_t UNCOMPENSATED_ANGLE_REGISTER = 0x3FFE;
        static constexpr uint16_t COMPENSATED_ANGLE_REGISTER = 0x3FFF;
        static constexpr uint16_t MAGNETIC_MAGNITUDE_REGISTER = 0x3FFD;

        static constexpr uint8_t WRITE = (uint8_t)0;
        static constexpr uint8_t READ = (uint8_t)1;

        static constexpr uint16_t RESULT_MASK = 0x3FFF;
        static constexpr uint16_t NOP_REGISTER = 0x0000;

        // The SPI handle to use
        SPI_HandleTypeDef* m_spi = nullptr;

        // The CS pin family
        GPIO_TypeDef* m_cs_family = nullptr;
        
        // The CS pin number
        int m_cs_pin;

        // SPI transaction timeout in milliseconds
        int m_timeout = 100; //ms

        /**
         * @brief Select the sensor for SPI transactions. Pulls CS Low
         * 
         */
        void select();

        /**
         * @brief Deselect the sensor after an SPI transaction. Pulls CS High
         * 
         */
        void deselect();

        /**
         * @brief Gets if the given 16 bit number has even parity
         * 
         * @param data `uint16_t`
         * @return `bool` 
         */
        bool is_parity_even(uint16_t data);


}; // class AS5047


#endif // AS5047_HPP