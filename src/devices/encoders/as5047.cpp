#include "devices/as5047.hpp"


#define BIT_MODITY(src, i, val) ((src) ^= (-(val) ^ (src)) & (1UL << (i)))
#define BIT_READ(src, i) (((src) >> (i)&1U))
#define BIT_TOGGLE(src, i) ((src) ^= 1UL << (i))


AS5047::AS5047(SPI_HandleTypeDef* spi, GPIO_TypeDef* GPIO_family, int pin_num, int timeout)
{
    m_spi = spi;
    m_cs_family = GPIO_family;
    m_cs_pin = pin_num;
    m_timeout = timeout;

} // end of "AS5047"


void AS5047::init()
{

} // end of "init"


int AS5047::get_uncompensated_counts()
{
    uint16_t data = read_data(UNCOMPENSATED_ANGLE_REGISTER);

    return data & RESULT_MASK;

} // end of "get_uncompensated_counts"


int AS5047::get_compensated_counts()
{
    uint16_t data = read_data(COMPENSATED_ANGLE_REGISTER);

    return data & RESULT_MASK;

} // end of "get_compensated_counts"


int AS5047::get_raw_counts(bool compensated)
{
    return compensated ? get_compensated_counts() : get_uncompensated_counts();

} // end of "get_raw_counts"


double AS5047::get_raw_rotations(bool compensated)
{
    double counts = compensated ? get_compensated_counts() : get_uncompensated_counts();

    return counts / CPR;

} // end of "get_raw_rotations"


double AS5047::get_raw_angle(bool compensated)
{
    // Get the raw counts
    double counts = compensated ? get_compensated_counts() : get_uncompensated_counts();

    // Rotations = counts / CPR
    // Radians = Rotations * 2PI
    // Return Radians
    return (counts / CPR) * 2 * M_PI;

} // end of "get_raw_angle"


double AS5047::get_angle(bool compensated)
{
    // TODO: Add angle wrapping so it's not constrained from [0, 2Pi]

    double raw_angle = get_raw_angle(compensated);

    return raw_angle;

} // end of "get_angle"

int AS5047::get_magnetic_magnitude()
{
    uint16_t data = read_data(MAGNETIC_MAGNITUDE_REGISTER);

    return data & RESULT_MASK;

} // end of "get_magnetic_magnitude"


void AS5047::NOP()
{
    send_command(NOP_REGISTER, READ);

} // end of "NOP"


void AS5047::send_command(uint16_t address, uint8_t read_or_write)
{
    uint16_t frame = address & RESULT_MASK;

    BIT_MODITY(frame, 14, read_or_write);

    if(!is_parity_even(frame))
        BIT_TOGGLE(frame, 15);

    transmit(frame);

} // end of "send_command(uint16_t, uint_t)"


uint16_t AS5047::read_data(uint16_t address)
{
    send_command(address, READ);

    return recieve();

} // end of read_data(uint16_t)


void AS5047::transmit(uint16_t data)
{
    uint8_t bytes[2];
    uint8_t* pointer = bytes;

    memcpy(bytes, &data, sizeof(data));

    // Select CS
    select();

    // Send Data
    // Size is 1 since data size was defined as 16 bit
    HAL_SPI_Transmit(m_spi, pointer, 1, m_timeout);

    // Deselect CS
    deselect();

} // end of "transmit"


uint16_t AS5047::recieve()
{
    // Store the data in this varaible
    uint16_t data;

    uint8_t bytes[2];
    uint8_t* pointer = bytes;

    // Select CS
    select();

    // Recieve the data
    HAL_SPI_Receive(m_spi, (uint8_t*)pointer, 1, m_timeout);

    memcpy(&data, bytes, sizeof(data));

    // Deselect CS
    deselect();

    return data;

} // end of "recieve()"


void AS5047::select()
{
    // Toggle pin LOW to activate CS
    HAL_GPIO_WritePin(m_cs_family, m_cs_pin, GPIO_PIN_RESET);

} // end of "select()"


void AS5047::deselect()
{
    // Toggle pin HIGH to activate CS
    HAL_GPIO_WritePin(m_cs_family, m_cs_pin, GPIO_PIN_SET);

} // end of "deselect()"


bool AS5047::is_parity_even(uint16_t data)
{
    data ^= data >> 8; // XOR upper 8 bits with lower 8 bits
    data ^= data >> 4; // XOR 4 bits with the next 4 bits
    data ^= data >> 2; // XOR 2 bits with the next 2 bits
    data ^= data >> 1; // XOR adjacent bits

    // The result (1 for even parity, 0 for odd parity) is in the least-significant bit
    return !(data & 1);

} // end of "is_parity_even(uint16_t)"