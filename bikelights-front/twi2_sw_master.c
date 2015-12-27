/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "twi2_master.h"
#include "nrf_delay.h"

//#include "twi2_master_config.h"
#include "twi2_config.h"

/*lint -e415 -e845 -save "Out of bounds access" */
#define TWI2_SDA_STANDARD0_NODRIVE1() do { \
        NRF_GPIO->PIN_CNF[TWI2_MASTER_CONFIG_DATA_PIN_NUMBER] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) \
        |(GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos)    \
        |(GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)  \
        |(GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) \
        |(GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);  \
} while (0) /*!< Configures SDA pin to Standard-0, No-drive 1 */


#define TWI2_SCL_STANDARD0_NODRIVE1() do { \
        NRF_GPIO->PIN_CNF[TWI2_MASTER_CONFIG_CLOCK_PIN_NUMBER] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) \
        |(GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos)    \
        |(GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)  \
        |(GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) \
        |(GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);  \
} while (0) /*!< Configures SCL pin to Standard-0, No-drive 1 */


/*lint -restore */

#ifndef TWI2_MASTER_TIMEOUT_COUNTER_LOAD_VALUE
#define TWI2_MASTER_TIMEOUT_COUNTER_LOAD_VALUE (0UL) //!< Unit is number of empty loops. Timeout for SMBus devices is 35 ms. Set to zero to disable slave timeout altogether.
#endif

static bool twi2_master_clear_bus(void);
static bool twi2_master_issue_startcondition(void);
static bool twi2_master_issue_stopcondition(void);
static bool twi2_master_clock_byte(uint_fast8_t databyte);
static bool twi2_master_clock_byte_in(uint8_t * databyte, bool ack);
static bool twi2_master_wait_while_scl_low(void);

bool twi2_master_init(void)
{
    // Configure both pins to output Standard 0, No-drive (open-drain) 1
    TWI2_SDA_STANDARD0_NODRIVE1(); /*lint !e416 "Creation of out of bounds pointer" */
    TWI2_SCL_STANDARD0_NODRIVE1(); /*lint !e416 "Creation of out of bounds pointer" */

    // Configure SCL as output
    TWI2_SCL_HIGH();
    TWI2_SCL_OUTPUT();

    // Configure SDA as output
    TWI2_SDA_HIGH();
    TWI2_SDA_OUTPUT();

    return twi2_master_clear_bus();
}

bool twi2_master_transfer(uint8_t address, uint8_t * data, uint8_t data_length, bool issue_stop_condition)
{
    bool transfer_succeeded = true;

    transfer_succeeded &= twi2_master_issue_startcondition();
    transfer_succeeded &= twi2_master_clock_byte(address);

    if (address & TWI2_READ_BIT)
    {
        /* Transfer direction is from Slave to Master */
        while (data_length-- && transfer_succeeded)
        {
            // To indicate to slave that we've finished transferring last data byte
            // we need to NACK the last transfer.
            if (data_length == 0)
            {
                transfer_succeeded &= twi2_master_clock_byte_in(data, (bool)false);
            }
            else
            {
                transfer_succeeded &= twi2_master_clock_byte_in(data, (bool)true);
            }
            data++;
        }
    }
    else
    {
        /* Transfer direction is from Master to Slave */
        while (data_length-- && transfer_succeeded)
        {
            transfer_succeeded &= twi2_master_clock_byte(*data);
            data++;
        }
    }

    if (issue_stop_condition || !transfer_succeeded)
    {
        transfer_succeeded &= twi2_master_issue_stopcondition();
    }

    return transfer_succeeded;
}

/**
 * @brief Function for detecting stuck slaves and tries to clear the bus.
 *
 * @return
 * @retval false Bus is stuck.
 * @retval true Bus is clear.
 */
static bool twi2_master_clear_bus(void)
{
    bool bus_clear;

    TWI2_SDA_HIGH();
    TWI2_SCL_HIGH();
    TWI2_DELAY();


    if (TWI2_SDA_READ() == 1 && TWI2_SCL_READ() == 1)
    {
        bus_clear = true;
    }
    else if (TWI2_SCL_READ() == 1)
    {
        bus_clear = false;
        // Clock max 18 pulses worst case scenario(9 for master to send the rest of command and 9 for slave to respond) to SCL line and wait for SDA come high
        for (uint_fast8_t i = 18; i--;)
        {
            TWI2_SCL_LOW();
            TWI2_DELAY();
            TWI2_SCL_HIGH();
            TWI2_DELAY();

            if (TWI2_SDA_READ() == 1)
            {
                bus_clear = true;
                break;
            }
        }
    }
    else
    {
        bus_clear = false;
    }

    return bus_clear;
}

/**
 * @brief Function for issuing TWI START condition to the bus.
 *
 * START condition is signaled by pulling SDA low while SCL is high. After this function SCL and SDA will be low.
 *
 * @return
 * @retval false Timeout detected
 * @retval true Clocking succeeded
 */
static bool twi2_master_issue_startcondition(void)
{
#if 0
    if (TWI2_SCL_READ() == 1 && TWI2_SDA_READ() == 1)
    {
        // Pull SDA low
        TWI2_SDA_LOW();
    }
    else if (TWI2_SCL_READ() == 1 && TWI2_SDA_READ() == 0)
    {
        // Issue Stop by pulling SDA high
        TWI2_SDA_HIGH();
        TWI2_DELAY();

        // Then Start by pulling SDA low
        TWI2_SDA_LOW();
    }
    else if (TWI2_SCL_READ() == 0 && TWI2_SDA_READ() == 0)
    {
        // First pull SDA high
        TWI2_SDA_HIGH();

        // Then SCL high
        if (!twi2_master_wait_while_scl_low())
        {
            return false;
        }

        // Then SDA low
        TWI2_SDA_LOW();
    }
    else if (TWI2_SCL_READ() == 0 && TWI2_SDA_READ() == 1)
    {
        // SCL high
        if (!twi2_master_wait_while_scl_low())
        {
            return false;
        }

        // Then SDA low
        TWI2_SDA_LOW();
    }

    TWI2_DELAY();
    TWI2_SCL_LOW();
#endif

    // Make sure both SDA and SCL are high before pulling SDA low.
    TWI2_SDA_HIGH();
    TWI2_DELAY();
    if (!twi2_master_wait_while_scl_low())
    {
        return false;
    }

    TWI2_SDA_LOW();
    TWI2_DELAY();

    // Other module function expect SCL to be low
    TWI2_SCL_LOW();
    TWI2_DELAY();

    return true;
}

/**
 * @brief Function for issuing TWI STOP condition to the bus.
 *
 * STOP condition is signaled by pulling SDA high while SCL is high. After this function SDA and SCL will be high.
 *
 * @return
 * @retval false Timeout detected
 * @retval true Clocking succeeded
 */
static bool twi2_master_issue_stopcondition(void)
{
#if 0
    if (TWI2_SCL_READ() == 1 && TWI2_SDA_READ() == 1)
    {
        // Issue start, then issue stop

        // Pull SDA low to issue START
        TWI2_SDA_LOW();
        TWI2_DELAY();

        // Pull SDA high while SCL is high to issue STOP
        TWI2_SDA_HIGH();
    }
    else if (TWI2_SCL_READ() == 1 && TWI2_SDA_READ() == 0)
    {
        // Pull SDA high while SCL is high to issue STOP
        TWI2_SDA_HIGH();
    }
    else if (TWI2_SCL_READ() == 0 && TWI2_SDA_READ() == 0)
    {
        if (!twi2_master_wait_while_scl_low())
        {
            return false;
        }

        // Pull SDA high while SCL is high to issue STOP
        TWI2_SDA_HIGH();
    }
    else if (TWI2_SCL_READ() == 0 && TWI2_SDA_READ() == 1)
    {
        TWI2_SDA_LOW();
        TWI2_DELAY();

        // SCL high
        if (!twi2_master_wait_while_scl_low())
        {
            return false;
        }

        // Pull SDA high while SCL is high to issue STOP
        TWI2_SDA_HIGH();
    }

    TWI2_DELAY();
#endif

    TWI2_SDA_LOW();
    TWI2_DELAY();
    if (!twi2_master_wait_while_scl_low())
    {
        return false;
    }

    TWI2_SDA_HIGH();
    TWI2_DELAY();

    return true;
}

/**
 * @brief Function for clocking one data byte out and reads slave acknowledgment.
 *
 * Can handle clock stretching.
 * After calling this function SCL is low and SDA low/high depending on the
 * value of LSB of the data byte.
 * SCL is expected to be output and low when entering this function.
 *
 * @param databyte Data byte to clock out.
 * @return
 * @retval true Slave acknowledged byte.
 * @retval false Timeout or slave didn't acknowledge byte.
 */
static bool twi2_master_clock_byte(uint_fast8_t databyte)
{
    bool transfer_succeeded = true;

    /** @snippet [TWI SW master write] */
    // Make sure SDA is an output
    TWI2_SDA_OUTPUT();

    // MSB first
    for (uint_fast8_t i = 0x80; i != 0; i >>= 1)
    {
        TWI2_SCL_LOW();
        TWI2_DELAY();

        if (databyte & i)
        {
            TWI2_SDA_HIGH();
        }
        else
        {
            TWI2_SDA_LOW();
        }

        if (!twi2_master_wait_while_scl_low())
        {
            transfer_succeeded = false; // Timeout
            break;
        }
    }

    // Finish last data bit by pulling SCL low
    TWI2_SCL_LOW();
    TWI2_DELAY();

    /** @snippet [TWI SW master write] */

    // Configure TWI2_SDA pin as input for receiving the ACK bit
    TWI2_SDA_INPUT();

    // Give some time for the slave to load the ACK bit on the line
    TWI2_DELAY();

    // Pull SCL high and wait a moment for SDA line to settle
    // Make sure slave is not stretching the clock
    transfer_succeeded &= twi2_master_wait_while_scl_low();

    // Read ACK/NACK. NACK == 1, ACK == 0
    transfer_succeeded &= !(TWI2_SDA_READ());

    // Finish ACK/NACK bit clock cycle and give slave a moment to release control
    // of the SDA line
    TWI2_SCL_LOW();
    TWI2_DELAY();

    // Configure TWI2_SDA pin as output as other module functions expect that
    TWI2_SDA_OUTPUT();

    return transfer_succeeded;
}


/**
 * @brief Function for clocking one data byte in and sends ACK/NACK bit.
 *
 * Can handle clock stretching.
 * SCL is expected to be output and low when entering this function.
 * After calling this function, SCL is high and SDA low/high depending if ACK/NACK was sent.
 *
 * @param databyte Data byte to clock out.
 * @param ack If true, send ACK. Otherwise send NACK.
 * @return
 * @retval true Byte read succesfully
 * @retval false Timeout detected
 */
static bool twi2_master_clock_byte_in(uint8_t *databyte, bool ack)
{
    uint_fast8_t byte_read          = 0;
    bool         transfer_succeeded = true;

    /** @snippet [TWI SW master read] */
    // Make sure SDA is an input
    TWI2_SDA_INPUT();

    // SCL state is guaranteed to be high here

    // MSB first
    for (uint_fast8_t i = 0x80; i != 0; i >>= 1)
    {
        if (!twi2_master_wait_while_scl_low())
        {
            transfer_succeeded = false;
            break;
        }

        if (TWI2_SDA_READ())
        {
            byte_read |= i;
        }
        else
        {
            // No need to do anything
        }

        TWI2_SCL_LOW();
        TWI2_DELAY();
    }

    // Make sure SDA is an output before we exit the function
    TWI2_SDA_OUTPUT();
    /** @snippet [TWI SW master read] */

    *databyte = (uint8_t)byte_read;

    // Send ACK bit

    // SDA high == NACK, SDA low == ACK
    if (ack)
    {
        TWI2_SDA_LOW();
    }
    else
    {
        TWI2_SDA_HIGH();
    }

    // Let SDA line settle for a moment
    TWI2_DELAY();

    // Drive SCL high to start ACK/NACK bit transfer
    // Wait until SCL is high, or timeout occurs
    if (!twi2_master_wait_while_scl_low())
    {
        transfer_succeeded = false; // Timeout
    }

    // Finish ACK/NACK bit clock cycle and give slave a moment to react
    TWI2_SCL_LOW();
    TWI2_DELAY();

    return transfer_succeeded;
}


/**
 * @brief Function for pulling SCL high and waits until it is high or timeout occurs.
 *
 * SCL is expected to be output before entering this function.
 * @note If TWI2_MASTER_TIMEOUT_COUNTER_LOAD_VALUE is set to zero, timeout functionality is not compiled in.
 * @return
 * @retval true SCL is now high.
 * @retval false Timeout occurred and SCL is still low.
 */
static bool twi2_master_wait_while_scl_low(void)
{
#if TWI2_MASTER_TIMEOUT_COUNTER_LOAD_VALUE != 0
    uint32_t volatile timeout_counter = TWI2_MASTER_TIMEOUT_COUNTER_LOAD_VALUE;
#endif

    // Pull SCL high just in case if something left it low
    TWI2_SCL_HIGH();
    TWI2_DELAY();

    while (TWI2_SCL_READ() == 0)
    {
        // If SCL is low, one of the slaves is busy and we must wait

#if TWI2_MASTER_TIMEOUT_COUNTER_LOAD_VALUE != 0
        if (timeout_counter-- == 0)
        {
            // If timeout_detected, return false
            return false;
        }
#endif
    }

    return true;
}

/*lint --flb "Leave library region" */
