/* Copyright (c) 2013 Microsoft Corporation. All Rights Reserved.
 *
 */

#include <stdbool.h>
#include <stdint.h>

#include "twi1_master.h"
#include "twi1_config.h"

#include "twi2_master.h"
#include "twi2_config.h"

// read n-bytes from i2c register at the given address where subsequent bytes
// are read from incrementally increasing register addresses.
bool readI2cData(uint8_t bus, uint8_t address, uint8_t reg, uint8_t* data, uint8_t len)
{		
  // initialize data to zero so we don't return random values.
  for (int i = 0; i < len; i++) 
   {
     data[i] = 0;
   }

  if(bus == 1)
  {	
  // Write: register address we want to start reading from
  if (twi1_master_transfer(address, &reg, 1, TWI1_DONT_ISSUE_STOP))
    {
		// Read: the number of bytes requested.
     if (twi1_master_transfer(address | TWI1_READ_BIT, data, len, TWI1_ISSUE_STOP))
      {			
        // Read succeeded.
        return true;
      }
     }
   }
  else if(bus == 2)
   {
     // Write: register address we want to start reading from
     if (twi2_master_transfer(address, &reg, 1, TWI2_DONT_ISSUE_STOP))
      {
       // Read: the number of bytes requested.
       if (twi2_master_transfer(address | TWI2_READ_BIT, data, len, TWI2_ISSUE_STOP))
        {
          // Read succeeded.
          return true;
        }
       }
    }
	
  // read or write failed.
  return false;
}

// read the i2c register at the given address (see table 13 in the LSM9DS0 spec)
// first we write the register address to tell the device to prepare that value
// then we read 1 byte in response from the same i2c device.
uint8_t readI2cReg(uint8_t bus, uint8_t address, uint8_t reg)
{
  uint8_t data = 0;
	
  if (readI2cData(bus, address, reg, &data, 1)) 
	{
 	 return data;
	}
  return 0;
}


// write 1 byte to the i2c register at the given address (see table 11 in the LSM9DS0 spec)
bool writeI2cReg(uint8_t bus, uint8_t address, uint8_t reg, uint8_t value)
{
  uint8_t data[2];
  data[0] = reg;
  data[1] = value;

  if(bus == 1)	
   {
     // Write: register protocol
     if (twi1_master_transfer(address, data, 2, TWI1_ISSUE_STOP))
      {
        return true;    
      }
    }
  else if(bus == 2)
   {
    if (twi2_master_transfer(address, data, 2, TWI2_ISSUE_STOP))
     {
        return true;
     }
   }

  // read or write failed.
  return false;
}


/*lint --flb "Leave library region" */
