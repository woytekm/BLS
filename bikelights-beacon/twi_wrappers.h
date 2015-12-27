bool readI2cData(uint8_t address, uint8_t reg, uint8_t* data, uint8_t len);
bool writeI2cReg(uint8_t address, uint8_t reg, uint8_t value);
uint8_t readI2cReg(uint8_t address, uint8_t reg);

