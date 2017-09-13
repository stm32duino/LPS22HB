# LPS22HB
Arduino library to support the LPS22HB 260-1260 hPa absolute digital output barometer

## API

This sensor uses I2C to communicate. It is then required to create a TwoWire interface before accessing to the sensors:  

    dev_i2c = new TwoWire(I2C2_SDA, I2C2_SCL);  
    dev_i2c->begin();  

An instance can be created and enbaled following the procedure below:  

    PressTemp = new LPS22HBSensor(dev_i2c);  
    PressTemp->Enable();  

The access to the sensor values is done as explained below:  

  Read pressure and temperature.  

    PressTemp->GetPressure(&pressure);  
    PressTemp->GetTemperature(&temperature);

## Documentation

You can find the source files at  
https://github.com/stm32duino/LPS22HB

The LPS22HB datasheet is available at  
http://www.st.com/content/st_com/en/products/mems-and-sensors/pressure-sensors/lps22hb.html
