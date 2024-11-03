#include <Arduino.h>
#include <Wire.h>

constexpr uint8_t I2C_ADDRESS = 0x22;

bool i2c_scanner()
{
    int nDevices = 0;

    Serial.println("Scanning...");

    for (byte address = 1; address < 127; ++address)
    {
        // The i2c_scanner uses the return value of
        // the Wire.endTransmission to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.print(address, HEX);
            Serial.println("  !");

            ++nDevices;
        }
        else if (error == 4)
        {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
    {
        Serial.println("No I2C devices found\n");
        return false;
    }
    else
    {
        Serial.println("done\n");
        return true;
    }
}

void setup()
{
    Serial.begin(115200);

    Wire.begin();
    Wire.setClock(1000); // slow down i2c clock to 1 kHz
    //Wire.setTimeout(1000);
}

void loop()
{
    const bool found = i2c_scanner();

    if (found)
    {
        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(0xaa); // LED on
        uint8_t rc = Wire.endTransmission();
        Serial.print("rc=");
        Serial.println(rc);

        delay(2500);

        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(0x55); // LED off
        rc = Wire.endTransmission();
        Serial.print("rc=");
        Serial.println(rc);


        rc = Wire.requestFrom(I2C_ADDRESS, static_cast<uint8_t>(4));
        Serial.print("rc=");
        Serial.println(rc);

        while (Wire.available())
        {
            Serial.print("RX: 0x");
            Serial.println(Wire.read(), HEX);
        }
    }

    delay(5000);
}