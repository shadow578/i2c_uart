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
    Wire.setClock(50000); // 100kHz I2C clock
}

void test_readback(uint8_t *bytes, uint8_t len)
{
    delay(10);

    if (len > 4)
    {
        len = 4;
    }

    Serial.println("\n -- test_readback --");

    // send len bytes to the I2C target
    Wire.beginTransmission(I2C_ADDRESS);
    for (size_t i = 0; i < len; ++i)
    {
        Wire.write(bytes[i]);
    }
    uint8_t rc = Wire.endTransmission();
    Serial.print("TX rc=");
    Serial.println(rc);

    delay(10);

    // read back len bytes from the I2C target
    rc = Wire.requestFrom(I2C_ADDRESS, len);
    Serial.print("RX rc=");
    Serial.println(rc);

    // print out expected and received bytes
    Serial.print("Expected: ");
    for (size_t i = 0; i < len; ++i)
    {
        Serial.print(bytes[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    Serial.print("Received: ");
    bool mismatch = false;
    uint8_t i = 0;
    while (Wire.available())
    {
        uint8_t b = Wire.read();
        Serial.print(b, HEX);
        Serial.print(" ");

        if (i < len && b != bytes[i])
        {
            mismatch = true;
        }

        i++;
    }
    Serial.println();

    if (mismatch)
    {
        Serial.println("Mismatch!");
    }
    else
    {
        Serial.println("Match!");
    }

    Serial.println("\n --  --");
}

void randomize_bytes(uint8_t *bytes, uint8_t len)
{
    for (uint8_t i = 0; i < len; ++i)
    {
        bytes[i] = random(0, 256);
    }
}

void loop()
{
    const bool found = i2c_scanner();

    if (found)
    {
        uint8_t bytes[4];
        randomize_bytes(bytes, 4);
        test_readback(bytes, 1);

        randomize_bytes(bytes, 4);
        test_readback(bytes, 2);

        randomize_bytes(bytes, 4);
        test_readback(bytes, 4);
    }

    delay(5000);
}