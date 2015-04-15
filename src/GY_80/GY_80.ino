/*****************************************************************/
#define OUTPUT_BAUD_RATE 57600
#define OUTPUT_DATA_INTERVAL 50  // in milliseconds
#define ALT_SEA_LEVEL_PRESSURE 102133

#include <Wire.h>


// RAW sensor data
float accel[3];  
float magnetom[3];
float gyro[3];
float temperature;
float pressure;
float altitude;

void ReadSensors() {
  Read_Pressure();
  Read_Gyro(); // Read gyroscope
  Read_Accel(); // Read accelerometer
  Read_Magn(); // Read magnetometer
}


void setup()
{
  // Init serial output
  Serial.begin(OUTPUT_BAUD_RATE);
  
  // Init sensors
  delay(50);  // Give sensors enough time to start
  I2C_Init();
  Accel_Init();
  Magn_Init();
  Gyro_Init();
  Pressure_Init();  
  delay(30);  // Give sensors enough time to collect data
}

// Main loop
void loop()
{
    ReadSensors();
    
    Serial.print(accel[0]); Serial.print(",");
    Serial.print(accel[1]); Serial.print(",");
    Serial.print(accel[2]); Serial.print(",");

    Serial.print(magnetom[0]); Serial.print(",");
    Serial.print(magnetom[1]); Serial.print(",");
    Serial.print(magnetom[2]); Serial.print(",");
  
    Serial.print(gyro[0]); Serial.print(",");
    Serial.print(gyro[1]); Serial.print(",");
    Serial.print(gyro[2]); Serial.print(",");

    Serial.print(temperature); Serial.print(",");
    Serial.print(pressure); Serial.println();
    
    //delay(OUTPUT_DATA_INTERVAL);
}
