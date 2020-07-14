#include "this_hardware.h"

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object.
LSM9DS1 imu[2];
// Connection status values for blocking reading when no device is present:
int imu_device0_connected = 0;
int imu_device1_connected = 0;

/* init_hardware
 *
 * initalises the hardware componenets of the microcontroller.
 * Pin direction (input/output/pullup)
 * Set starting state (default safe)
 * Starts any communication hardware (WIRE, SPI, ETC)
 * Starts any device drivers as needed.
 * Initalises timers/interrupts as needed.
 * 
 */
void init_hardware(void){
  // LED output for effect:
  pinMode(13,OUTPUT);
  
  // Start the I2C module (default pins and mode)
  Wire.begin();

  // connect to accelerometer device 0 via I2C
  if (imu[0].begin(IMU_DEVICE_0_AG_ADDR,IMU_DEVICE_0_M_ADDR,Wire) != 0) {
    imu_device0_connected = 1;
  }
  // connect to accelerometer device 1 via I2C
  if (imu[1].begin(IMU_DEVICE_1_AG_ADDR,IMU_DEVICE_1_M_ADDR,Wire) != 0) {
    imu_device1_connected = 1;
  }
}

/* _is_imu_connected
 *
 * determines if an accelerometer device is connected and initalised.
 * returns a boolean value: true only if connected, else false in all other cases (including invalid device ID)
 */
bool _is_imu_connected(int device_id){
  if (device_id == 0){
    return (bool)imu_device0_connected;
  }
  else if (device_id == 1){
    return (bool)imu_device1_connected;
  }
  else {
    return false;
  }
}

/* get_gyro_x
 * 
 * Returns the calculated x-axis value of the gyroscope from the selected device.
 * Return error code -89 if an incorrect device ID or deactivated device is selected.
 */
float get_gyro_x(int device_id){
  if (_is_imu_connected(device_id)) {
    return imu[device_id].calcGyro(imu[device_id].readGyro(X_AXIS));
  }
  return -89; //return a bogus value so I know where the error is.
}

/* get_gyro_y
 * 
 * Returns the calculated y-axis value of the gyroscope from the selected device.
 * Return error code -89 if an incorrect device ID or deactivated device is selected.
 */
float get_gyro_y(int device_id){
  if (_is_imu_connected(device_id)) {
    return imu[device_id].calcGyro(imu[device_id].readGyro(Y_AXIS));
  }
  return -89; //return a bogus value so I know where the error is.
}

/* get_gyro_z
 * 
 * Returns the calculated z-axis value of the gyroscope from the selected device.
 * Return error code -89 if an incorrect device ID or deactivated device is selected.
 */
float get_gyro_z(int device_id){
  if (_is_imu_connected(device_id)) {
    return imu[device_id].calcGyro(imu[device_id].readGyro(Z_AXIS));
  }
  return -89; //return a bogus value so I know where the error is.
}

/* get_accel_x
 * 
 * Returns the calculated x-axis value of the accelerometer from the selected device.
 * Return error code -89 if an incorrect device ID or deactivated device is selected.
 */
float get_accel_x(int device_id){
  if (_is_imu_connected(device_id)) {
    return imu[device_id].calcAccel(imu[device_id].readAccel(X_AXIS));
  }
  return -89; //return a bogus value so I know where the error is.
}

/* get_accel_y
 * 
 * Returns the calculated y-axis value of the accelerometer from the selected device.
 * Return error code -89 if an incorrect device ID or deactivated device is selected.
 */
float get_accel_y(int device_id){
  if (_is_imu_connected(device_id)) {
    return imu[device_id].calcAccel(imu[device_id].readAccel(Y_AXIS));
  }
  return -89; //return a bogus value so I know where the error is.
}

/* get_accel_z
 * 
 * Returns the calculated z-axis value of the accelerometer from the selected device.
 * Return error code -89 if an incorrect device ID or deactivated device is selected.
 */
float get_accel_z(int device_id){
  if (_is_imu_connected(device_id)) {
    return imu[device_id].calcAccel(imu[device_id].readAccel(Z_AXIS));
  }
  return -89; //return a bogus value so I know where the error is.
}

/* get_mag_x
 * 
 * Returns the calculated x-axis value of the magnetometer from the selected device.
 * Return error code -89 if an incorrect device ID or deactivated device is selected.
 */
float get_mag_x(int device_id){
  if (_is_imu_connected(device_id)) {
    return imu[device_id].calcMag(imu[device_id].readMag(X_AXIS));
  }
  return -89; //return a bogus value so I know where the error is.
}

/* get_mag_y
 * 
 * Returns the calculated y-axis value of the magnetometer from the selected device.
 * Return error code -89 if an incorrect device ID or deactivated device is selected.
 */
float get_mag_y(int device_id){
  if (_is_imu_connected(device_id)) {
    return imu[device_id].calcMag(imu[device_id].readMag(Y_AXIS));
  }
  return -89; //return a bogus value so I know where the error is.
}

/* get_mag_y
 * 
 * Returns the calculated y-axis value of the magnetometer from the selected device.
 * Return error code -89 if an incorrect device ID or deactivated device is selected.
 */
float get_mag_z(int device_id){
  if (_is_imu_connected(device_id)) {
    return imu[device_id].calcMag(imu[device_id].readMag(Z_AXIS));
  }
  return -89; //return a bogus value so I know where the error is.
}

void run_stuff(void) {
   

}

void printGyro(int i)
{
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  Serial.print(imu[i].calcGyro(imu[i].gx), 2);
  Serial.print(", ");
  Serial.print(imu[i].calcGyro(imu[i].gy), 2);
  Serial.print(", ");
  Serial.print(imu[i].calcGyro(imu[i].gz), 2);
  Serial.println(" deg/s");
#elif defined PRINT_RAW
  Serial.print(imu[i].gx);
  Serial.print(", ");
  Serial.print(imu[i].gy);
  Serial.print(", ");
  Serial.println(imu[i].gz);
#endif
}

void printAccel(int i)
{
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(imu[i].calcAccel(imu[i].ax), 2);
  Serial.print(", ");
  Serial.print(imu[i].calcAccel(imu[i].ay), 2);
  Serial.print(", ");
  Serial.print(imu[i].calcAccel(imu[i].az), 2);
  Serial.println(" g");
#elif defined PRINT_RAW
  Serial.print(imu[i].ax);
  Serial.print(", ");
  Serial.print(imu[i].ay);
  Serial.print(", ");
  Serial.println(imu[i].az);
#endif

}

void printMag(int i)
{
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  Serial.print("M: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  Serial.print(imu[i].calcMag(imu[i].mx), 2);
  Serial.print(", ");
  Serial.print(imu[i].calcMag(imu[i].my), 2);
  Serial.print(", ");
  Serial.print(imu[i].calcMag(imu[i].mz), 2);
  Serial.println(" gauss");
#elif defined PRINT_RAW
  Serial.print(imu[i].mx);
  Serial.print(", ");
  Serial.print(imu[i].my);
  Serial.print(", ");
  Serial.println(imu[i].mz);
#endif
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  float heading;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);

  heading -= DECLINATION * PI / 180;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);

  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;

  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("Heading: "); Serial.println(heading, 2);
}

void read_IMU_example(int device_id) {
    if ( imu[device_id].gyroAvailable() ) {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu[device_id].readGyro();
  }
  if ( imu[device_id].accelAvailable() ) {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu[device_id].readAccel();
  }
  if ( imu[device_id].magAvailable() ) {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu[device_id].readMag();
  }
  printGyro(device_id);  // Print "G: gx, gy, gz"
  printAccel(device_id); // Print "A: ax, ay, az"
  printMag(device_id);   // Print "M: mx, my, mz"
  // Print the heading and orientation for fun!
  // Call print attitude. The LSM9DS1's mag x and y
  // axes are opposite to the accelerometer, so my, mx are
  // substituted for each other.
  printAttitude(imu[device_id].ax, imu[device_id].ay, imu[device_id].az,
                -imu[device_id].my, -imu[device_id].mx, imu[device_id].mz);
  Serial.println();
}