
#include <Arduino.h>
#include <MPU6500_WE.h>
#include <math.h>

#include <Wire.h>

#define MPU6500_ADDR 0x68

// State Machine of Flight controller
#define DISARMED 0
#define ARMED 1
#define TAKEOFF 2
#define FLIGHT 3
#define LANDING 4
#define EMERGENCY 5
#define CALLIBRATION 6

// For receiver
#define PPM_PIN 2         // Must be interrupt pin (D2 or D3)
#define CHANNELS 8        // Number of channels you expect
#define PPM_SYNC_GAP 3000 // µs (sync pulse gap)

// ESC signal pins
#define ESC1_PIN 3
#define ESC2_PIN 9
#define ESC3_PIN 10
#define ESC4_PIN 11

// ESC pulse values (microseconds)
#define ESC_MIN 1000 // minimum throttle
#define ESC_MAX 2000 // maximum throttle

MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);

// All variables
xyzFloat gValue;
xyzFloat gyr;

int state = DISARMED;

const uint8_t gyro_address = 0x68; // Default I2C address

float acc_total_vector;
int temperature;
float acc_axis[3], gyro_axis[3];

// Processed values
int16_t acc_x, acc_y, acc_z;

float gyro_roll, gyro_pitch, gyro_yaw;
float gyro_roll2, gyro_pitch2, gyro_yaw2;
float angle_roll, angle_pitch, angle_yaw;
float acc_roll, acc_pitch;

float alpha = 0.04;

float dt = 0.004;

float pid_error_temp = 0;
float pid_p_gain_roll =1.3;
float pid_i_gain_roll =0.003;
float pid_d_gain_roll =20.0;
int pid_max_roll = 400;

float pid_p_gain_pitch = pid_p_gain_roll;
float pid_i_gain_pitch = pid_i_gain_roll;
float pid_d_gain_pitch = pid_d_gain_roll;
int pid_max_pitch = pid_max_roll;

float pid_p_gain_yaw = 2.00;
float pid_i_gain_yaw = 0.02;
float pid_d_gain_yaw = 0.00;
int pid_max_yaw = 400;

boolean auto_level = true; // Auto level on (true) or off (false)
float gyro_roll_input, gyro_pitch_input, gyro_yaw_input;

float pid_i_mem_roll, pid_roll_setpoint, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, pid_output_yaw, pid_last_yaw_d_error;
double gyro_axis_cal[3] = {0, 0, 0};
float angle_roll_acc, angle_pitch_acc;
boolean gyro_angles_set;

float roll_level_adjust, pitch_level_adjust;

int throttle, esc_1, esc_2, esc_3, esc_4, cal_int;

float battery_voltage = 0;

volatile uint8_t esc_no = 0;
volatile uint16_t esc_ticks[4];
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4;
unsigned long loop_timer, esc_timer, esc_loop_timer, current_time;

volatile int ppm[CHANNELS];
volatile uint8_t channelIndex = 0;
volatile uint32_t lastTime = 0;

void raw_value_print();
void angles_print();
void read_MPU_data();

void MPU_Init();
void set_gyro_registers();

// void setupTimer1ESC();

void calculate_pid();

void ppmISR()
{
  uint32_t now = micros();
  uint32_t diff = now - lastTime;
  lastTime = now;

  if (diff > PPM_SYNC_GAP)
  {
    channelIndex = 0;
    // Serial.println();
  }

  else
  {
    if (channelIndex < CHANNELS)
    {
      ppm[channelIndex] = diff;

      // Serial.print(",");
      // Serial.print(diff);
      channelIndex++;
    }
  }
}

void setup()
{

  // put your setup code here, to run once:

  Serial.begin(115200);

  state = DISARMED; // initial state
  Wire.begin();

  DDRD |= B00011000; // Set pin 4 as OUTPUT for LED and pin 3 as OUTPUT for ESC1
  DDRB |= B00001110; // Set pin 11,10,9 as OUTPUT for ESCs

  for (int i = 0; i < CHANNELS; i++)
    ppm[i] = 0;
  digitalWrite(4, HIGH);
  delay(500);

  MPU_Init();
  // set_gyro_registers();

  for (cal_int = 0; cal_int < 1250; cal_int++)
  { // Wait 5 seconds before continuing.
    PORTD |= B00001000;
    PORTB |= B00001110;      // Set digital poort 3, 9, 10 and 11 high.
    delayMicroseconds(1000); // Wait 1000us.
    PORTD &= ~(B00001000);
    PORTB &= ~(B00001110);   // Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000); // Wait 3000us.
  }

  for (cal_int = 0; cal_int < 2000; cal_int++)
  { // Take 2000 readings for calibration.
    if (cal_int % 15 == 0)
      digitalWrite(4, !digitalRead(4)); // Change the led status to indicate calibration.
    read_MPU_data();                    // Read the gyro output.
    gyro_axis_cal[0] += gyro_axis[0];   // Ad roll value to gyro_roll_cal.
    gyro_axis_cal[1] += gyro_axis[1];   // Ad pitch value to gyro_pitch_cal.
    gyro_axis_cal[2] += gyro_axis[2];   // Ad yaw value to gyro_yaw_cal.
    // We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
    PORTD |= B00001000;
    PORTB |= B00001110;      // Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000); // Wait 1000us.
    PORTD &= ~(B00001000);
    PORTB &= ~(B00001110); // Set digital poort 4, 5, 6 and 7 low.
    delay(3);              // Wait 3 milliseconds before the next loop.
  }

  gyro_axis_cal[0] /= 2000; // Divide the roll total by 2000.
  gyro_axis_cal[1] /= 2000; // Divide the pitch total by 2000.
  gyro_axis_cal[2] /= 2000;

  Serial.print(String(gyro_axis_cal[0]) + "," + String(gyro_axis_cal[1]) + "," + String(gyro_axis_cal[2]));
  pinMode(PPM_PIN, INPUT_PULLUP); // PPM idle is HIGH
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, FALLING);

  delay(1000); // wait for a second

  state = DISARMED;

  // Serial.println(state);

  // Attach ESCs

  // --- ESC CALIBRATION SEQUENCE ---

  /*
    for (cal_int = 0; cal_int < 375; cal_int++)
    { // Wait 5 seconds before continuing.
      PORTD |= B00001000;
      PORTB |= B00001110;      // Set digital poort 4, 5, 6 and 7 high.
      delayMicroseconds(2000); // Wait 1000us.
      PORTD &= ~(B00001000);
      PORTB &= ~(B00001110);   // Set digital poort 4, 5, 6 and 7 low.
      delayMicroseconds(3000); // Wait 3000us.
    }
    digitalWrite(4, !digitalRead(4));
    for (cal_int = 0; cal_int < 375; cal_int++)
    { // Wait 5 seconds before continuing.
      PORTD |= B00001000;
      PORTB |= B00001110;      // Set digital poort 4, 5, 6 and 7 high.
      delayMicroseconds(2000); // Wait 1000us.
      PORTD &= ~(B00001000);
      PORTB &= ~(B00001110);   // Set digital poort 4, 5, 6 and 7 low.
      delayMicroseconds(3000); // Wait 3000us.
    }
    digitalWrite(4, !digitalRead(4));

    for (cal_int = 0; cal_int < 375; cal_int++)
    { // Wait 5 seconds before continuing.
      PORTD |= B00001000;
      PORTB |= B00001110;      // Set digital poort 4, 5, 6 and 7 high.
      delayMicroseconds(1000); // Wait 1000us.
      PORTD &= ~(B00001000);
      PORTB &= ~(B00001110);   // Set digital poort 4, 5, 6 and 7 low.
      delayMicroseconds(4000); // Wait 3000us.
    }
    digitalWrite(4, !digitalRead(4));
    for (cal_int = 0; cal_int < 375; cal_int++)
    { // Wait 5 seconds before continuing.
      PORTD |= B00001000;
      PORTB |= B00001110;      // Set digital poort 4, 5, 6 and 7 high.
      delayMicroseconds(1000); // Wait 1000us.
      PORTD &= ~(B00001000);
      PORTB &= ~(B00001110);   // Set digital poort 4, 5, 6 and 7 low.
      delayMicroseconds(4000); // Wait 3000us.
    }
      */

  // Load the battery voltage to the battery_voltage variable.
  // 65 is the voltage compensation for the diode.
  // 12.6V equals ~5V @ Analog 0.
  // 12.6V equals 1023 analogRead(0).
  // 1260*11 / 1023 = .
  // The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  battery_voltage = analogRead(0) * 12.90;

  loop_timer = micros(); // Set the timer for the next loop.

  digitalWrite(4, LOW);
}

void loop()
{
  
  //16.4 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  //gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_axis[0]/16.4) * 0.3);   //Gyro pid input is deg/sec.
  //gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_axis[1]/16.4) * 0.3);//Gyro pid input is deg/sec.
  //gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_axis[2]/16.4) * 0.3);      //Gyro pid input is deg/sec.

  gyro_roll_input = angle_roll;   //Gyro pid input is deg/sec.
  gyro_pitch_input = angle_pitch;//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_axis[2]/16.4) * 0.3);      //Gyro pid input is deg/sec.

  gyro_roll2 = gyro_roll;
  gyro_pitch2 = gyro_pitch;
  gyro_yaw2 = gyro_yaw;



  float ay = (float)acc_y;
  float az = (float)acc_z;
  float ax = (float)acc_x;

  acc_roll = atan2(ay, az) * 57.296f;
  acc_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 57.296f;

  // Serial.println(String(acc_roll)+","+String(acc_pitch)+",");
  // Serial.println(String(acc_x)+","+String(acc_y)+","+String(acc_z));

  gyro_roll = gyro_roll + gyro_axis[0] * 0.000244;
  gyro_pitch = gyro_pitch + gyro_axis[1] * 0.000244;
  gyro_yaw = gyro_yaw + gyro_axis[2] * 0.000244;

  angle_roll = gyro_roll * 0.996 + acc_roll * 0.004;
  angle_pitch = gyro_pitch * 0.996 + acc_pitch * 0.004;
  angle_yaw = gyro_yaw;

  gyro_roll = angle_roll;
  gyro_pitch = angle_pitch;

  //pitch_level_adjust = angle_pitch * 10.0;                                    //Calculate the pitch angle correction
  //roll_level_adjust = angle_roll * 10.0; 

  // MPU_raw_to_used1();
   angles_print();

  // state machine
  if (state == DISARMED && ppm[0] < 1050 && ppm[1] < 1050)
    state = ARMED;

  if (state == ARMED && ppm[0] < 1050 && ppm[1] > 1450)
  {
    state = TAKEOFF;

    angle_pitch = acc_pitch; // Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = acc_roll;   // Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    gyro_angles_set = true;
   

    // reset the pid controllers
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }

  if (state == TAKEOFF && ppm[0] < 1050 && ppm[1] > 1950)
    state = DISARMED;

  // Set PID setpoint for roll
  pid_roll_setpoint = 0;
  if (ppm[2] > 1520)
    pid_roll_setpoint = ppm[2] - 1520;
  else if (ppm[2] < 1480)
    pid_roll_setpoint = ppm[2] - 1480;

  //pid_roll_setpoint -= roll_level_adjust;
  pid_roll_setpoint = pid_roll_setpoint / 50.0; // scale to manageable setpoint
  

  // Set PID setpoint for pitch
  pid_pitch_setpoint = 0;
  if (ppm[3] > 1530)
    pid_pitch_setpoint = ppm[3] - 1530;
  else if (ppm[3] < 1490)
    pid_pitch_setpoint = ppm[3] - 1490;
  //pid_pitch_setpoint -= pitch_level_adjust;
  pid_pitch_setpoint = pid_pitch_setpoint / 50.0; // scale to manageable setpoint

  // Set PID setpoint for yaw
  pid_yaw_setpoint = 0;
  if (ppm[0] > 1050)
  {
    if (ppm[1] > 1530)
      pid_yaw_setpoint = ppm[1] - 1530;
    else if (ppm[1] < 1490)
      pid_yaw_setpoint = ppm[1] - 1490;
  }
  pid_yaw_setpoint = 0; //pid_yaw_setpoint / 3.0; // scale to manageable setpoint

  calculate_pid();

  // The battery voltage is needed for compensation.
  // A complementary filter is used to reduce noise.
  // 0.09853 = 0.08 * 12.90.

  battery_voltage = battery_voltage * 0.92 + analogRead(0) * 1.032;
  if (battery_voltage < 1000 && battery_voltage > 600)
    digitalWrite(4, HIGH);

  // Turn on the led if battery voltage is to low.
  //Serial.print(String(ppm[0]) + "," + String(ppm[1]) + "," + String(ppm[2]) + "," + String(ppm[3]) + "," + String(ppm[4]) + "," + String(state) + "\n");
  throttle = ppm[4];

  if (state == TAKEOFF)
  {
    if (throttle > 1800)
      throttle = 1800;
    esc_1 = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw;
    esc_2 = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw;
    esc_3 = throttle + pid_output_pitch + pid_output_roll - pid_output_yaw;
    esc_4 = throttle + pid_output_pitch - pid_output_roll + pid_output_yaw;
    
    if (esc_1 < 1100)
      esc_1 = 1100;
    if (esc_2 < 1100)
      esc_2 = 1100;
    if (esc_3 < 1100)
      esc_3 = 1100;
    if (esc_4 < 1100)
      esc_4 = 1100;

    if (esc_1 > 2000)
      esc_1 = 2000;
    if (esc_2 > 2000)
      esc_2 = 2000;
    if (esc_3 > 2000)
      esc_3 = 2000;
    if (esc_4 > 2000)
      esc_4 = 2000;
  }

  else
  {
    esc_1 = 1000;
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;
  }
  

  while (micros() - loop_timer < 4000); // We wait until 5000us are passed.
  loop_timer = micros();

  timer_channel_1 = esc_1 + loop_timer;
  timer_channel_2 = esc_2 + loop_timer;
  timer_channel_3 = esc_3 + loop_timer;
  timer_channel_4 = esc_4 + loop_timer;

  PORTD |= B00001000;
  PORTB |= B00001110; // Set digital poort 4 high.

  //long unsigned t0 = micros();
  read_MPU_data();

  //long unsigned t1 = micros();
  //Serial.println(t1 - t0);

  while ((PORTD & B00001000) || (PORTB & B00001110)) // While one of the ports is high.
  {
    esc_loop_timer = micros();

    if (esc_loop_timer >= timer_channel_1)
      PORTD &= ~(B00001000); // Set digital poort 3 low.
    if (esc_loop_timer >= timer_channel_2)
      PORTB &= ~(B00000010); // Set digital poort 9 low.
    if (esc_loop_timer >= timer_channel_3)
      PORTB &= ~(B00000100); // Set digital poort 10 low.
    if (esc_loop_timer >= timer_channel_4)
      PORTB &= ~(B00001000); // Set digital poort 11 low.
  }
}

// Initialisation of MPU and its all callibrations //
void MPU_Init()
{
  Wire.begin();
  Wire.setClock(400000); // IMPORTANT for I2C speed

  if (!myMPU6500.init())
  {
    Serial.println("MPU6500 does not respond");
    while (1)
      ;
  }

  Serial.println("Position you MPU6500 flat and don't move it - Calibrationg...");
  delay(1000);

  myMPU6500.autoOffsets();
  Serial.println("Calibration Done");

  // Gyro Settings
  myMPU6500.enableGyrDLPF();
  myMPU6500.setGyrDLPF(MPU6500_DLPF_3);
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_2000); // Set full scale gyro range to +/-2000deg/s

  // Accelerometer Settings
  myMPU6500.enableAccDLPF(true);
  myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
  // myMPU6500.setAccDLPF(MPU6500_DLPF_3);

  myMPU6500.setSampleRateDivider(1); // Set Gyro and Accel to 1kHz
}



void set_gyro_registers()
{
  // Setup the MPU-6050

  Wire.beginTransmission(gyro_address); // Start communication with the address found during search.
  Wire.write(0x6B);                     // We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                     // Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();               // End the transmission with the gyro.

  Wire.beginTransmission(gyro_address); // Start communication with the address found during search.
  Wire.write(0x1B);                     // We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x08);                     // Set the register bits as 00001000 (500dps full scale)
  Wire.endTransmission();               // End the transmission with the gyro

  Wire.beginTransmission(gyro_address); // Start communication with the address found during search.
  Wire.write(0x1C);                     // We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x10);                     // Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission();               // End the transmission with the gyro

  // Let's perform a random register check to see if the values are written correct
  Wire.beginTransmission(gyro_address); // Start communication with the address found during search
  Wire.write(0x1B);                     // Start reading @ register 0x1B
  Wire.endTransmission();               // End the transmission
  Wire.requestFrom(gyro_address, 1);    // Request 1 bytes from the gyro
  while (Wire.available() < 1)
    ; // Wait until the 6 bytes are received
  if (Wire.read() != 0x08)
  {                        // Check if the value is 0x08
    digitalWrite(4, HIGH); // Turn on the warning led
    while (1)
      delay(10); // Stay in this loop for ever
  }

  Wire.beginTransmission(gyro_address); // Start communication with the address found during search
  Wire.write(0x1A);                     // We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                     // Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();               // End the transmission with the gyro
}

void read_MPU_data()
{
  Wire.beginTransmission(gyro_address);
  Wire.write(0x3B); // Start reading at ACCEL_XOUT_H
  Wire.endTransmission();
  Wire.requestFrom(gyro_address, 14); // Request 14 bytes

  while (Wire.available() < 14)
    ; // Wait for data

  // Read accelerometer
  acc_axis[0] = Wire.read() << 8 | Wire.read();
  acc_axis[1] = Wire.read() << 8 | Wire.read();
  acc_axis[2] = Wire.read() << 8 | Wire.read();

  // Read temperature
  temperature = Wire.read() << 8 | Wire.read();

  // Read gyroscope
  gyro_axis[0] = Wire.read() << 8 | Wire.read();
  gyro_axis[1] = Wire.read() << 8 | Wire.read();
  gyro_axis[2] = Wire.read() << 8 | Wire.read();

  if (cal_int >= 2000)
  {
    gyro_axis[0] -= gyro_axis_cal[0]; // Only compensate after the calibration.
    gyro_axis[1] -= gyro_axis_cal[1]; // Only compensate after the calibration.
    gyro_axis[2] -= gyro_axis_cal[2]; // Only compensate after the calibration.
                                      // Only compensate after the calibration.
  }

  // Serial.println(String(gyro_axis[0])+","+String(gyro_axis[1])+","+String(gyro_axis[2]));

  // Map accelerometer axes
  acc_x = acc_axis[0];
  acc_y = acc_axis[1];
  acc_z = acc_axis[2];
}

void raw_value_print()
{
  Serial.println();
  Serial.print("Acceleration in g (x,y,z): ");
  Serial.print("X:");
  Serial.print(gValue.x);
  Serial.print(",");
  Serial.print("Y:");
  Serial.print(gValue.y);
  Serial.print(",");
  Serial.print("Z:");
  Serial.println(gValue.z);
  Serial.print("Gyroscope data in degrees: ");
  Serial.print("X:");
  Serial.print(gyr.x);
  Serial.print(",");
  Serial.print("Y:");
  Serial.print(gyr.y);
  Serial.print(",");
  Serial.print("Z:");
  Serial.print(gyr.z);
  Serial.println();
}

void angles_print()
{
  // Serial.println();
  // Serial.print("Angles : ");
  Serial.print(angle_roll);
  Serial.print(",");
  Serial.print(angle_pitch);
  Serial.print(",");
  Serial.print(angle_yaw);
  Serial.println();
}

void calculate_pid()
{
  // Calculate PID for roll
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll)
    pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < -pid_max_roll)
    pid_i_mem_roll = -pid_max_roll;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll)
    pid_output_roll = pid_max_roll;
  else if (pid_output_roll < -pid_max_roll)
    pid_output_roll = -pid_max_roll;

  pid_last_roll_d_error = pid_error_temp;

  // Calculate PID for pitch
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch)
    pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < -pid_max_pitch)
    pid_i_mem_pitch = -pid_max_pitch;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)
    pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < -pid_max_pitch)
    pid_output_pitch = -pid_max_pitch;

  pid_last_pitch_d_error = pid_error_temp;

  // Calculate PID for yaw
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw)
    pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < -pid_max_yaw)
    pid_i_mem_yaw = -pid_max_yaw;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw)
    pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < -pid_max_yaw)
    pid_output_yaw = -pid_max_yaw;
  pid_last_yaw_d_error = pid_error_temp;
}
