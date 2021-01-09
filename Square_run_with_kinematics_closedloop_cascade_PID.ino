#include "pid.h"
#include "kinematics.h"
#include "LineSensor.h"
#include "encoders.h"
#include "motor.h"
#include <Wire.h>
#include <LSM6.h>

LSM6 imu;

//Bluetooth instance
HardwareSerial& bthc05(Serial1);

#define E1_A_PIN       7   // Encoder Pins
#define E1_B_PIN      23
#define E0_A_PIN      26

#define LINE_LEFT_PIN   22   // Line Sensor left pin
#define LINE_CENTRE_PIN 21 // Line Sensor centre pin
#define LINE_RIGHT_PIN  20   // Line Sensor right pin

#define L_PWM_PIN   10 // Motor Pins.
#define L_DIR_PIN   16
#define R_PWM_PIN    9
#define R_DIR_PIN   15

// Flags used for a basic finite state machine
#define INITIAL_STATE     0
#define FOLLOW_SQUARE     1
#define RESTING           2

int STATE = INITIAL_STATE;  // System starts by driving straight

unsigned long spd_update_ts;
unsigned long pid_update_ts;
unsigned long pose_print_ts;
unsigned long initial_state_ts;
unsigned long T_update_ts;

// variables for speed estimate
long          last_el_count;
long          last_er_count;
float el_spd;
float er_spd;

// variables for speed demand
float dl_forward;
float dr_forward;
float d_turn;
float T; //Theta required value


// variables for power to motors
float pwr_l;
float pwr_r;

int state_sqr = 0;
#define First_LEG   0
#define Second_LEG  1
#define Third_LEG   2
#define Fourth_LEG  3
#define Last_LEG    4

// Gains for motor PID
#define Kp_left      30 //Proportional gain
#define Kd_left      0 //Derivative gain
#define Ki_left     0.7 //Integral gain
#define Kp_right     30 //Proportional gain
#define Kd_right     0 //Derivative gain
#define Ki_right    0.7 //Integral gain
#define H_PGAIN     1.2
#define H_IGAIN   0.002
#define H_DGAIN    -2.5


// Class instances.
Motor_c       L_Motor( L_PWM_PIN, L_DIR_PIN);                       // To set left motor power.
Motor_c       R_Motor( R_PWM_PIN, R_DIR_PIN);
LineSensor_c  line_left(LINE_LEFT_PIN);
LineSensor_c  line_centre(LINE_CENTRE_PIN);
LineSensor_c  line_right(LINE_RIGHT_PIN);
PID_c         left_PID(Kp_left, Ki_left, Kd_left);
PID_c         right_PID(Kp_right, Ki_right, Kd_right);
PID_c         H_PID(H_PGAIN, H_IGAIN , H_DGAIN );
Kinematics_c  RomiPose;

//These definitions below are about the gyro
int16_t gyroOffset;
uint16_t gyroLastUpdate = 0;
uint32_t turnAngle = 0;
int16_t turnRate;
float turnRotation = 0;
// This constant represents a turn of 45 degrees.
const int32_t turnAngle45 = 0x20000000;
// This constant represents a turn of 90 degrees.
const int32_t turnAngle90 = turnAngle45 * 2;
// This constant represents a turn of approximately 1 degree.
const int32_t turnAngle1 = (turnAngle45 + 22) / 45;

void turnSensorReset()
{
  gyroLastUpdate = micros();
  turnAngle = 0;
}

void turnSetup(uint16_t cal_rot_rate) {
  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < cal_rot_rate; i++)
  {
    // Wait for new data to be available, then read it.
    while (!imu.readReg(LSM6::STATUS_REG) & 0x08);
    imu.read();
    // Add the Z axis reading to the total.
    total += imu.g.z;
  }

  gyroOffset = total / cal_rot_rate;

  // Display the angle (in degrees from -180 to 180) until the
  gyroLastUpdate = micros();
  turnAngle = 0;
}

void turnUpdate() {
  // Read the measurements from the gyro.
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;

  // Figure out how much time has passed since the last update (dt)
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  // Multiply dt by turnRate in order to get an estimation of how
  // much the robot has turned since the last update.
  // (angular change = angular velocity * time)
  float d = (int32_t)turnRate * dt;

  // The units of d are gyro digits times microseconds.  We need
  // to convert those to the units of turnAngle, where 2^29 units
  // represents 45 degrees.  The conversion from gyro digits to
  // degrees per second (dps) is determined by the sensitivity of
  // the gyro: 0.035 degrees per second per digit.
  //
  // (0.035 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
  // = 7340032/17578125 unit/(digit*us)
  turnAngle += (int64_t)d * 7340032.0 / 17578125.0;
  turnRotation = turnAngle / (int32_t)turnAngle1;
}


// put your setup code here, to run once:
void setup() {

  // These two function set up the pin
  // change interrupts for the encoders.
  setupEncoder0();
  setupEncoder1();

  // Make sure motors are off so the robot
  // stays still.
  L_Motor.setPower( 0 );
  R_Motor.setPower( 0 );

  // Initialise the Serial communication
  Wire.begin();
  Serial.begin( 9600 );
  bthc05.begin(9600);
  delay(1500);
  // Flag up reset to Serial monitor
  Serial.println("*** RESET ***");

  //calibrating line sensors
  //line_left.calibrate();
  //line_centre.calibrate();
  //line_right.calibrate();

  spd_update_ts = millis(); //Initialize time stamps
  pid_update_ts = millis();
  pose_print_ts = millis();
  initial_state_ts = millis();
  T_update_ts =   millis();

  el_spd =         0;
  er_spd =         0;
  last_el_count =  0;
  last_er_count =  0;

  dl_forward =  .7;
  dr_forward =  .7;
  d_turn =       0;
  T =            0;
  pwr_l =        0;
  pwr_r =        0;

  // We set the robot to start kinematics
  // X = 0, Y = 0, Theta = 0.05
  RomiPose.setPose( 0, 0, 0 );

  if (!imu.init()) {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();
  // Set the gyro full scale to 1000 dps because the default
  // value is too low, and leave the other settings the same.
  imu.writeReg(LSM6::CTRL2_G, 0b10001000);
  turnSensorReset();
  turnSetup(1024);

}

// Main code runnig repeatedly
void loop() {
  // Updating kinematics
  RomiPose.update( count_el, count_er );
  printPose();  //print pose parameters

  if ( STATE == INITIAL_STATE  ) {
    intialisingBeeps();
  }
  else if ( STATE == FOLLOW_SQUARE  ) {
    if (RomiPose.theta < 7) {
      speed_update(); //get an update on speed
      pidUpdate();
      square_run(500, 500); //provide lenght of Shape to follow in mm
    }
    else {
      STATE = 2;
    }
  }
  else if ( STATE == RESTING) {
    L_Motor.setPower(0);
    R_Motor.setPower(0);
    //Serial.println("Resting...........");
    delay(500);
  }

  //print_count();  //prints the counts for both wheels
  delay(1);

} // End of main loop

void intialisingBeeps() {
  // fuction for state 0
  unsigned long initial_state_dt = millis() - initial_state_ts;
  if ( initial_state_dt < 3000 ) {

    analogWrite(6, 10);
    delay(100);
    analogWrite(6, 0);
    delay(900);

  } else {

    left_PID.reset();
    right_PID.reset();
    H_PID.reset();

    STATE =   1;
  }
}

void pidUpdate() {
  // output_signal <----PID-- demand, measurement
  unsigned long pid_update_dt = millis() - pid_update_ts;

  if ( pid_update_dt > 20) {
    pid_update_ts = millis();
    //d_turn = H_PID.update(0.05,RomiPose.theta);
    pwr_l = left_PID.update(dl_forward - d_turn, el_spd);
    pwr_r = right_PID.update(dr_forward + d_turn, er_spd);
    //left_PID.printComponents();
    //Serial.println(dl_forward-d_turn);


    L_Motor.setPower(pwr_l);
    R_Motor.setPower(pwr_r);

  }
}

//This function move Romi on a shape with X and Y sides dimensions
void square_run(int X, int Y) {
  turnUpdate();
  float loc_turn = (float)turnRotation / 1.07 * PI/180;
  if (state_sqr == First_LEG) {
    if (RomiPose.x < X - 40) {
      //The -40 is used to start turing before reaching
      //final position in X to avoid overshooting

      //d_turn is the demand for truning left or right
      d_turn = H_PID.update(0, tan(loc_turn));
      //Taking tan to avoid big radian value when angle theta
      // is less  than Zero. Tan will be + and - around 0 radian
    }
    else {
      state_sqr = 1;
    }
  }

  if (state_sqr == Second_LEG) {
    if (RomiPose.y < Y - 40) {
      //Sending required Theta value to T_Update
      T_update(1.57);

      //Check for edge case where theta is less
      //than Zero deg, convert it to negative value.
      float t = loc_turn;
      if (t > 5) {
        t = TWO_PI - t;
      } else {
        t = t;
      }

      d_turn = H_PID.update(T, t);
      //Serial.println(T);
    }
    else {
      state_sqr = 2;
    }
  }

  if (state_sqr == Third_LEG) {
    if (RomiPose.x > 40 ) {
      T_update(3.14);
      d_turn = H_PID.update(T, loc_turn);
    }
    else {
      state_sqr = 3;
    }
  }

  if (state_sqr == Fourth_LEG) {
    if (RomiPose.y > 40 ) {
      T_update(4.71);
      d_turn = H_PID.update(T, loc_turn);
    }
    else {
      state_sqr = 4;
    }
  }

  if (state_sqr == Last_LEG) {
    if (RomiPose.theta < 6.2 ) {
      T_update(6.2);
      d_turn = H_PID.update(T, loc_turn);
    }
    else {
      STATE = 2;
    }
  }


}
// This T_update function increase the theta value incrementally to the new desired theta
void T_update(float d) { //d is new dissered theta value
  unsigned long T_update_dt = millis() - T_update_ts;
  if ( T_update_dt > 3 ) { //updates every 3ms
    T_update_ts = millis();
    if ( T < d ) {
      T = T + .01;
    }
    else {
      T = d;
    }

  }
}

void printPose() {
  unsigned long pose_print_dt = millis() - pose_print_ts;

  if (pose_print_dt > 100) {
    pose_print_ts = millis();
    Serial.print(RomiPose.x);
    Serial.print("\t");
    Serial.print(RomiPose.y);
    Serial.print( "\t" );
    Serial.println( RomiPose.theta );
  }
}

void speed_update() {
  unsigned long spd_update_dt = millis() - spd_update_ts;

  if (spd_update_dt > 20) {
    spd_update_ts = millis();

    long el_diff = count_el - last_el_count;
    last_el_count = count_el;

    el_spd = (float)el_diff / (float)spd_update_dt;

    long er_diff = count_er - last_er_count;
    last_er_count = count_er;

    er_spd = (float)er_diff / (float)spd_update_dt;
    //spd_avg = (spd_avg* .8) + (eo_spd * .2) ;
    //Serial.print(spd_avg);
    //Serial.print(",");
    //Serial.print( el_spd );
    //Serial.print( "," );
    //Serial.print( er_spd );
    //Serial.print( "," );
    //Serial.println( Faward_Bias );

  }

}


void print_count() {
  Serial.print( count_el);
  Serial.print( ", ");
  Serial.println( count_er );

  delay( 2 );
}
