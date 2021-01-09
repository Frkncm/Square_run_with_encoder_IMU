#ifndef _Motor_h
#define _Motor_h

// A class to neatly contain commands for the 
// motors, to take care of +/- values, a min/max
// power value, & pin setup.

class Motor_c {

  public:
    Motor_c( int pwm, int dir );
    void setPower(int demand);
    
  private:
    int pwm_pin;
    int dir_pin;

};

// Constructor: pass in pins for a motor.
Motor_c::Motor_c( int pwm, int dir ) {
  pwm_pin = pwm;
  dir_pin = dir;

  pinMode(pwm_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);

  digitalWrite(pwm_pin, LOW);
  digitalWrite(dir_pin, LOW);
  
}




// Sets the power of the motor, takes
// care of +/- and min/max values.
// (typical errors)
void Motor_c::setPower(int demand) {
  
  //Handle setting directions
  if( demand < 0 ) {    
    digitalWrite(dir_pin, HIGH);
  } else {
    digitalWrite(dir_pin, LOW);
  }

  // Get rid of - sign.
  demand = abs(demand);

  // Keep between 0 255
  demand = constrain( demand, 0, 255 );
  
  //Write out.
  analogWrite(pwm_pin, demand );
}

#endif
