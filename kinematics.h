#ifndef _Kinematics
#define _Kinematics_h


class Kinematics_c {
  public:
  
    //Public variables and methods go here
    float x;
    float y; 
    float theta;
    float last_theta;
    long last_e0;
    long last_e1;

    /* 
     *  Function prototypes 
     */
    Kinematics_c::Kinematics_c();
    void update( long e0, long e1 );
    void setPose( float _x, float _y, float _theta );


  private:

    float wheel_sep         = (143 / 2);
    float wheel_radius      = 35;
    float rad_per_enc_step  = (TWO_PI / 1440.0);
    float mm_per_enc_step   = 0.15;

}; // End of class definition.


/*
 * Constructor, zeros variables.
 */
Kinematics_c::Kinematics_c() {
  x = 0;
  y = 0;
  theta = 0;
  last_theta = 0;
  last_e0 = 0;
  last_e1 = 0;
} // end of constructor.


/*
 * Allows the kinematics to be set to an intial value
 * or reset.
 */
void Kinematics_c::setPose( float _x, float _y, float _theta ) {
  x     = _x;
  y     = _y;
  theta = _theta;
  
}

/*
 * Update for the kinematics.
 */
void Kinematics_c::update( long e0, long e1 ) {

  // This class stores the last received
  // encoder counts to calculate the difference
  // in encoder counts for itself.
  long delta_e0 = e0 - last_e0;
  long delta_e1 = e1 - last_e1;
  last_e0 = e0;
  last_e1 = e1;

  // Rotation for each wheel, using
  // the number of radians per encoder count
  // We're only keeping track of distance travelled
  // (a cartesian coordinate position), not speed, 
  // so we don't need to factor elapsed time here.
  // However, knowing how long a time elapsed between
  // update() might help to estimate error in the 
  // approximation (?)
  float av0;    // av, angular velocity
  av0 = rad_per_enc_step;
  av0 = av0 * (float)delta_e0;

  float av1;
  av1 = rad_per_enc_step;
  av1 = av1 * (float)delta_e1;


  // Kinematics without ICC projection
  // Some error is going to accumulate.
  // But with a quick enough update, its a pretty good
  // straight-line approximation for curves.
  // new_x becuase we are assuming that travel is only in x direction. no moment in y
  float new_x = (( av0 * wheel_radius ) + ( av1 * wheel_radius )) / 2;
  
  float new_theta = ( (av1 * wheel_radius ) - ( av0 * wheel_radius)  ) / (2 * wheel_sep );
  //float new_theta = ( av0 - av1 ) / (2*wheel_sep);
  
  
  // record current theta as 'old' so that we can
  // keep track of angular change else where in 
  // the program.
  last_theta = theta;
  

  // Update global theta.
  theta = theta + new_theta;

  // Lets wrap theta to keep it between 0 and TWO_PI
  // Not strictly necessary, but predictable at least.
  while( theta < 0 ) theta += TWO_PI;
  while( theta > TWO_PI ) theta -= TWO_PI;

  // Integrate this movement step by rotating
  // the x contribution by theta, therefore "sharing" it
  // between x and y in the global reference frame.
  x = x + (new_x * cos( theta ) );
  y = y + (new_x * sin( theta ) );

} // End of update()





#endif
