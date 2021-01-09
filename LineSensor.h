#ifndef _LineSensor_h
#define _LineSensor_h
#define buzz 6
class LineSensor_c {
  
  public:

    LineSensor_c(int line_pin)
    {
      pin = line_pin;
      pinMode(pin, INPUT);
    };

    int read_value() {
      return analogRead(pin);
    }
    // Write your calibration routine here
    // to remove bias offset
    void calibrate() {
      bias=0;
      Serial.print("Running Calibration on pin: ");
      Serial.println(pin);
      delay(50);
      float value = analogRead(pin);
      int c = 0;

      while (c < 50) {
        c = c + 1;
        value = value + analogRead(pin);
      }

      bias = (value / c);
      analogWrite(buzz, 10);
      delay(100);
      analogWrite(buzz, 0);
    }

    boolean online(float threshold)
    {
      float value = analogRead(pin);
      if (value < threshold) {
        return false;
      }
      else {
        return true;
      }


    }

    // Useful debug function to print out the raw
    // sensor values (e.g., with the offset present)
    void printRawReading() {
      Serial.print( analogRead( pin ) );
    }

    // Does a sensor read for left/centre/right, 
    // and removes the mean (offset).  Note, 
    // LEFT/CENTRE/RIGHT are defined as private
    // variables in this class.  
    float getCalibrated() {
       float sensor_read = (float)analogRead( pin );
       return (sensor_read - bias );
      
    }
  private:
   int pin; // to store hardware pins in use.
   float bias; 



};

#endif
