// Required Libraries
#include <Adafruit_CircuitPlayground.h>
#include <Wire.h>
#include <SPI.h>
 
// Accelerometer variables
float X = 0;
float Y = 0; 
float Z = 0;

// Motion sensing variables:
int Sum_counter = 0;
float X_sum = 0;
float Y_sum = 0;
float Z_sum = 0;
float X_avg = 0;
float Y_avg = 0;
float Z_avg = 0;
int avg_window = 10;
bool X_motion_check = false;
bool Y_motion_check = false;
bool Z_motion_check = false;
bool continous_check = false;
// time for LED alarm and Audio alarm
int start_timer = 0;
int delta_timer = 0;
 
void setup() 
{
  // Start serial terminal:
  Serial.begin(115200);
  // Start circuit playground:
  CircuitPlayground.begin();
  CircuitPlayground.setAccelRange(LIS3DH_RANGE_2_G);
}
 
void loop() 
{
  // Previous X, Y, and Z values:
  float X_prev = X;
  float Y_prev = Y;
  float Z_prev = Z;
  // Get X, Y, and Z accelerometer:
  X = CircuitPlayground.motionX();
  Y = CircuitPlayground.motionY();
  Z = CircuitPlayground.motionZ();
  // Test print:
  Serial.print(X);
  Serial.print(",");
  Serial.print(Y);
  Serial.print(",");
  Serial.println(Z);

  // Caluculate average:
  X_sum += X;		// Summing X motion values
  Y_sum += Y;		// Summing Y motion values
  Z_sum += Z;		// Summing Z motion values
  Sum_counter++;

  if (Sum_counter == avg_window)
  {
  	X_avg = X_sum/avg_window;			// X average after 20 iterations
  	Y_avg = Y_sum/avg_window;			// Y average after 20 iterations
  	Z_avg = Z_sum/avg_window;			// Z average after 20 iterations

  	// Make averaging values zero
  	X_sum = 0;
  	Y_sum = 0;
  	Z_sum = 0;
  	Sum_counter = 0;
  }

  // Check for no motion: 
  if (X_avg <= (X+0.8) && X_avg >= (X-0.8))
  {
  	X_motion_check = true;			// Set true if there is NO motion
  }
  else 
  {
  	X_motion_check = false;			// Set false if there IS motion
  }
  if (Y_avg <= (Y+0.8) && Y_avg >= (Y-0.8))
  {
  	Y_motion_check = true;			// Set true if there is NO motion 
  }
  else
  {
  	Y_motion_check = false;			// Set false if there IS motion
  }
  if (Z_avg <= (Z+0.8) && Z_avg >= (Z-0.8))
  {
  	Z_motion_check = true;			// Set true if there is NO motion
  }
  else
  {
  	Z_motion_check = false;			// Set false if there IS motion
  }
  // There is NO Motion:
  if (X_motion_check == true && Y_motion_check == true && Z_motion_check == true)
  {
  	if (continous_check == false)
  	{
  		start_timer = micros();			// Start timer 
  		continous_check = true;			// Check if NO motion continous
  	}
  	// Test Print:
  	//Serial.println("NO_motion");
  }
  // There IS motion:
  else 
  {
  	 start_timer = micros();
  	 continous_check = false;
  	 CircuitPlayground.clearPixels();
  	 // Test Print:
  	 //Serial.println("IS_motion");
  }

  delta_timer = micros() - start_timer;

  // If NO motion longer than 10's - red LED alarm:
  if (delta_timer >= 10000000)
  {
  	CircuitPlayground.setPixelColor(0, 255,   0,   0);
  	CircuitPlayground.setPixelColor(1, 255,   0,   0);
  	CircuitPlayground.setPixelColor(2, 255,   0,   0);
  	CircuitPlayground.setPixelColor(3, 255,   0,   0);
  	CircuitPlayground.setPixelColor(4, 255,   0,   0);
  	CircuitPlayground.setPixelColor(5, 255,   0,   0);
  	CircuitPlayground.setPixelColor(6, 255,   0,   0);
  	CircuitPlayground.setPixelColor(7, 255,   0,   0);
  	CircuitPlayground.setPixelColor(8, 255,   0,   0);
  	CircuitPlayground.setPixelColor(9, 255,   0,   0);
  }


  // If NO motion longer than 15's - audio alarm:
  if (delta_timer >= 15000000)
  {
  	while(1)
  	{
  		CircuitPlayground.playTone(200, 50);		// Play sound
  	}
  	
  }

  // Sensor is upside down - blue LED alarm
  if (Z < 0)
  {
  	CircuitPlayground.setPixelColor(0, 0,   0,   255);
  	CircuitPlayground.setPixelColor(1, 0,   0,   255);
  	CircuitPlayground.setPixelColor(2, 0,   0,   255);
  	CircuitPlayground.setPixelColor(3, 0,   0,   255);
  	CircuitPlayground.setPixelColor(4, 0,   0,   255);
  	CircuitPlayground.setPixelColor(5, 0,   0,   255);
  	CircuitPlayground.setPixelColor(6, 0,   0,   255);
  	CircuitPlayground.setPixelColor(7, 0,   0,   255);
  	CircuitPlayground.setPixelColor(8, 0,   0,   255);
  	CircuitPlayground.setPixelColor(9, 0,   0,   255);
  }

  // Sensor has fallen - green LED and audio alarm:
  if (X > (X_prev+10) || X < (X_prev-10) || Y > (Y_prev+10) || Y < (Y_prev-10) || Z > (Z_prev+10) || Z < (Z_prev-10))
  {
  	CircuitPlayground.setPixelColor(0, 0,   255,   0);
  	CircuitPlayground.setPixelColor(1, 0,   255,   0);
  	CircuitPlayground.setPixelColor(2, 0,   255,   0);
  	CircuitPlayground.setPixelColor(3, 0,   255,   0);
  	CircuitPlayground.setPixelColor(4, 0,   255,   0);
  	CircuitPlayground.setPixelColor(5, 0,   255,   0);
  	CircuitPlayground.setPixelColor(6, 0,   255,   0);
  	CircuitPlayground.setPixelColor(7, 0,   255,   0);
  	CircuitPlayground.setPixelColor(8, 0,   255,   0);
  	CircuitPlayground.setPixelColor(9, 0,   255,   0);
  	while(1)
  	{
  		CircuitPlayground.playTone(200, 50);		// Play sound
  	}
  }
}
