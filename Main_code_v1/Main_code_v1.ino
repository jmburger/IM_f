// Required Libraries
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

// MAX30105 sensor
#include <MAX30105.h>
MAX30105 MAX30105_sensor;

//Filter Variables
//DC Removal variable:
#define SAMPLE_SIZE   100            	// Mean difference filter sample size used to calculate the running mean difference
#define ALPHA_DCR     0.95             	// DC filter alpha value for LED
double prev_filtered_IR = 0;     		// HR
double prev_filtered_RED = 0;    		// Sp02
//Mean Difference variables:
float MeanDiff_TV[SAMPLE_SIZE];     	// Trailling measurements
uint8_t Index = 0;
uint8_t count = 0;
float sum = 0;
//Low pass filter (butterworth filter) variables:
float Val[3];

// Global values:
float Tb_val = 0;
int BPM_val = 0;
float SpO2_val = 0;
int RR_val = 0;
float HRV_val = 0;

// Flags
bool Prop_rec = true;       //Check to see if data was recorded properly if not redo recording.

//timer requiremnts:
int start_12s = 0;
int delta_12s = 0;
int start_2m = 0;
int delta_2m = 0;

// Adafruit Circuit Playground Express (ACPE) microcontroller:
#include <Adafruit_CircuitPlayground.h>

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

  // start-up the MAX30105 sensor
  MAX30105_Startup(); 
  delay(200);       

  Serial.println("INFANT MONITOR:");
  Serial.println("=========");
  Serial.println("");
  Serial.println("Warming Up.....");
  Serial.println("");

  //------------------
  // MAX30105 Warm Up:
  int recording_time = 4500000;     //Heart rate recording time
  int array_size = 225;
  HR_SpO2_RR_HRV_Tb(recording_time, array_size, false); 
  //------------------

  // Get HR SpO2 RR HRV:
  recording_time = 30000000;			//Heart rate recording time
  array_size = 2390;
  HR_SpO2_RR_HRV_Tb(recording_time, array_size, true);
  while (Prop_rec == false)
  {
    print_data();
    Accelerometer_ACPE();
    HR_SpO2_RR_HRV_Tb(recording_time, array_size, true);
  }

  // Print vital data:
  print_data();

  //Start timers:
  start_12s = micros();
  start_2m = micros();
}

void loop()
{
  Accelerometer_ACPE();
  // do every 12 seconds
  if(delta_12s >= 12000000)    
  {
	start_12s = micros();
	int recording_time_HR = 8000000;			//Heart rate recording time
    int array_size = 630;
    HR_SpO2_RR_HRV_Tb(recording_time_HR, array_size, false);
    while (Prop_rec == false)
    {
      Accelerometer_ACPE();
      print_data();
      HR_SpO2_RR_HRV_Tb(recording_time_HR, array_size, false);
    }
    // Print vital data:
	print_data(); 
  }
  delta_12s = micros() - start_12s; 

  // do every 2 minutes
  if(delta_2m >= 120000000)    
  {
	start_2m = micros();
	// On start up do current balancing:
	//Current_Balancing();	
	int recording_time_HR = 30000000;			//Heart rate recording time
    int array_size = 2390;
	HR_SpO2_RR_HRV_Tb(recording_time_HR, array_size, true);
    while (Prop_rec == false)
    {
      Accelerometer_ACPE();
      print_data();
      HR_SpO2_RR_HRV_Tb(recording_time_HR, array_size, true);
    }
    // Print vital data:
	print_data();   			
  }
  delta_2m = micros() - start_2m; 
}
 
void Accelerometer_ACPE() 
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
  // Serial.print(X);
  // Serial.print(",");
  // Serial.print(Y);
  // Serial.print(",");
  // Serial.println(Z);

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
  		CircuitPlayground.playTone(500, 50);		// Play sound
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
  	CircuitPlayground.playTone(50, 50);		// Play sound
  }

  // Sensor has fallen - green LED and audio alarm:
  if (X > (X_prev+15) || X < (X_prev-15) || Y > (Y_prev+15) || Y < (Y_prev-15) || Z > (Z_prev+15) || Z < (Z_prev-15))
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

void print_data()
{
	//Test print:
  Serial.println("INFANT MONITOR:");
  Serial.println("=========");
  Serial.println("");
  Serial.println("Vital Data:");
  Serial.println("-----------");
  if (Prop_rec == false)
  {
    Serial.println("");
    Serial.println("Error occured busy rerecording...");
    Serial.println("");
    Serial.println("");
  }
  else
  {
    Serial.print("Core Body Temperature: "); 
    // print core body temperature:
    Serial.print(Tb_val);  
    Serial.println(" °C");
    delay(20);
    // print heart rate:
    Serial.print("HR: ");
    if (BPM_val < 240 && BPM_val > 0)
    {
      Serial.print(BPM_val);
      Serial.println(" bpm");
    }
    else
    {
      Serial.println("Error");
    }
    // print heart rate varibility:
    Serial.print("HRV: ");
    Serial.println(HRV_val);
    if (HRV_val == 4)
    {
      Serial.println("Error");
    }
    // print blood oxygention:
    Serial.print("SpO2: ");
    if (SpO2_val <= 80 && SpO2_val >= 100)
    {
      Serial.println("Error");
    }
    else
    {
      Serial.print(int(SpO2_val));
      Serial.println(" %");
    } 
    delay(20);
    // print respratory rate:
    Serial.print("RR: ");
    Serial.print(RR_val);
    Serial.println(" pm");
    Serial.println("");
    delay(20);
  }
}

// Function to get HR, SpO2, RR, HRV values:
void HR_SpO2_RR_HRV_Tb(int rec_time, int array_size, bool RR_HRV)
{
  // Run accelrometer check:
  Accelerometer_ACPE();
  //Wake Sensor up:
  MAX30105_sensor.wakeUp();
  delay(200);
  //Variables store raw RED and IR values:
  uint32_t raw_IR_Val = 0;
  uint32_t raw_RED_Val = 0;
  int Expected_Peaks = ((rec_time)/1000000)*0.5;	//Expected beat peaks if heart rate is 30 bpm
  // Recording required varabiles:
  int window = 20;                        //length of analyzing window
  int IR_array_size = array_size;							// IR's size of array_.
  float IR_AC_array[IR_array_size];      						// IR signal AC array_.                     
  int i = 0;													// Counter
  float IR_DC_val = 0;               							// DC value of the IR signal
  float RED_DC_val = 0;              							// DC value of the RED signal
  float IR_DC_val_SpO2 = 0;          							// DC value of the IR signal for SpO2 calculation
  float RED_DC_val_SpO2 = 0;         							// DC value of the RED signal for SpO2 calculation
  float Sum_AC_IR = 0;               							// Sum of the IR AC signal value
  float Sum_AC_RED = 0;              							// Sum of the RED AC signal value
  int delta_rec = 0;				      						// delta time between current and start time
  int start_rec = micros();		  							// start record time
  for(int i = 0; i < IR_array_size; i++)
  {
    IR_AC_array[i] = 0;
  }
  // while loop to record data:
  while(delta_rec <= rec_time)
  {
  	// Run accelrometer check:
    Accelerometer_ACPE();
    // if raw data is available 
    while (MAX30105_sensor.available() == false) //do we have new data
    MAX30105_sensor.check();                     //chech for new data
    //Get IR and Red raw data:
    raw_IR_Val = MAX30105_sensor.getIR();
    raw_RED_Val = MAX30105_sensor.getRed();
    // Get Temperature:
    float Core_body_T = MAX30105_sensor.readTemperature();
    Tb_val = Core_body_T;
    MAX30105_sensor.nextSample();  // Finished with this sample so move to next sample
    // Test Print:
    // Serial.print(raw_IR_Val);
    // Serial.print(",");
    // Serial.println(raw_RED_Val);
    // Serial.println(i);
    // IR Signal:
    bool IR_DC = false;	//Return either DC value (true) or AC value (false)  
    IR_AC_array[i] = DCR_function_IR(raw_IR_Val, ALPHA_DCR, IR_DC);         //filter raw IR LED data through DC removal
    //Calculating AC RMS value: (only after 100 iterations - remove noise)
    if (i > 300 && i <= 400)
    {
      Sum_AC_IR += pow((IR_AC_array[i]),2);                                 //Sum of the IR AC signal value
    }
    //Add filtering to raw values:
    IR_AC_array[i] = MDF_function(IR_AC_array[i]);                          //mean difference filter IR LED data 
    IR_AC_array[i] = Butterworth_LPF_function(IR_AC_array[i]);              //low pass butterworth filter IR LED data
    //Get DC value from signal:
    IR_DC = true;
    IR_DC_val = DCR_function_IR(raw_IR_Val, ALPHA_DCR, IR_DC);        		//Get DC value from IR signal
    if (i == 350)
    {
      IR_DC_val_SpO2 = IR_DC_val;
    }
    // Test Print:
    //Serial.print(IR_DC_val);
    //Serial.print(" , ");
    // RED Signal:
    bool RED_DC = false;	//Return either DC value (true) or AC value (false)  
    float RED_AC_value = DCR_function_RED(raw_RED_Val, ALPHA_DCR, RED_DC); 	//filter raw RED LED data through DC removal
    //Calculating AC RMS value: (only after 50 iterations - remove noise)
    if (i > 300 && i <= 400)
    {
      Sum_AC_RED += pow((RED_AC_value),2);                               	//Sum of the RED AC signal value
    }
    //Get DC value from signal
    RED_DC = true;
    RED_DC_val = DCR_function_RED(raw_RED_Val, ALPHA_DCR, RED_DC);    		//Get DC value from RED signal
    if (i == 350)
    {
      RED_DC_val_SpO2 = RED_DC_val;
    }
    // Test Print:
    //Serial.println(RED_DC_val);
    if (i >= IR_array_size)
    {
      break;
    }
    // Test Print:
    // Serial.print(IR_AC_array[i]);
    // Serial.print(",");
    // Serial.print(IR_RED_array[i]);
    // Serial.println(i);

    i++;
    
    delta_rec = micros() - start_rec;						// delta time calculation 30 seconds 
  }
  // Shut down MAX30105 sensor:
  MAX30105_sensor.shutDown();       						// Shutdown MAX30105 sensor
  // Start data processing: 

  // Slope Sum Function (SSF): 
  int j = 0;               								//new counter
  float SSF = 0;           								//summation in window period
  for (int i = window+1; i < IR_array_size; i++)
  {
	for (int x = window; x >= 0; x--)
	{
	  SSF = 0;
	  float delta_input = (IR_AC_array[i-window]) - (IR_AC_array[(i-window)-1]);
	  if (delta_input > 0)
	  {
	    SSF += (delta_input);          
      }
	}
	IR_AC_array[j] = SSF;
	//Test print:
	// Serial.print(IR_AC_array[j]);
	// Serial.print(",");
	// Serial.println(j);
	// delay(15);
	j++;
  }
  //Test print:
  //Serial.println(j);

  // Filter signal (remove spikes in signal - by capping the signal)
  int Signal_cap = 1000;											// Cap signal can't go higher than 9.
  int count_cap_reached = 0;                // count the amount of times the signal cap has been reached
  for (int i = 0; i < j; i++)
  {
    if(IR_AC_array[i] >= Signal_cap)
    {
	  IR_AC_array[i] = Signal_cap;
      count_cap_reached++;
    }
  }
  // Check if a clean signal was recorded:
  if(count_cap_reached >= 1)
  {
    Prop_rec = false;
  }
  else 
  {
    Prop_rec = true;
  }

  //Identifies the highest peaks of the of the number of expected peaks.
  float Value_of_Peak = 0;        // Value of peak detected
  float Prev_value_of_peak = 0;     // Previous value of peak detected 
  float Sum_of_Peak = 0;                  // Sum of peak detected
  int Count_peaks = 0;          // Count peaks
  int Prev_i = 0;             // Previous i
  for (int i = 0; i < j; i++)
  {
    if(IR_AC_array[i-1] < IR_AC_array[i] && IR_AC_array[i] > IR_AC_array[i+1] && IR_AC_array[i] > 1 && IR_AC_array[i] != Signal_cap)
    { 
      Value_of_Peak = IR_AC_array[i];
      Sum_of_Peak += IR_AC_array[i];
      Count_peaks++;
      if (i < Prev_i+14)
      {
        if (Value_of_Peak > Prev_value_of_peak)
        {
          Sum_of_Peak -= Prev_value_of_peak;
        }
        else
        {
          Sum_of_Peak -= Value_of_Peak;
        }
        Count_peaks--;
      }
      Prev_i = i;
      Prev_value_of_peak = Value_of_Peak;
    }
  }
  //Calculating threshold
  float threshold = 0.8*(Sum_of_Peak/Count_peaks);  //threshold value for beat detection

  // Test print: 
  // for(int i = 0; i < j; i++)
  // {
  //   Serial.print(IR_AC_array[i]);
  //   Serial.print(",");
  //   Serial.println(threshold);
  //   //Serial.print(",");
  //   //Serial.println(i);
  //   delay(5);
  //  } 

  // Counting the peaks to calculate BPM, RR and HRV:
  int Peak_count = 0;                   // Counter to count the number of peaks
  int P2p_time_start = 0;               // Peak to peak start time
  int Sum_of_p2p_times = 0;             // sum of the times between peaks in the 5 second recording
  int Delta_p2p_time[110];              // Peak to peak delta time
  int Start_delta = 0;
  Prev_i = 0;	   						  // Previous i
  bool check = false;
  for(int i = 0; i < j; i++)
  {  
    if (IR_AC_array[i] > threshold)      											//Count peaks above threshold (beats) 
    {
      if(IR_AC_array[i-1] < IR_AC_array[i] && IR_AC_array[i] > IR_AC_array[i+1])    	//Peak detecting 
      {
        Peak_count++;
        if (i < Prev_i+14)
        {
      	  Peak_count--;
      	  check = true;
        }																//increment the peak counts 
        if (Peak_count == 1) 
        {
      	  Start_delta = micros();           										// start of the total recording time
        }                                           
        if (Peak_count > 1 && check == false)
        {
          Delta_p2p_time[Peak_count-2] = micros() - P2p_time_start;    			//delta time between peak to peak
          // Test print:
          //Serial.println(Delta_p2p_time[Peak_count-2]);
          Sum_of_p2p_times += Delta_p2p_time[Peak_count-2];
        }
	    check = false;
        P2p_time_start = micros();                       		 					//starting time of peak to peak
        Prev_i = i;
      }
    }
  }
  int End_delta = micros() - Start_delta;                 						// Delta time of the for loop

  // Calculating heart rate (HR):
  float refine_factor = 1;
  if (RR_HRV == true)
  {
    refine_factor = 1;												// Factor to refine BPM value
  }
  else
  {
    refine_factor = 1.05;												// Factor to refine BPM value
  }
  int Total_60s = End_delta*(60000000/(rec_time));                        // Taking the 5 seconds/ 30 seconds to 60 seconds
  int Avg_p2p_time = Sum_of_p2p_times/Peak_count;      							// Average peak to peak time in 5 second recording 
  float BPM = (Total_60s/Avg_p2p_time)*refine_factor;         					// Calculating the beats per minute
  BPM_val = BPM;

  // Calculating SpO2:
  float RMS_AC_IR = sqrt(Sum_AC_IR/50);                     						// RMS of the IR AC signal
  float RMS_AC_RED = sqrt(Sum_AC_RED/50);                   						// RMS of the RED AC signal
  float R = (RMS_AC_RED/RED_DC_val_SpO2)/(RMS_AC_IR/IR_DC_val_SpO2);    			// R value used to calculate Sp02
  float SpO2 = 110 - 25*R; 
  SpO2_val = SpO2;

  if (RR_HRV == true)
  {
    // Calculating respiratory rate (RR):
    int RR_count = 0;
    for ( int i = 1; i < Peak_count-2; i++)
    {
      // Test print:
      //Serial.println(Delta_p2p_time[i]);
      if(Delta_p2p_time[i-1] < Delta_p2p_time[i] && Delta_p2p_time[i] > Delta_p2p_time[i+1])
      {
        RR_count++;                                       	// count breaths
        // Test print:
        //Serial.println("-----");
      }
    }
    float RR = RR_count*2;                               	// RR breaths per minute (count 30's times 2)
    RR_val = RR;

    // Calculating heart rate variability (HRV):
    float HRV = 0;            								// Heart rate variablity score from 0 - 100
    int sum_of_HRV = 0;     								//sum of square peak to peak values for RMSSD calculation 
    for(int i = 1; i < Peak_count-1; i++) 
    {
      // Test print:
      //Serial.println(Delta_p2p_time[i]);
      sum_of_HRV += pow((Delta_p2p_time[i] - Delta_p2p_time[i-1]), 2);
      //Serial.println(sum_of_HRV);
    }
    // Test print:
    //Serial.println(sum_of_HRV);
    //Serial.println(Peak_count);
    HRV = sqrt(sum_of_HRV/(Peak_count-2));        			// RMSSD calculation to get HRV score value between 0-6.5
    Serial.println(HRV);
    //Serial.println(HRV_score_float);
    float HRV_score = HRV*15.385;                    		// ln(RMSSD0 value between 0-100
    //Serial.println(HRV_score);
    HRV_val = HRV_score;
  }
}

// MAX30105 sensor start up:
void MAX30105_Startup()
{
  //Serial.print("Initializing MAX30105..");
  // Initialize the sensor
  // Failures are generally due to an improper I2C wiring, missing power supply or wrong target chip
  // Initialize sensor
  if (!MAX30105_sensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }

  byte ledBrightness = 50;  //Options: 0=Off to 255=50mA
  byte sampleAverage = 1;   //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;         //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 200;    //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;     //Options: 69, 118, 215, 411
  int adcRange = 4096;      //Options: 2048, 4096, 8192, 16384

  MAX30105_sensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
	MAX30105_sensor.clearFIFO();                           //rest fifo register 
}

// DC Removeral filter for Heart Rate (HR)
float DCR_function_IR(double raw_input, float alpha, bool ret) 
{  
  float output_DCR = 0;
  float filtered = raw_input + alpha * prev_filtered_IR;
  float AC_val = filtered - prev_filtered_IR;             //AC value of signal
  float DC_val = filtered;                                //DC value of signal
  //Weather to return the ac or dc value:
  if (ret == true)
  {
    output_DCR = DC_val;
  } 
  else
  {
    prev_filtered_IR = filtered;
    output_DCR = AC_val;
  } 
  //Serial.println(output_DCR); 
  return output_DCR;
}

// DC Removeral filter for Sp02
float DCR_function_RED(double raw_input, float alpha, bool ret) 
{  
  float output_DCR = 0;
  float filtered = raw_input + alpha * prev_filtered_RED;
  float AC_val = filtered - prev_filtered_RED;            //AC value of signal
  float DC_val = filtered;                                //DC value of signal 
  //Weather to return the ac or dc value:
  if (ret == true)
  {
    output_DCR = DC_val;
  } 
  else
  {
    prev_filtered_RED = filtered; 
    output_DCR = AC_val;
  }
  //Serial.println(output_DCR); 
  return output_DCR;
}

// Mean difference filter
float MDF_function(float raw_input) 
{
  float avg = 0; 
  sum -= MeanDiff_TV[Index];
  MeanDiff_TV[Index] = raw_input;
  sum += MeanDiff_TV[Index];  
  Index++;
  Index = Index % SAMPLE_SIZE;
  if (count < SAMPLE_SIZE) 
  {
    count++;
  }
  avg = sum / count;
  //Serial.println(avg);
  return avg - raw_input;
}

// Low pass filter (Butterworth filter)
float Butterworth_LPF_function(float raw_input) 
{
  //Second order low pass filter
  Val[0] = Val[1];
  Val[1] = Val[2];
  //Fs = 100Hz (sample rate) and Fc = 10Hz (cut-off frequency)
  Val[2] = (6.745527388907189559e-2 * raw_input) + (-0.41280159809618854894 * Val[0]) + (1.14298050253990091107 * Val[1]);
  //Fs = 100Hz (sample rate) and Fc = 5Hz (cut-off frequency)
  //Val[2] = (2.008336556421122521e-2 * raw_input) + (-0.64135153805756306422 * Val[0]) + (1.56101807580071816339 * Val[1]);
  //Fs = 100Hz (sample rate) and Fc = 3Hz (cut-off frequency)
  //Val[2] = (7.820208033497201908e-3 * raw_input) + (-0.76600660094326400440 * Val[0]) + (1.73472576880927520371 * Val[1]);
  float BWF_output = (Val[0] + Val[2]) + 2*Val[1];
  //Serial.println(BWF_output);  
  return BWF_output;
}