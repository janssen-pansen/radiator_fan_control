/**
 * Radiator fan control based on measured temperature.
 * 
 * Measures temperature on radiator sensor and ambient sensor. Activates fan as follows:
 * -Early temperature increase detection based on regression fit;
 * -sustain fan activation when temperature is above threshold (but not necessarily increasing);
 * -switching between active/non-active not possible during cooldown timer (prevents pendling).
 * 
 * Once active, fan speed is modulated using radiator temperature as a ratio between maximum and ambient temperatures.
 */

// available from standard library
#include <OneWire.h> // one-wired sensors
#include <DallasTemperature.h> // DS18B20U temp sensor
#include <QList.h> // linked lists

// copy these from git into local sketchbook path
#include "PWM.h" // PWN fan control
#include "LinearRegression.h" // linear regression
#include "MemoryFree.h" // memory monitoring

// general settings
const bool FRONT_RADIATOR = true; // set to true for front or false for rear radiator
const float TEST_RATIO = -1; // set to x<0 to disable test mode, otherwise 0<=g_test_duty_cycle<=1

// arduino settings
const byte SENSOR_PIN = 2; // temperature sensor pin
const byte GATE_PIN = 8; // mosfet pin
const byte PWM_PIN = 9; // fan modulation pin

// temperature and sampling settings
const byte TIMESTEP_MAIN = 3; // seconds between main loop iterations
const byte TIMESTEP_UPDATE = 15; // seconds between updates TODO 15
const float N_MINUTES_REGRESSION = 0.5; // length of temperature sample in minutes
const float N_MINUTES_AVERAGING = 6; // length of temperature sample in minutes
const float TEMP_THRESHOLD_PER_MINUTE = 1; // minimum temperature increase per minute to activate fan 
const float DEACTIVATION_RATIO = 1.25; // stopping temp as ratio between ambient and max temp
const float TEMP_MIN = 13; // mininum expected temperature
const float TEMP_MAX = 50 - 3; // mininum expected temperature

// these are automatically calculated/instantiated
const float REAL_MIN_DUTYCYCLE = FRONT_RADIATOR ? 0.35: 0.4; // minimum and maximum fan duty cycles
const float REAL_MAX_DUTYCYCLE = FRONT_RADIATOR ? 0.65 : 0.75; // minimum and maximum fan duty cycles
const DallasTemperature *g_sensors; // temperature sensor objects
bool g_is_init_success; // did init succeed?
float g_temp_threshold_per_sample; // global just for one-time print

struct Printer
{
/**
 * Prints measurements in csv-format
 */
  byte status_;
  byte dutycycle;
  byte cooldown;
  unsigned long start_time = millis();
  float temp_radiator;
  float temp_ambient;
  float temp_slope;
  float temp_ratio;
  float temp_stop;
  float temp_rad_average;
  float temp_amb_average;

  static void printDowncasted(float value)
  /**
   * Print as int if there is no loss in precision, else print as float
   */
  {
    if(floorf(value) == value)
    {
      Serial.print(int(value));
    }
    else
    {
      Serial.print(value);
    }
  }

  static void printNice(const String& label, float value)
  {
    /*
     * Print a label and float on a single line
     */
    Serial.print(label);
    Serial.print("=");
    Printer::printDowncasted(value);
    Serial.println();
  }

  static void printHeader()
  {
    /*
     * Print header line
     */
    Serial.println("\nsecondsElapsed,mem,status,cooldown,tempRadiator,tempAmbient,tempSlope,tempRatio,tempStop,tempRadAverage,tempAmbAverage,dutyCycle");
  }

  void printValues()
  {
    /*
     * Print set values
     */    
    // calculate elapsed time
    unsigned long current_time = millis();
    unsigned long elapsed_time = current_time - start_time;
    int seconds_elapsed = elapsed_time / 1000;
    int free_memory = freeMemory();

    // print variables
    float values[] = {seconds_elapsed, free_memory, status_, cooldown, temp_radiator, temp_ambient, temp_slope, 
                      temp_ratio, temp_stop, temp_rad_average, temp_amb_average, dutycycle};
    int n_items = sizeof(values)/sizeof(values[0]);
    for(int i = 0; i < n_items; i++)
    {
      Printer::printDowncasted(values[i]);
      if(i < n_items - 1)
      {
        Serial.print(", ");
      }
      else
      {
        Serial.println();
      }
    }
  }
};

class Util
{
  /*
   * Just some utility functions grouped together
   */ 
  public:
    static void setTemperatureToArguments(float& temp_radiator, float& temp_ambient)
    {
      /**
       * Obtain temperature measurements and set to arguments
       */       
      g_sensors->requestTemperatures();
      temp_radiator = g_sensors->getTempCByIndex((int)FRONT_RADIATOR);
      temp_ambient = g_sensors->getTempCByIndex((int)!FRONT_RADIATOR);
    }
    
    static float scale(float value, float mininum, float maximum)
    {
      /**
       * Scale (and possibly truncate) value into range 0-1
       */       
      float standardized = (value - mininum) / (maximum - mininum);
      standardized = standardized < 0 ? 0 : standardized > 1 ? 1 : standardized;
      return standardized;
    }
};

class History 
{
  /*
   * History class tracks temperature history and provides regression and averaging methods
   */  
  private:
    class compressedValueList 
    {
      /**
       * A list that internally represents floats as bytes (requires less memory)
       */
       
      private:
        QList<byte> values;

      public:        
        void push_front(float temperature) 
        {
          /*
           * Compress temperature into range min-max temp and encode represent as 8-bit value
           */
          float standardized = Util::scale(temperature, TEMP_MIN, TEMP_MAX);
          byte compressed = (byte) round(standardized * UINT8_MAX);
          this->values.push_front(compressed);
        }
    
        float get(int index) 
        {
          /*
           * Decompress 8-bit temperature representation into floating-point value
           */
          byte compressed = this->values.get(index);
          float standardized = (float(compressed) / UINT8_MAX);
          float decompressed = (TEMP_MAX - TEMP_MIN) * standardized + TEMP_MIN;
          return decompressed;
        }

        void pop_back()
        {
          this->values.pop_back();
        }

        int size()
        {
          return this->values.size();
        }
        
    };

    float calculateAverage(const compressedValueList& temperatures)    
    {
      /*
       * Calculate average on radiator temperature
       */
      float sum = 0;

      for(int i = 0; i < temperatures.size(); i++)
      {
        sum += temperatures.get(i);
      }

      return sum / temperatures.size();
    }

    compressedValueList radiator_temps;
    compressedValueList ambient_temps;
    
  public:
    History(int n_radiator_samples, int n_ambient_samples)
    {
      /**
       * Construct history object
       * - n_radiator_samples is number radiator samples stored
       * - n_ambient_samples is number of ambient samples stored
       * - At init, history is assumed to be the current temperature for the specified number of samples
       */
      // get current temperature
      float temp_radiator;
      float temp_ambient;
      Util::setTemperatureToArguments(temp_radiator, temp_ambient);

      // internal lists initialize to current temp for specified length
      for(int i = 0; i < n_radiator_samples; i++)
      {
        this->radiator_temps.push_front(temp_radiator);
      }

      for(int i = 0; i < n_ambient_samples; i++)
      {
        this->ambient_temps.push_front(temp_ambient);
      }
    }

    void recordTemp(float temp_radiator, float temp_ambient)
    {
      /**
       * Add current temperature values to history
       */       
      // containers are FIFO; keep at fixed length
      radiator_temps.pop_back();
      ambient_temps.pop_back();

      // scale current temperature values into 8-bit precision and push into container
      this->radiator_temps.push_front(temp_radiator);
      this->ambient_temps.push_front(temp_ambient);
    }

    float calculateAverageAmbient() 
    {
      return this->calculateAverage(this->ambient_temps);
    }

    float calculateAverageRadiator() 
    {
      return this->calculateAverage(this->radiator_temps);
    }

    float calculateSlopeDiff()    
    {
      /*
       * - Calculate slope on temperature difference
       */
      LinearRegression regression_model = LinearRegression(0, radiator_temps.size());

      for(int i = 0; i < radiator_temps.size(); i++)
      {
        int reverse_index = radiator_temps.size() - i - 1;
        float temp_radiator = radiator_temps.get(reverse_index);
        regression_model.learn(i, temp_radiator);
      }

      double regression_values[] = {};
      regression_model.getValues(regression_values);
      return regression_values[0]; //slope is 0, intercept is 1
    }
};

// declare history and printer objects
const History *g_history; // provides temperature regression functionality
Printer *g_printer; // provides easy printing functionality

void setup() 
{
  /*
   * Run this once at initialization
   */  
  // start serial port for debugging
  Serial.begin(9600);
  
  // set pin controlling fan mosfet power gate to output mode
  pinMode(GATE_PIN, OUTPUT);

  // initialize temperature sensor
  g_sensors = new DallasTemperature(new OneWire(SENSOR_PIN));
  g_sensors->begin(); 

  // set fan pwm frequency
  InitTimersSafe();
  g_is_init_success = SetPinFrequencySafe(PWM_PIN, 25000);

  // convert temp slope threshold per minute to threshold per sample
  float samples_per_minute = 60. / TIMESTEP_MAIN;
  g_temp_threshold_per_sample = TEMP_THRESHOLD_PER_MINUTE / samples_per_minute;

  // instantiate custom classes
  // TODO
  int n_radiator_samples = samples_per_minute * N_MINUTES_REGRESSION;
  int n_ambient_samples = samples_per_minute * N_MINUTES_AVERAGING;
  g_history = new History(n_radiator_samples, n_ambient_samples);
  g_printer = new Printer();

  // some values to print at init
  Printer::printNice("FRONT_RADIATOR",FRONT_RADIATOR);
  Printer::printNice("TEMP_MIN",TEMP_MIN);
  Printer::printNice("TEMP_MAX",TEMP_MAX);
  Printer::printNice("TIMESTEP_MAIN",TIMESTEP_MAIN);
  Printer::printNice("TIMESTEP_UPDATE",TIMESTEP_UPDATE);
  Printer::printNice("DEACTIVATION_RATIO",DEACTIVATION_RATIO);
  Printer::printNice("REAL_MIN_DUTYCYCLE",REAL_MIN_DUTYCYCLE);
  Printer::printNice("REAL_MAX_DUTYCYCLE",REAL_MAX_DUTYCYCLE);
  Printer::printNice("TEMP_THRESHOLD_PER_MINUTE",TEMP_THRESHOLD_PER_MINUTE);
  Printer::printNice("\nsamples_per_minute", samples_per_minute);
  Printer::printNice("g_temp_threshold_per_sample", g_temp_threshold_per_sample);
  Printer::printNice("cooldown", UINT8_MAX / (60. / TIMESTEP_UPDATE));
  Printer::printNice("N_MINUTES_AVERAGING", N_MINUTES_AVERAGING);
  Printer::printNice("N_MINUTES_REGRESSION", N_MINUTES_REGRESSION);
  Printer::printNice("n_radiator_samples", n_radiator_samples);
  Printer::printNice("n_ambient_samples", n_ambient_samples);
  Printer::printHeader();
}

byte calculateDutyCycle(float temp_ambient, float temp_radiator) 
{
  /*
   * Calculate duty cycle based on radiator temperature relative to ambient and max temp
   */    
  // calculate temp ratio relative to ambient and max temp
  float temp_ratio = Util::scale(temp_radiator, temp_ambient, TEMP_MAX);
  g_printer->temp_ratio = temp_ratio;
  
  // convert temp ratio into duty cycle value
  byte min_dutycycle = round(REAL_MIN_DUTYCYCLE * UINT8_MAX);
  byte dutycycle_range = round(REAL_MAX_DUTYCYCLE * UINT8_MAX) - min_dutycycle;
  byte dutycycle = round(temp_ratio * dutycycle_range + min_dutycycle); 
  return dutycycle;
}

byte set_fan(float temp_radiator, float temp_ambient, byte cooldown_timer_)
{
  /*
   * Determine whether to enable (and modulate) or disable fan. Sets fan PWM signal.
   */  
  // duty cycle var and status should have persist globally
  static byte dutycycle;
  static byte status_; // 1: hot start; 2: idle; 3: cold start; 4: active; 5: disable; 6: cooldown; 7: active
     
  // calculate avg and slope
  float temp_rad_average = g_history->calculateAverageRadiator();
  float temp_amb_average = g_history->calculateAverageAmbient();
  float temp_slope = g_history->calculateSlopeDiff();
  g_printer->temp_rad_average = temp_rad_average;
  g_printer->temp_amb_average = temp_amb_average;
  g_printer->temp_slope = temp_slope;

  // detertime threshold condtionals
  // TODO
  bool fan_is_enabled = dutycycle != 0;
  bool temp_is_increasing = temp_slope >= g_temp_threshold_per_sample;
  float temp_stop = temp_amb_average * DEACTIVATION_RATIO;
  bool temp_is_high = temp_rad_average > temp_stop;
  bool cooldown_is_active = cooldown_timer_ > 0;
  g_printer->temp_stop = temp_stop; 

  if(!fan_is_enabled)
  {
    if(temp_is_increasing || temp_is_high)
    {
        digitalWrite(GATE_PIN, HIGH);
        dutycycle = UINT8_MAX;
        status_ = temp_is_increasing ? 3 : 1;
    }
    else
    {
      status_ = 2;
    }
  }
  else
  {
    if(temp_is_increasing || temp_is_high || cooldown_is_active)
    {
      dutycycle = calculateDutyCycle(temp_ambient, temp_radiator);
      status_ = temp_is_increasing ? 7 : temp_is_high ? 4 : 6;      
    }
    else
    {
      digitalWrite(GATE_PIN, LOW);
      dutycycle = 0;
      status_ = 5;
    }
  }
  
  // set PWM signal, write to printer, and return status
  pwmWrite(PWM_PIN, dutycycle);
  g_printer->dutycycle = dutycycle;
  g_printer->status_ = status_;
  return status_;
}

void loop() 
{
  /*
   * Repeat this over and over
   */
  // if not in test mode...
  if(TEST_RATIO < 0) 
  {    
    // if init was sucessfull...
    if(g_is_init_success)
    {
      // these variables have global lifetime (but are still scoped within this function)
      static byte counter; // used to determine at what interval to update (see TIMESTEP_UPDATE)
      static byte cooldown_timer; // fan is guaranteed to be (de)activated for 255 iterations of TIMESTEP_UPDATE
      g_printer->cooldown = cooldown_timer;
      
      // request and log temperatures
      float temp_radiator;
      float temp_ambient;
      Util::setTemperatureToArguments(temp_radiator, temp_ambient);
      g_history->recordTemp(temp_radiator, temp_ambient);
      g_printer->temp_radiator = temp_radiator;
      g_printer->temp_ambient = temp_ambient;

      byte status_; // 1: hot start; 2: idle; 3: cold start; 4: active; 5: disable; 6: cooldown; 7: active
      // only update fan every TIMESTEP_UPDATE seconds
      if(counter % (TIMESTEP_UPDATE / TIMESTEP_MAIN) == 0)
      {
        // determine fan activation mode
        status_ = set_fan(temp_radiator, temp_ambient, cooldown_timer);
        g_printer->printValues();
      
        // if fan is enabling/disabling or cooldown timer is counting, increment cooldown timer
        if(status_ == 1 || status_ == 3 || status_ == 5 || cooldown_timer > 0) 
        {
          cooldown_timer++;
        }
      }    
        
      counter++;
    }
    // if init unsucessfull
    else
    {
      Serial.println("failed!");
    }
  }
  // Test PWM activation (e.g., sound levels)
  else
  {
    Serial.println("test mode...");
    Serial.println(TEST_RATIO);
    byte test_dutycycle = round(TEST_RATIO * UINT8_MAX);
    Serial.println(test_dutycycle);
    digitalWrite(GATE_PIN, HIGH);
    pwmWrite(PWM_PIN, test_dutycycle);
  }

  // loop delay
  delay(TIMESTEP_MAIN * 1000);
}
