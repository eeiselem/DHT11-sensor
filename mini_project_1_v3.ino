//DHT11 sensor is connect to 3.3v, D4, and GND pins

//imports
#include "DHT.h"
#include <vector>
#include <numeric>
#include <algorithm>
#include <cmath>

// dynamic range of DHT11 is 0–50°C (±2°C accuracy) and 20–90% Relative Humidity (RH) (±5% RH accuracy)
// resolution of 1°C/1% RH. Bandwidth of 1 Hz

// configuration
constexpr uint8_t DHT_PIN = 4; // define pin D4 for data
constexpr uint8_t DHT_TYPE = DHT11; // define sensor type
constexpr uint8_t LED_PIN = 2; // LED pin D2 (on-board)
constexpr long SERIAL_BAUD = 115200; // changed to ESP32 standard

// timing settings
const unsigned long READING_TIME_INTERVAL = 2000; //DHT11 hardware limit is 1Hz, increased to 2 seconds to decrease DHT components generating heat
const unsigned long SERIAL_TIME_INTERVAL = 2000; //serial print interval in ms, matches reading time interval
const unsigned long LED_BLINK_TIME_INTERVAL = 1000; //LED blink interval in ms
const int CALIBRATION_SAMPLES = 10; // number of samples required to complete calibration

// Hystersis compensation
// How much the DHT11 physically "lags" behind the true temperature direction
const float TEMP_HYSTERESIS_FACTOR = 0.3; 
const float HUMID_HYSTERESIS_FACTOR = 1.0;

//Thresholds for temperature and humidity
float led_on_temperature = 30;
float led_on_humidity = 70;
float led_blink_temperature = 15;
float led_blink_humidity = 30;

float raw_temperature = 0; // Unfiltered hardware reading
float raw_humidity = 0; 
float last_known_ground_temp = 0; // user input ground truth/reference value, will use to apply offset
float last_known_ground_humid = 0; 
float temperature = 0; // after noise reduction/EMA filtering and offset application
float humidity = 0; // 
float temperature_f = 0; // in farenheit

// Measured against a digital thermometer on Feb 13th
const float FACTORY_TEMP_OFFSET = 0.5; 
const float FACTORY_HUMID_OFFSET = 0.0; 

// Start the global offsets with the factory bias instead of 0
float temp_offset = FACTORY_TEMP_OFFSET; 
float humid_offset = FACTORY_HUMID_OFFSET;

// Buffers for Calibration
std::vector<float> temp_buffer;
std::vector<float> humid_buffer;

bool plotter_mode = false; // Disables the text UI to stream CSV-formatted data for the Arduino Serial Plotter
bool monitoring_active = true; //monitoring active / inactive

// some boards have active high and some active low for LED
// this was added so more intuitive in program, can flip settings here
const int LED_ON = HIGH; 
const int LED_OFF = LOW; 

DHT dht(DHT_PIN, DHT_TYPE); //create sensor object

// for multi-point calibration
struct CalPoint {
  float raw;
  float truth;
};

std::vector<CalPoint> temp_cal_curve; // Stores all user-entered calibration points

// groups statistic metrics under one container
struct Stats {
  float mean;
  float stdDev;
};

// Enforces distinct system phases and prevents overlapping logic execution
enum SystemState {
  CALIBRATING,
  MONITORING
};
SystemState current_state = CALIBRATING; // start in calibration to build stable baseline before monitoring

Forward Declarations 
void handleSensors(unsigned long current_time);
void handleLED(unsigned long current_time);
void handleSerialOutput(unsigned long current_time);
void handleUserInput();
void print_output();
void process_input(String input);
void finalizeCalibration();

// Note: Pass-by-reference (&) is used for these vectors to prevent expensive 
// memory copying during high-frequency loop execution
float calculateDynamicThreshold(std::vector<float> &data); 
float removeOutliersAndAverage(std::vector<float> &data);

// setup runs once to tell hardware how to behave
void setup() {
  Serial.begin(SERIAL_BAUD); //init serial communication
  // Reduces the serial timeout to 10ms. This prevents readStringUntil() from blocking the main sensor loop 
  Serial.setTimeout(10); 

  dht.begin(); //init sensor and prep for readings
  pinMode(LED_PIN, OUTPUT); //activate LED pin
  digitalWrite(LED_PIN, LED_OFF); // start with light off
} 

void loop() {
  // Take current time once per loop to ensure synchronized execution across all functions
  unsigned long current_time = millis(); 

  handleSensors(current_time);
  handleSerialOutput(current_time);
  handleLED(current_time);
  handleUserInput();
}

void handleSensors(unsigned long current_time) {
  static unsigned long previous_reading_time = 0;

  // proceed only if the hardware interval has fully elapsed
  if (monitoring_active && (current_time - previous_reading_time >= READING_TIME_INTERVAL)) {
    previous_reading_time = current_time;
    
    // get raw data
    float raw_t = dht.readTemperature();
    float raw_h = dht.readHumidity();

    // Check for hardware failure
    if (isnan(raw_t) || isnan(raw_h)) return;  // Keep old values if read fails to prevent crashes

    if (current_state == CALIBRATING) {
      // Store new raw data in buffer
      temp_buffer.push_back(raw_t);
      humid_buffer.push_back(raw_h);

      // Transition to monitoring after acquiring a sufficient baseline (10 samples)
      if (temp_buffer.size() >= CALIBRATION_SAMPLES) {
        finalizeCalibration(); // Calculate Offsets
        current_state = MONITORING; // Switch Mode
      }
    } 

    else if (current_state == MONITORING) {
      // update for graphing
      raw_temperature = raw_t;
      raw_humidity = raw_h;

      // update rolling window buffers
      temp_buffer.push_back(raw_t);
      humid_buffer.push_back(raw_h);
      if (temp_buffer.size() > CALIBRATION_SAMPLES) {
        temp_buffer.erase(temp_buffer.begin());
        humid_buffer.erase(humid_buffer.begin());
      }
      // Calculate the Derivative (Are we heating up or cooling down?)
      float temp_trend = calculateTrend(temp_buffer);
      float humid_trend = calculateTrend(humid_buffer);

      float hysteresis_t = raw_t;
      float hysteresis_h = raw_h;

      // Apply Hysteresis Correction 
      // If the trend shows we are actively heating up (> 0.2 slope)
      if (temp_trend > 0.2) {
          // Sensor is lagging behind reality. Push it forward.
          hysteresis_t += TEMP_HYSTERESIS_FACTOR; 
      } 

      // If the trend shows we are actively cooling down (< -0.2 slope)
      else if (temp_trend < -0.2) {
          // Sensor is holding onto old heat. Pull it down.
          hysteresis_t -= TEMP_HYSTERESIS_FACTOR; 
      }

      if (humid_trend > 1.0) hysteresis_h += HUMID_HYSTERESIS_FACTOR;
      else if (humid_trend < -1.0) hysteresis_h -= HUMID_HYSTERESIS_FACTOR;

      // hardware calibration
      // Calculate proposed values by applying correction factors/offset input from reference sources
      // Multi-point for temp, flat offset for humid.
      float proposed_t = applyReferenceOffset(hysteresis_t);
      float proposed_h = hysteresis_h + humid_offset; 

      // Update the global temp_offset so the UI is accurate
      temp_offset = proposed_t - raw_t;

      // Use the flat global offset for humidity. Calibration curve not set up for humidity
      float proposed_h = raw_h + humid_offset;
    
      // Noise filtering
      temperature = applyNoiseFilter(proposed_t, temperature, temp_buffer, 0.15);
      humidity    = applyNoiseFilter(proposed_h, humidity, humid_buffer, 0.50);
      
      temperature_f = (temperature * 9.0 / 5.0) + 32.0;
    }
  }
}

// Hysteriss trend tracking - Calculates the slope of the data buffer to determine if the 
// physical environment is actively heating/cooling or just hovering at rest.
float calculateTrend(std::vector<float> &buffer) {
  if (buffer.size() < 2) return 0.0;
  
  // Slope = Newest Reading (back) - Oldest Reading (front)
  return buffer.back() - buffer.front();
}

// Calibration runs once when enough data is collected. Takes  and sets the offset.
void finalizeCalibration() {
  float avg_t = removeOutliersAndAverage(temp_buffer);
  float avg_h = removeOutliersAndAverage(humid_buffer);

  // average as baseline once calibration finished
  temperature = avg_t + temp_offset;
  humidity = avg_h + humid_offset;
}

// Z-score outlier removal - removes data points that are > 2 standard deviations away.
float removeOutliersAndAverage(std::vector<float> &data) {
  if (data.empty()) return 0;
  if (data.size() < 3) return calculateStats(data).mean; // Too small to filter outliers

  Stats stats = calculateStats(data);
  
  // If StdDev is 0 (all values identical), just return the mean
  if (stats.stdDev == 0) return stats.mean;

  float clean_sum = 0;
  int clean_count = 0;

  for (float val : data) {
    // Calculate Z-Score
    float z_score = abs(val - stats.mean) / stats.stdDev;

    // Keep points within 2 Standard deviations (95% of data)
    if (z_score <= 2.0) {
      clean_sum += val;
      clean_count++;
    }
  }

  if (clean_count == 0) return stats.mean; // Protect from divide by zero risk
  return clean_sum / clean_count;
}

// Calculates mean and standard deviation
Stats calculateStats(std::vector<float> &data) {
  Stats result = {0, 0};
  if (data.empty()) return result;

  //  Mean
  float sum = 0;
  for (float val : data) sum += val;
  result.mean = sum / data.size();

  // Variance
  float sumSqDev = 0;
  for (float val : data) {
    sumSqDev += pow(val - result.mean, 2);
  }
  float variance = sumSqDev / data.size();
  
  // Std Dev
  result.stdDev = sqrt(variance);
  
  return result;
}

// Multipoint reference offset
// Finds where the raw reading falls on the calibration curve and computes the exact offset.
float applyReferenceOffset(float raw_reading) {
  // If no points entered yet, trust the raw reading + global temp_offset if user hasn't established curve
  if (temp_cal_curve.empty()) return raw_reading + temp_offset;

  // If only 1 point entered, use it as a flat single-point offset
  if (temp_cal_curve.size() == 1) {
    float single_offset = temp_cal_curve[0].truth - temp_cal_curve[0].raw;
    return raw_reading + single_offset;
  }

  // extraploation if the reading is colder than our coldest data point
  if (raw_reading <= temp_cal_curve.front().raw) {
    float offset = temp_cal_curve.front().truth - temp_cal_curve.front().raw;
    return raw_reading + offset;
  }
  
  // extrapolation if the reading is hotter than our hottest data point
  if (raw_reading >= temp_cal_curve.back().raw) {
    float offset = temp_cal_curve.back().truth - temp_cal_curve.back().raw;
    return raw_reading + offset;
  }

  // Find the two points the reading falls between
  for (size_t i = 0; i < temp_cal_curve.size() - 1; i++) {
    if (raw_reading >= temp_cal_curve[i].raw && raw_reading <= temp_cal_curve[i+1].raw) {
      float x0 = temp_cal_curve[i].raw;   // Colder raw
      float y0 = temp_cal_curve[i].truth; // Colder truth
      float x1 = temp_cal_curve[i+1].raw; // Hotter raw
      float y1 = temp_cal_curve[i+1].truth; // Hotter truth

      // Calculate the exact point on the line between them
      return y0 + (raw_reading - x0) * ((y1 - y0) / (x1 - x0));
    }
  }
  return raw_reading + temp_offset;
}

// Calculates the dynamic noise threshold and blends the new reading with the 
// old reading to eliminate jitter while preserving real changes.
float applyNoiseFilter(float proposed_val, float current_filtered_val, std::vector<float> &buffer, float min_floor) {
  // Calculate the dynamic threshold based on recent noise
  float noise_threshold = calculateDynamicThreshold(buffer, min_floor);
  
  // Calculate the difference to determine if this a jump or just static
  float difference = abs(proposed_val - current_filtered_val);
  
  // EMA
  float alpha;
  if (difference > noise_threshold) alpha = 0.8;  // Real change: High trust, move quickly
  else alpha = 0.05; // Noise: Low trust, move slowly to smooth it out
  
  // Apply the Exponential Moving Average formula
  return (proposed_val * alpha) + (current_filtered_val * (1.0 - alpha));
}

// Sets the noise threshold dynamically 
float calculateDynamicThreshold(std::vector<float> &data, float min_floor) {
  Stats stats = calculateStats(data);

  // threshold must be between min_floor and 2.0 (heuristically chosen)
  return constrain(stats.stdDev * 2.0, min_floor, 2.0);
}

// prints output interval determined by SERIAL_TIME_INTERVAL variable/setting
void handleSerialOutput(unsigned long current_time) {
  static unsigned long serial_prev_time = 0;
  if (current_time - serial_prev_time >= SERIAL_TIME_INTERVAL) {
    serial_prev_time = current_time;
    print_output();
  }
}

// Generates output to print to console showing menu and sensor readings
void print_output() {
  if (plotter_mode) {
    // Print Temp Data
    Serial.print("Raw_Temp:"); Serial.print(raw_temperature); Serial.print(","); 
    Serial.print("Filtered_Temp:"); Serial.print(temperature); Serial.print(","); 
    
    // Print Humid Data
    Serial.print("Raw_Humid:"); Serial.print(raw_humidity); Serial.print(","); 
    Serial.print("Filtered_Humid:"); Serial.print(humidity);
  
    Serial.println(); 
    return;
  }
  
  // Status Banner
  if (current_state == CALIBRATING) {
    Serial.print("\n");
    Serial.println("CALIBRATING SENSOR...");
    Serial.print("Collecting Samples: ");
    Serial.print(temp_buffer.size());
    Serial.print(" / ");
    Serial.print(CALIBRATION_SAMPLES);
    Serial.print("\t");
    Serial.print((int)((float)temp_buffer.size() / CALIBRATION_SAMPLES * 100));
    Serial.println(" % complete.");
  }
  if (current_state == MONITORING) {
    Serial.print("\n\n\n\n\n"); //print empty lines to move old text up for cleaner output

    // print project info
    Serial.println("==============================================================================================");
    Serial.println("UMKC | COMP-SCI 5577 Internet of Things | Mini Project 1 | Sean Gupta & Elliott Eisele-Miller");
    Serial.println("==============================================================================================");

    if (monitoring_active) {
      Serial.println("MONITORING ACTIVE");
    } else if (!monitoring_active){
      Serial.println("MONITORING INACTIVE");
    }

    //print menu and options
    Serial.println("============================================ MENU ============================================");
    Serial.println("Change value input format: \"menu_option_number:new_value\"");
    Serial.print("1: Set temperature upper (LED on) threshold    | Current value: "); Serial.print(led_on_temperature); Serial.println(" °C");
    Serial.print("2: Set humidity upper (LED on) threshold       | Current value: "); Serial.print(led_on_humidity); Serial.println(" %");
    Serial.print("3: Set temperature lower (LED blink) threshold | Current value: "); Serial.print(led_blink_temperature); Serial.println(" °C");
    Serial.print("4: Set humidity lower (LED blink) threshold    | Current value: "); Serial.print(led_blink_humidity); Serial.println(" %");
    Serial.print("5: Start / stop monitoring toggle              | Current value: "); Serial.println(monitoring_active ? "True" : "False");
    Serial.print("6: Calibrate Temp (Input Reference Temp)       | Raw value: "); Serial.print(raw_temperature); Serial.println(" °C");
    Serial.print("7: Calibrate Humidity (Input Reference Humid)  | Raw value: "); Serial.print(raw_humidity); Serial.println(" %");
    Serial.println("8: Toggle Graph Mode (With Serial Plotter) with '8:1'");
    Serial.println("============================================ Data ============================================");
    //check if monitoring active and only print data if active
    if (monitoring_active) {
      Serial.print("Temp: "); Serial.print(temperature); Serial.print(" °C | "); Serial.print(temperature_f); Serial.print(" °F "); Serial.print("(Offset: "); Serial.print(temp_offset); Serial.print(" °C)"); Serial.print("\t");
      Serial.print("Humid: "); Serial.print(humidity); Serial.print(" % (Offset: "); Serial.print(humid_offset); Serial.println(" %)");
      // Serial.print("Noise Threshold: +/-"); Serial.print(temp_threshold); Serial.print(" C, +/-"); Serial.print(humid_threshold); Serial.println(" %");
    } else {
      Serial.println("Data Stream PAUSED.");
    }
  } 
}

void handleLED(unsigned long current_time) {
  // assume the LED should be OFF.
  int intended_state = LED_OFF;

  // Monitoring Check
  if (!monitoring_active) {
    // Do nothing (led_state is already OFF)
  }
  
  // Solid ON 
  else if (temperature > led_on_temperature || humidity > led_on_humidity) {
    intended_state = LED_ON;
  }

  // Blinking
  else if (temperature < led_blink_temperature || humidity < led_blink_humidity) {
    static unsigned long previous_blink_time = 0;
    static bool blink_memory = false; 

    // Check the timer
    if (current_time - previous_blink_time >= LED_BLINK_TIME_INTERVAL) {
      previous_blink_time = current_time;
      blink_memory = !blink_memory; // Toggle the memory
    }

    // Use memory to switch LED on/off
    if (blink_memory) intended_state = LED_ON;
    else intended_state = LED_OFF;
  }

  digitalWrite(LED_PIN, intended_state);
}

void handleUserInput() {
  if (Serial.available() > 0) {
    String inputLine = Serial.readStringUntil('\n');
    inputLine.trim();
    if (inputLine.length() > 0) {
      process_input(inputLine);
    }
  }
}

void process_input(String input) {
  int colonIndex = input.indexOf(':'); //find colon index in string

  //check if input matches expected format
  if (colonIndex != -1) {
    String menu_option_number = input.substring(0, colonIndex); //get option number before separation char
    String new_value = input.substring(colonIndex + 1); //get new val from after separation char

    //remove white space
    menu_option_number.trim(); 
    new_value.trim(); 

    //switch case to handle which option the user chose
    switch (menu_option_number.toInt()) {
      case 1: // upper temp (LED on)
        led_on_temperature = new_value.toFloat(); //update value from user input
        break;
      
      case 2: // upper humidity (LED on)
        led_on_humidity = new_value.toFloat(); //update value from user input
        break;

      case 3: // lower temp (LED blink)
        led_blink_temperature = new_value.toFloat(); //update value from user input
        break;

      case 4: //lower humidity (LED blink)
        led_blink_humidity = new_value.toFloat(); //update value from user input
        break;

      case 5: // set monitoring active/inactive
        new_value.toLowerCase(); //convert input to lower case

        //validate input with different input options to cover cases
        if(new_value == "true" || new_value == "t" || new_value == "1"){
          monitoring_active = true; //update monitoring to true (active)
        } else if(new_value == "false" || new_value == "f" || new_value == "0"){
          monitoring_active = false; //update monitoring to false (inactive)
        } else {
          //advise user of error with input
          Serial.print("Error: Invalid input \"");
          Serial.print(new_value);
          Serial.println("\". Use True/False or T/F or 1/0.");
        } 
        break;

      case 6: // multipoint calibration
      {
         last_known_ground_temp = new_value.toFloat();
         
         // Record the current state (Raw vs Truth)
         temp_cal_curve.push_back({raw_temperature, last_known_ground_temp});
         
         // Sort the vector by raw temperature (coldest to hottest)
         std::sort(temp_cal_curve.begin(), temp_cal_curve.end(), 
          [](const CalPoint& a, const CalPoint& b) {
             return a.raw < b.raw;
         });
         
         Serial.print("Recorded Calibration Point: Raw ");
         Serial.print(raw_temperature);
         Serial.print(" = Truth ");
         Serial.println(last_known_ground_temp);
         break;
      }

      case 7: // calibrate humid with reference
         last_known_ground_humid = new_value.toFloat();
         humid_offset = last_known_ground_humid - raw_humidity;
         humidity = last_known_ground_humid; 
         break;

      case 8: // TOGGLE GRAPH MODE
         plotter_mode = !plotter_mode; // Switch True/False
         break;

      default:
        Serial.println("ERROR: Invalid input"); //advice of error with input
        break;
    } 
  } 
} 
