System Implementation
Pin Configuration and Library setup



Our system uses a DHT11 sensor connected via three pins: power is supplied by the ESP32 3.3V output, ground is tied to the ESP32 GND pin, and the data line is connected to GPIO pin D4. We use the standard DHT Sensor Library to interface with the hardware and extract raw temperature and humidity readings, which are subsequently converted from Celsius to Fahrenheit within the software pipeline.
Setup and Loop

The setup() function initializes serial communication, prepares the DHT11 sensor, and configures the onboard LED pin.

The main loop runs continuously, using non-blocking time intervals to manage the different phases of the program without freezing the processor. It captures the current hardware time for different interval execution of functions in the program. The handleSensors() function performs a large part of the program as it dictates whether the system is in its initial baseline calibration phase or the active monitoring phase and along with helper functions, performs the task that consist within those modes, including a large amount of data processing. Following data processing, the loop triggers interval-based Serial Monitor outputs, updates the hardware LED state, and continuously polls for user input via the Serial interface.
Serial Output


The Serial Monitor acts as the primary user interface, dynamically updating based on the current system state (Calibration vs. Monitoring) or switching to a comma-separated value (CSV) stream when toggled into Plotter Mode for graphing.

During the active Monitoring state, the system displays current filtered (noise reduction and calibrated) readings, applied offsets, and an interactive menu with 8 options:
1 & 2: Set the upper temperature and humidity thresholds (Solid LED).
3 & 4: Set the lower temperature and humidity thresholds (Blinking LED).
5: Toggle active data monitoring (Pause/Resume data stream).
6 & 7: Input reference values for temperature and humidity to correct hardware bias.
8: Toggle Graphing Mode (CSV output for the Arduino Serial Plotter).
User Interface




The handleUserInput() function captures incoming data the user inputs in response to the serial output, cleans it, and passes it to process_input(). The system expects a menu_option_number:new_value format. The colon acts as a delimiter to parse the command and the float/string parameter. Notably, Option 6 not only applies an immediate offset to the temperature data but also pushes the raw and reference values into a dynamically sorted array, to help create a linear interpolation curve to intelligently predict offsets across a range of temperatures.
LED Logic

The LED status is updated based on the current temperature and humidity readings compared to the threshold values. The LED can be on, blinking, or off. If monitoring is inactive, the LED is set to off. It is written to the built-in LED pin at the end of the function. Note that LED_ON is set to high or low as a setting in variable declarations as some ESP32 boards result in an on LED for LOW voltage and some an off LED for the same LOW voltage. Another ESP32 board was used during production of this code that had alternate configurations as the board that was given out in class.
Initial Calibration Algorithm



At start-up the system enters a calibration state to establish a stable baseline before moving on to the monitoring logic/stage. The system collects 10 raw samples into a buffer. The finalizeCalibration() function processes this buffer by calculating Z-scores; any reading exceeding 2 standard deviations is discarded as an outlier from hardware errors. The clean data is averaged, and a bias offset (determined during our testing to be +0.5°C for this specific DHT11 unit) is applied. A bias offset is not applied at calibration for humidity as we did not have an alternate, validated result to compare to.

Ongoing Calibration Algorithm


During the monitoring phase, within the handleSensors() function, an ongoing calibration is applied to the raw values. The applyReferenceOffset() function utilizes the historical reference measurements inputted by the user to extrapolate an offset for the current raw reading, dynamically compensating for sensor non-linearity.
Noise Algorithm

The noise filter function, applyNoiseFilter, is also called in the monitoring section of the handleSensors function as shown in previous images. After the reference calibration is applied the noise filtering applies an exponential moving average equation that changes depending on if the current reading differs beyond a dynamically determined threshold from the previous reading to reduce noise.

Following hardware calibration, the data passes through applyNoiseFilter(). This function utilizes an Exponential Moving Average (EMA). The EMA's tuning parameter (alpha) adapts dynamically. If the difference between the new reading and the recent baseline exceeds a noise threshold based on the recent results’ standard deviation, the filter reacts quickly to track the real physical change. If the difference is below the threshold, the filter heavily smooths the data to eliminate jitter.
Key Challenges
One key challenge in this project was handling the user input. Deciding on the best user input format took time and careful consideration. The user must enter an option and a new value. In some cases the value is processed as a float and in others it is processed as a string or boolean. The solution was to separate the two values by a pre-defined char and parse it in the code.

The ongoing calibration and noise filtering proved difficult as well. Balancing noise reduction with system responsiveness was significantly challenging. We had to consider a number of characteristics including time and memory efficiency and obviously how well the implemented changes would meet the desired behavior. Implementing the Z-score outlier removal and EMA filtering inherently introduces a delay/lag in the final data. We had to tune the dynamic thresholds, EM alpha values and other parameters to try to allow the system to remain responsive to true environmental changes without allowing high-frequency noise to influence the data.

Finally, the physical limitations of the DHT11 sensor presented accuracy hurdles. The dynamic range (0–50°C and 20–90% RH) was sufficient for our testing environment, but the factory accuracy tolerances (±2°C and ±5% RH) required us to build the multi-point reference calibration to minimize drift. Also we noticed that there is a discrepancy between the code’s/library's reported resolution and the hardware's actual capabilities. While the dht.readTemperature() function returns floating-point values to the tenth decimal place, the underlying hardware only possesses a resolution of 1°C / 1% RH, meaning the decimal outputs are likely interpolations rather than true high-resolution measurements.
