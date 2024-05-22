/**
 * @file main.cpp
 * @brief Example program to demonstrate sensor data acquisition and activity classification
 *        with comfort data checking on STM32L475 IoT node.
 */

#include "mbed.h"
#include "stm32l475e_iot01_accelero.h"
#include "stm32l475e_iot01_gyro.h"
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_hsensor.h"
#include <cmath>
#include <cstdio>
#include "aes.h"

/**********************************************************************************
 #define DEBUG_SERIAL  //uncomment BELOW if serial DEBUGGING is necessary <<<<<
 #define ENCRYPTEDTEXT //uncomment BELOW if serial ENCRYPTEDTEXT is necessary <<<<<
 #define PLAINTEXT     //uncomment BELOW if serial PLAINTEXT is necessary <<<<<
***********************************************************************************/

#define BUFFER_SIZE 32
//#define PLAINTEXT
#define ENCRYPTEDTEXT

/** Serial communication setup */
BufferedSerial pc(USBTX, USBRX, 115200);


#ifdef ENCRYPTEDTEXT
    mbedtls_aes_context aes;
#endif

// Define encryption key
uint8_t key[] = 
{
    0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
    0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c,
};

/** Sample count for movement detection */
constexpr size_t SAMPLES_COUNT = 13;
/** Activity threshold for detecting movement */
constexpr float ACTIVITY_THRESHOLD = 910.0f;
/** Idle threshold for detecting inactivity */
constexpr float IDLE_THRESHOLD = 770.0f;
/** Time threshold for detecting sleeping state, in milliseconds */
constexpr uint32_t SLEEPING_TIME_THRESHOLD = 30 * 60 * 1000; // 30 minutes
/** Time threshold for detecting idle state, in milliseconds */
constexpr uint32_t IDLE_TIME_THRESHOLD = 40 * 1000; // 40 seconds
/** Minimum comfortable temperature */
constexpr float kComfortTempMin = 20.0f;
/** Maximum comfortable temperature */
constexpr float kComfortTempMax = 25.0f;
/** Minimum comfortable humidity percentage */
constexpr float kComfortHumMin = 40.0f;
/** Maximum comfortable humidity percentage */
constexpr float kComfortHumMax = 70.0f;
/** Cooldown time after running detection, in milliseconds */
constexpr uint32_t RUNNING_COOLDOWN_TIME_MS = 3000;
/** Cooldown time after walking or fall detection, in milliseconds */
constexpr uint32_t WALK_FALL_COOLDOWN_TIME_MS = 3000;


/** Array to store magnitudes for movement detection */
float magnitudes[SAMPLES_COUNT];
/** Current index in the magnitudes array */
size_t currentIndex = 0;
/** Timer for idle detection */
uint32_t idleTime = 0;
/** Last time RUNNING was detected */
static uint32_t lastRunningTime = 0;
/** Last time any non-idle activity was detected */
static uint32_t lastActiveTime = 0;
/** Last time WALKING or FALL DETECTED was detected */
static uint32_t lastWalkFallTime = 0;

// Global Variables for Cooldown Management.
/* Timestamp of the last activity change */
uint32_t lastActivityChangeTime = 0; 

// General cooldown period for any state change (5 seconds)
const uint32_t activityChangeCooldownMs = 3000; 

/* 5 seconds cooldown before switching to walking*/
const uint32_t postRunningToWalkingCooldownMs = 5000; 
/* 55 seconds cooldown before switching to idle */
const uint32_t postRunningToIdleCooldownMs = 5000; 

/** Ticker for sensor data acquisition */
Ticker sensorTicker;
/** Timer for event duration measurement */
Timer timer;
/** Buffer to store the last detected event */
char lastEvent[BUFFER_SIZE] = "";


/**
 * @brief Sends formatted text data over the serial connection in plaintext.
 *
 * This function works similarly to the standard `printf` function, formatting the given
 * string and arguments into a single string and then sending it over the serial connection.
 * It's designed for use when plaintext communication is preferred or required. The function
 * formats the input string and arguments into a buffer, which is then written to the serial
 * port specified by the `pc` global variable.
 *
 * @param format A format string that specifies how subsequent arguments are converted for output,
 *               similar to the first parameter of the standard `printf` function.
 * @param ... A variable number of arguments to be formatted according to the format string.
 *
 * @note This function is only compiled and available when `PLAINTEXT` is defined.
 * @note The buffer used for formatting the output is statically allocated with a fixed size.
 *       Ensure that the formatted string does not exceed this size to avoid truncation.
 * @note It relies on the `BufferedSerial` object `pc` being properly initialized and configured
 *       for serial communication.
 *
 * @pre `pc` must be initialized and configured for serial communication before calling this function.
 *
 * @see printf (for formatting options and syntax)
 * @see BufferedSerial::write (for details on the underlying write operation)
 */

#ifdef PLAINTEXT
    void output(const char* format, ...) 
    {
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        pc.write(buffer, strlen(buffer));
    }
#endif

/**
 * @brief Encrypts and sends data over the serial connection.
 *
 * This function takes a buffer of data and its size, encrypts the data using AES encryption
 * with a predefined key, and then sends the encrypted data over a serial connection. The function
 * handles the encryption process in Electronic Codebook (ECB) mode block by block, ensuring that
 * the data is properly padded to meet AES block size requirements.
 *
 * @param data Pointer to the buffer containing the data to be encrypted and sent.
 * @param dataSize The size of the data in the buffer, in bytes.
 *
 * @note This function is only available when `ENCRYPTEDTEXT` is defined.
 * @note It assumes that the `mbedtls_aes_context` and the encryption key (`key`) have been
 *       properly initialized elsewhere in the code.
 * @note The function uses a statically allocated buffer for the encrypted data. Ensure
 *       that this buffer is large enough for the encrypted data plus padding.
 * @note The function utilizes Mbed TLS's AES functionality for encryption. Ensure that
 *       Mbed TLS library is included and configured in the project.
 *
 * @pre The caller must ensure that `data` points to a valid buffer of at least `dataSize` bytes.
 * @pre `mbedtls_aes_context` must be properly initialized with a valid encryption key before
 *      calling this function.
 *
 * @see mbedtls_aes_context
 * @see mbedtls_aes_init()
 * @see mbedtls_aes_setkey_enc()
 * @see mbedtls_aes_crypt_ecb()
 * @see mbedtls_aes_free()
 */

#ifdef ENCRYPTEDTEXT

    void output(const unsigned char *data, size_t dataSize) 
    {
        unsigned char encryptedBuffer[256]; // Ensure this is large enough for your data + padding

        // Assuming `key` has been properly defined and initialized elsewhere
        mbedtls_aes_context aes;
        mbedtls_aes_init(&aes);
        mbedtls_aes_setkey_enc(&aes, key, 128); // Set the encryption key

        // Calculate padded length for AES encryption (block size = 16 bytes)
        size_t paddedLength = ((dataSize + 15) / 16) * 16;
        unsigned char paddedData[256]; // Ensure this buffer is large enough
        memcpy(paddedData, data, dataSize); // Copy original data
        memset(paddedData + dataSize, 0, paddedLength - dataSize); // Apply padding

        // Encrypt the data block by block
        for (size_t i = 0; i < paddedLength; i += 16) 
        {
            mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, paddedData + i, encryptedBuffer + i);
        }

        mbedtls_aes_free(&aes); // Free AES context

        // Send the encrypted data over serial
        pc.write(encryptedBuffer, paddedLength);
    }
#endif


/**
 * Checks the comfort level based on temperature and humidity data from sensors
 * and sends this data along with the last detected event.
 */
#ifdef PLAINTEXT
 void checkAndSendComfortData() 
{
     float temperature = BSP_TSENSOR_ReadTemp();
     float humidity = BSP_HSENSOR_ReadHumidity();
     bool isComfortable = (temperature >= kComfortTempMin && temperature <= kComfortTempMax) &&
                          (humidity >= kComfortHumMin && humidity <= kComfortHumMax);

    output("%s,%.2f,%.2f,%d\r\n", lastEvent, temperature, humidity, isComfortable ? 1 : 0);

}
#endif

/**
 * @brief Checks the comfort level based on temperature and humidity data from sensors
 * and sends this data along with the last detected event in an encrypted format.
 * 
 * This function reads the current temperature and humidity from the onboard sensors,
 * determines whether the readings fall within predefined comfort ranges, and then
 * constructs a packet containing the last detected event character, temperature,
 * humidity, and comfort status. The packet is encrypted before being sent over
 * the serial connection.
 *
 * @note This function is conditional on ENCRYPTEDTEXT being defined. It requires
 * the mbedtls library for AES encryption.
 *
 * @pre Assumes `mbedtls_aes_context` has been initialized and set up with a key.
 * @pre Temperature and humidity sensors (TSENSOR and HSENSOR) must be initialized.
 *
 * @see mbedtls_aes_context
 * @see BSP_TSENSOR_ReadTemp()
 * @see BSP_HSENSOR_ReadHumidity()
 */

#ifdef ENCRYPTEDTEXT
    void checkAndSendComfortData() 
    {
        float temperature = BSP_TSENSOR_ReadTemp();
        float humidity = BSP_HSENSOR_ReadHumidity();
        bool isComfortable = (temperature >= kComfortTempMin && temperature <= kComfortTempMax) &&
                            (humidity >= kComfortHumMin && humidity <= kComfortHumMax);

        // Assuming the event is represented by the first character in currentEvent
        char eventChar = lastEvent[0]; // Or use currentEvent[0] if more appropriate

        // Convert floats to integers to maintain precision
        int16_t tempInt = (int16_t)(temperature * 100);
        int16_t humidInt = (int16_t)(humidity * 100);
        uint8_t comfortByte = isComfortable ? 1 : 0;

        // Pack the data, adjusted to include the event identifier
        uint8_t dataToSend[6]; // Adjusted size to include event identifier
        dataToSend[0] = eventChar; // Event identifier
        dataToSend[1] = (tempInt >> 8) & 0xFF; // Temperature high byte
        dataToSend[2] = tempInt & 0xFF;        // Temperature low byte
        dataToSend[3] = (humidInt >> 8) & 0xFF; // Humidity high byte
        dataToSend[4] = humidInt & 0xFF;        // Humidity low byte
        dataToSend[5] = comfortByte;            // Comfort status

        // Encrypt and send the packed data
        output(dataToSend, sizeof(dataToSend)); 
    }

#endif

/**
 * @brief Calculates the combined magnitude of acceleration and gyroscope data.
 * 
 * This function takes raw acceleration and gyroscope data as input, converts
 * gyroscope data from milli-degrees per second to degrees per second, and then
 * calculates the magnitude of acceleration and gyroscope data. The combined
 * magnitude is computed as the average of these two magnitudes. This value can
 * be used for movement detection or activity classification.
 * 
 * @param accData An array of three int16_t values representing the X, Y, and Z
 *                acceleration data from the accelerometer.
 * @param gyroData An array of three float values representing the X, Y, and Z
 *                 gyroscope data in milli-degrees per second. These values are
 *                 converted to degrees per second within the function.
 * 
 * @return The combined magnitude of acceleration and gyroscope data as a float.
 * 
 * @note The function uses `sqrt` from `<cmath>` for magnitude calculations..
 */

float calculateMagnitude(int16_t accData[3], float gyroData[3]) 
{
    // Convert gyroscope data from milli-degrees per second to degrees per second
    for (int i = 0; i < 3; ++i) 
    {
        gyroData[i] /= 1000.0f;
    }

    // Calculate the magnitudes of acceleration and gyroscope data 6  >>  2 
    float accMagnitude = sqrt(accData[0] * accData[0] + accData[1] * accData[1] + accData[2] * accData[2]);
    float gyroMagnitude = sqrt(gyroData[0] * gyroData[0] + gyroData[1] * gyroData[1] + gyroData[2] * gyroData[2]);

    //Calculate the combined magnitude
    float combinedMagnitude = (accMagnitude + gyroMagnitude) / 2.0f;

    //Print the combined magnitude for real-time observation
    //output("Acc Magnitude: %.2f, Gyro Magnitude: %.2f, Combined Magnitude: %.2f\n", accMagnitude, gyroMagnitude, combinedMagnitude);
   
    return combinedMagnitude;
}


/**
 * @brief Classifies the current movement based on sensor data magnitudes.
 * 
 * This function assesses movement by analyzing the magnitudes of sensor data stored
 * in a global array. It determines the current activity state (e.g., WALKING, RUNNING,
 * FALL DETECTED, IDLE, SLEEPING) based on predefined thresholds and implements cooldown
 * periods to prevent rapid state changes. It updates the `lastEvent` variable with the
 * current state and resets data collection for the next cycle. Optionally, for debugging,
 * it prints the current state along with comfort data (temperature and humidity) to the
 * serial port.
 * 
 * The function utilizes global variables for tracking the last detection time of different
 * activity states to implement cooldown mechanisms, ensuring stability in activity classification.
 */

void classifyMovement() 
{
    uint32_t currentTime = Kernel::get_ms_count();

    // Dynamic cooldown adjustment based on the last activity
    uint32_t dynamicCooldown = RUNNING_COOLDOWN_TIME_MS; // Default to running cooldown
    if (strcmp(lastEvent, "R") == 0) { // If the last event was Running
        if (currentTime - lastRunningTime < postRunningToIdleCooldownMs) 
        {
            dynamicCooldown = postRunningToIdleCooldownMs; // Apply faster cooldown for transition to idle
        }
    }

    // Apply dynamic cooldown for transitions from RUNNING
    if (currentTime - lastRunningTime < dynamicCooldown && lastRunningTime != 0) 
    {
        currentIndex = 0;
        return;
    }

    // Original cooldown checks for WALKING/FALL DETECTED remain unchanged
    if (currentTime - lastWalkFallTime < WALK_FALL_COOLDOWN_TIME_MS && lastWalkFallTime != 0) 
    {
        currentIndex = 0;
        return;
    }

    size_t aboveThreshold = 0;
    for (size_t i = 0; i < SAMPLES_COUNT; i++) 
    {
        if (magnitudes[i] > ACTIVITY_THRESHOLD) 
        {
            aboveThreshold++;
        }
    }

    char currentEvent[BUFFER_SIZE] = "";

    // Classify the current movement
    if (aboveThreshold >= 1 && aboveThreshold <= 2) 
    {
        strcpy(currentEvent, "F");
        lastWalkFallTime = currentTime;
    } 
    else if (aboveThreshold >= 3 && aboveThreshold <= 7) 
    {
        strcpy(currentEvent, "W");
        lastWalkFallTime = currentTime;
    } 
    else if (aboveThreshold > 7) 
    {
        strcpy(currentEvent, "R");
        lastRunningTime = currentTime;
    } 
    else 
    {
        // Determine IDLE or SLEEPING based on elapsed time
        if (currentTime - lastWalkFallTime > IDLE_TIME_THRESHOLD &&
            currentTime - lastWalkFallTime < SLEEPING_TIME_THRESHOLD) 
            {
            strcpy(currentEvent, "I");
        } 
        else if (currentTime - lastWalkFallTime >= SLEEPING_TIME_THRESHOLD) 
        {
            strcpy(currentEvent, "S");
        }
    }

    // Update and output the current event if changed
    if ((strcmp(currentEvent, lastEvent) != 0 && strlen(currentEvent) > 0) ||
        (timer.elapsed_time() >= 60s)) 
        {
        #ifdef PLAINTEXT
            output("%s\r\n", currentEvent);
        #endif
        strcpy(lastEvent, currentEvent);
    }

    // Prepare for the next data collection cycle
    currentIndex = 0;
    memset(magnitudes, 0, sizeof(magnitudes));
}


/**
 * @brief Processes sensor data from the accelerometer and gyroscope.
 * 
 * Collects accelerometer and gyroscope data, computes their combined magnitude, and stores it in a global array.
 * Once a sufficient number of samples have been collected, it triggers movement classification and resets for
 * the next data collection cycle.
 */
void processSensorData() 
{
    static int sampleCounter = 0; // Tracks the number of samples collected

    int16_t accData[3] = {0}; // Accelerometer data
    float gyroData[3] = {0}; // Gyroscope data
    BSP_ACCELERO_AccGetXYZ(accData); // Get accelerometer data
    BSP_GYRO_GetXYZ(gyroData); // Get gyroscope data

    float magnitude = calculateMagnitude(accData, gyroData); // Calculate combined magnitude

    // Store the magnitude and prepare for the next sample
    magnitudes[currentIndex] = magnitude;
    currentIndex = (currentIndex + 1) % SAMPLES_COUNT;
    sampleCounter++;

    // If enough samples have been collected, classify the movement
    if (sampleCounter == SAMPLES_COUNT) 
    {
        classifyMovement(); // Classify movement based on collected data
        sampleCounter = 0; // Reset the sample counter for the next cycle
        memset(magnitudes, 0, sizeof(magnitudes)); // Clear the magnitudes array
    }
}

/**
 * @brief The main function initializes sensors, starts data collection and processing, and checks for comfort data.
 * 
 * Initializes the accelerometer and gyroscope sensors, then enters a loop that periodically collects sensor data
 * for movement classification and checks for comfort data every minute.
 */

int main() 
{
    #ifdef ENCRYPTEDTEXT
        mbedtls_aes_init(&aes);
        mbedtls_aes_setkey_enc(&aes, key, 128); // 128-bit key
    #endif

    #ifdef PLAINTEXT
        output("Initializing Sensors...\n");
    #endif

    BSP_ACCELERO_Init(); // Initialize accelerometer
    BSP_GYRO_Init(); // Initialize gyroscope
    BSP_TSENSOR_Init(); //Initialize temperature
    BSP_HSENSOR_Init(); //Init Humidity

    sensorTicker.attach(&processSensorData, 81ms); // Attach processSensorData function to ticker for periodic execution
    timer.start(); // Start the timer

    while (true) 
    {
        // Periodically check and send comfort data
        if (timer.elapsed_time() >= 20s) 
        {
            checkAndSendComfortData(); // Check comfort level based on sensor data
            timer.reset(); // Reset timer for the next period
        }
        ThisThread::sleep_for(500ms); // Reduce CPU usage
    }

    #ifdef ENCRYPTEDTEXT
        mbedtls_aes_free(&aes); // Cleanup AES context
    #endif
}

