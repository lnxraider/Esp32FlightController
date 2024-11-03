// SC16IS752.h
#ifndef SC16IS752_H
#define SC16IS752_H

//=====================================================================
// Platform Detection and Includes
//=====================================================================

#ifndef ESP32
#define ESP32
#endif

#if defined(ARDUINO)
    #if defined(ESP32)
        #include <Arduino.h>
        #include <Wire.h>
        #define PLATFORM_ESP32
    #elif defined(ESP8266)
        #include <Arduino.h>
        #include <Wire.h>
        #define PLATFORM_ESP8266
    #else
        #include <Arduino.h>
        #include <Wire.h>
        #define PLATFORM_ARDUINO
    #endif
#else
    #error "Unsupported platform"
#endif

//=====================================================================
// Global Configuration Constants
//=====================================================================

#define I2C_DEFAULT_ADDR 0x4D

// Platform-specific defaults
#if defined(PLATFORM_ESP32)
    #define DEFAULT_SDA_PIN 23
    #define DEFAULT_SCL_PIN 21
    #define DEFAULT_I2C_FREQ 100000  // 100kHz for ESP32
#elif defined(PLATFORM_ESP8266)
    #define DEFAULT_SDA_PIN 4
    #define DEFAULT_SCL_PIN 5
    #define DEFAULT_I2C_FREQ 100000  // 100kHz for ESP8266
#else
    #define DEFAULT_I2C_FREQ 100000  // 100kHz for Arduino
#endif

class SC16IS752 {
public:
    //=====================================================================
    // Public Constants
    //=====================================================================

    // Timeouts and Limits
    static const unsigned long TX_TIMEOUT_MS = 1000;
    static const unsigned long RX_TIMEOUT_MS = 1000;
    static const uint16_t MAX_TRANSFER_SIZE = 256;
    static const uint8_t MAX_RETRIES = 5;
    static const bool DEBUG_ENABLED = true;

    // Error codes
    static const int OK = 0;
    static const int ERR_PARAM = -1;      // Invalid parameter
    static const int ERR_I2C = -2;        // I2C communication error
    static const int ERR_TIMEOUT = -3;    // Operation timeout
    static const int ERR_OVERFLOW = -4;   // Buffer overflow
    static const int ERR_VERIFY = -5;     // Verification failed
    static const int ERR_PLATFORM = -6;   // Platform-specific error
    static const int ERR_BUSY = -7;       // Device busy
    static const int ERR_FIFO = -8;       // FIFO error
    static const int ERR_STATE = -9;      // Invalid state

    // Error status bit masks
    static const uint8_t ERROR_OVERRUN = 0x01;
    static const uint8_t ERROR_PARITY = 0x02;
    static const uint8_t ERROR_FRAMING = 0x04;
    static const uint8_t ERROR_BREAK = 0x08;
    static const uint8_t ERROR_FIFO = 0x10;

    // Channel selection
    static const uint8_t CHANNEL_A = 0;
    static const uint8_t CHANNEL_B = 1;
    static const uint8_t MAX_CHANNELS = 2;

    //=====================================================================
    // Register Definitions
    //=====================================================================

    // UART Registers
    static const uint8_t REG_RHR = 0x00;       // Receive Holding Register (Read)
    static const uint8_t REG_THR = 0x00;       // Transmit Holding Register (Write)
    static const uint8_t REG_IER = 0x01;       // Interrupt Enable Register
    static const uint8_t REG_FCR = 0x02;       // FIFO Control Register (Write)
    static const uint8_t REG_IIR = 0x02;       // Interrupt Identification Register (Read)
    static const uint8_t REG_LCR = 0x03;       // Line Control Register
    static const uint8_t REG_MCR = 0x04;       // Modem Control Register
    static const uint8_t REG_LSR = 0x05;       // Line Status Register
    static const uint8_t REG_MSR = 0x06;       // Modem Status Register
    static const uint8_t REG_SPR = 0x07;       // Scratch Pad Register
    static const uint8_t REG_TXLVL = 0x08;     // Transmit FIFO Level Register
    static const uint8_t REG_RXLVL = 0x09;     // Receive FIFO Level Register
    static const uint8_t REG_DLL = 0x00;       // Divisor Latch LSB (LCR[7] = 1)
    static const uint8_t REG_DLH = 0x01;       // Divisor Latch MSB (LCR[7] = 1)
    static const uint8_t REG_EFR = 0x02;       // Enhanced Feature Register

    // GPIO Registers
    static const uint8_t REG_IODir = 0x0A;     // GPIO Direction Register
    static const uint8_t REG_IOState = 0x0B;   // GPIO State Register
    static const uint8_t REG_IOIntEna = 0x0C;  // GPIO Interrupt Enable Register
    static const uint8_t REG_IOControl = 0x0E; // GPIO Control Register
    static const uint8_t REG_EFCR = 0x0F;      // Extra Features Control Register

    // PWM Registers
    static const uint8_t REG_PWM_CTRL = 0x10;  // PWM Control Register
    static const uint8_t REG_PWM_DUTY = 0x11;  // PWM Duty Cycle Register
    static const uint8_t REG_PWM_PRE = 0x12;   // PWM Prescaler Register

    //=====================================================================
    // Register Bit Definitions
    //=====================================================================

    // Line Status Register bits
    static const uint8_t LSR_DATA_READY = 0x01;
    static const uint8_t LSR_OVERRUN_ERROR = 0x02;
    static const uint8_t LSR_PARITY_ERROR = 0x04;
    static const uint8_t LSR_FRAMING_ERROR = 0x08;
    static const uint8_t LSR_BREAK_INTERRUPT = 0x10;
    static const uint8_t LSR_THR_EMPTY = 0x20;
    static const uint8_t LSR_TRANSMITTER_EMPTY = 0x40;
    static const uint8_t LSR_FIFO_ERROR = 0x80;

    // Line Control Register bits
    static const uint8_t LCR_WORD_LEN_5 = 0x00;
    static const uint8_t LCR_WORD_LEN_6 = 0x01;
    static const uint8_t LCR_WORD_LEN_7 = 0x02;
    static const uint8_t LCR_WORD_LEN_8 = 0x03;
    static const uint8_t LCR_STOP_1 = 0x00;
    static const uint8_t LCR_STOP_2 = 0x04;
    static const uint8_t LCR_PARITY_NONE = 0x00;
    static const uint8_t LCR_PARITY_ODD = 0x08;
    static const uint8_t LCR_PARITY_EVEN = 0x18;
    static const uint8_t LCR_BREAK_ENABLE = 0x40;
    static const uint8_t LCR_DIVISOR_ENABLE = 0x80;
    static const uint8_t LCR_ACCESS_EFR = 0xBF;

    // FIFO Control Register bits (add these to the public section)
    static const uint8_t FCR_FIFO_ENABLE = 0x01;
    static const uint8_t FCR_RX_FIFO_RESET = 0x02;
    static const uint8_t FCR_TX_FIFO_RESET = 0x04;
    static const uint8_t FCR_TX_TRIGGER_LVL0 = 0x00;
    static const uint8_t FCR_TX_TRIGGER_LVL1 = 0x10;
    static const uint8_t FCR_TX_TRIGGER_LVL2 = 0x20;
    static const uint8_t FCR_TX_TRIGGER_LVL3 = 0x30;
    static const uint8_t FCR_RX_TRIGGER_LVL0 = 0x00;
    static const uint8_t FCR_RX_TRIGGER_LVL1 = 0x40;
    static const uint8_t FCR_RX_TRIGGER_LVL2 = 0x80;
    static const uint8_t FCR_RX_TRIGGER_LVL3 = 0xC0;

    // Modem Control Register bits
    static const uint8_t MCR_DTR = 0x01;
    static const uint8_t MCR_RTS = 0x02;
    static const uint8_t MCR_LOOPBACK = 0x10;
    static const uint8_t MCR_TCR_TLR = 0x40;
    static const uint8_t MCR_XON_ANY = 0x20;

    //=====================================================================
    // GPIO and PWM Definitions
    //=====================================================================

    // GPIO Pin Definitions
    static const uint8_t GPIO_0 = 0;
    static const uint8_t GPIO_1 = 1;
    static const uint8_t GPIO_2 = 2;
    static const uint8_t GPIO_3 = 3;
    static const uint8_t GPIO_4 = 4;
    static const uint8_t GPIO_5 = 5;
    static const uint8_t GPIO_6 = 6;
    static const uint8_t GPIO_7 = 7;

    // GPIO Direction and States (renamed to avoid Arduino conflicts)
    static const uint8_t GPIO_INPUT = 0x01;
    static const uint8_t GPIO_OUTPUT = 0x00;
    static const uint8_t GPIO_LOW = 0x00;
    static const uint8_t GPIO_HIGH = 0x01;

    // PWM Constants
    static const uint8_t PWM_OFF = 0x00;
    static const uint8_t PWM_ON = 0x01;

    enum class PWMResolution {
        PWM_8_BIT = 8,
        PWM_10_BIT = 10,
        PWM_12_BIT = 12,
        PWM_16_BIT = 16
    };

    enum class PWMClockSource {
        XTAL = 0,      // Crystal/External Clock
        INTERNAL = 1   // Internal oscillator
    };

    //=====================================================================
    // Public Data Structures
    //=====================================================================

    #if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)
    struct I2CPins {
        int sda;
        int scl;
        I2CPins(int sda_pin = -1, int scl_pin = -1) : 
            sda(sda_pin), scl(scl_pin) {}
    };
    #endif

    struct TransferResult {
        uint16_t bytesTransferred;
        int8_t error;
        uint32_t timeMs;
        bool complete;

        TransferResult() : 
            bytesTransferred(0),
            error(OK),
            timeMs(0),
            complete(false) {}
    };

    struct UARTStatus {
        uint8_t lsr;
        uint8_t msr;
        uint8_t txlvl;
        uint8_t rxlvl;
        int8_t error;

        UARTStatus() :
            lsr(0), msr(0), txlvl(0), rxlvl(0), error(OK) {}
    };


    struct TransferTimings {
        static const unsigned long SUSTAINED_RATE_BPS_SMALL = 982;
        static const unsigned long SUSTAINED_RATE_BPS_LARGE = 719;
        static const uint32_t SMALL_PACKET_THRESHOLD = 32;
        static const uint32_t FIFO_STABILIZE_TIME = 100;
        static const uint32_t FIFO_WAIT_MS = 27;

        uint32_t INTER_BYTE_DELAY_US;
        uint32_t INTER_CHUNK_DELAY_US;
        uint32_t STATUS_CHECK_DELAY_US;

        TransferTimings() :
            INTER_BYTE_DELAY_US(45),
            INTER_CHUNK_DELAY_US(180),
            STATUS_CHECK_DELAY_US(90) {}

        void adjustForPacketSize(size_t packetSize);
    };

    struct TransferStats {
        uint32_t totalTransfers;
        uint32_t successfulTransfers;
        uint32_t totalBytes;
        uint32_t totalTimeMs;
        float averageRate;
        uint32_t maxSuccessiveTransfers;
        uint32_t currentSuccessiveTransfers;

        TransferStats() {
            reset();
        }

        void reset() {
            totalTransfers = 0;
            successfulTransfers = 0;
            totalBytes = 0;
            totalTimeMs = 0;
            averageRate = 0;
            maxSuccessiveTransfers = 0;
            currentSuccessiveTransfers = 0;
        }

        void updateRate() {
            if (totalTimeMs > 0) {
                averageRate = (float)(totalBytes * 1000) / totalTimeMs;
            }
        }

        void recordTransfer(bool success, size_t bytes, uint32_t timeMs) {
            totalTransfers++;
            if (success) {
                successfulTransfers++;
                totalBytes += bytes;
                totalTimeMs += timeMs;
                currentSuccessiveTransfers++;
                if (currentSuccessiveTransfers > maxSuccessiveTransfers) {
                    maxSuccessiveTransfers = currentSuccessiveTransfers;
                }
            } else {
                currentSuccessiveTransfers = 0;
            }
            updateRate();
        }
    };

    //=====================================================================
    // Public Methods
    //=====================================================================

    // Constructors
    #if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)
    explicit SC16IS752(TwoWire& wire = Wire, const I2CPins& pins = I2CPins(DEFAULT_SDA_PIN, DEFAULT_SCL_PIN));
    #else
    explicit SC16IS752(TwoWire& wire = Wire);
    #endif

    // Initialization Methods
    bool begin(uint8_t i2cAddr = I2C_DEFAULT_ADDR, uint32_t i2cFreq = DEFAULT_I2C_FREQ);
    void end();
    bool detectDevice();

    // UART Configuration and Status
    int initializeUART(uint8_t channel, uint32_t baudRate, bool testMode = false, uint32_t xtalFreq = 1843200);
    int setBaudRate(uint8_t channel, uint32_t baud, uint32_t xtalFreq = 1843200);
    int setLineControl(uint8_t channel, uint8_t databits, uint8_t stopbits, uint8_t parity);
    int configureFIFO(uint8_t channel, uint8_t txTrigger, uint8_t rxTrigger);
    UARTStatus getStatus(uint8_t channel);
    int getErrorStatus(uint8_t channel);
    bool isTxEmpty(uint8_t channel);
    bool isRxAvailable(uint8_t channel);

    // UART Transfer Methods
    TransferResult writeBufferOptimized(uint8_t channel, const uint8_t* buffer, size_t length);
    TransferResult readBufferOptimized(uint8_t channel, uint8_t* buffer, size_t length);
    TransferResult writeBufferChunked(uint8_t channel, const uint8_t* buffer, size_t length);
    TransferResult readBufferChunked(uint8_t channel, uint8_t* buffer, size_t length);
    int writeByte(uint8_t channel, uint8_t data);
    int readByte(uint8_t channel);
    size_t writeBytes(uint8_t channel, const uint8_t* data, size_t length);
    size_t readBytes(uint8_t channel, uint8_t* buffer, size_t length);

    // GPIO Methods
    void pinMode(uint8_t pin, uint8_t mode);
    void digitalWrite(uint8_t pin, uint8_t state);
    uint8_t digitalRead(uint8_t pin);

    // PWM Methods
    bool pwmBegin(uint8_t pin, uint32_t frequency = 1000, PWMResolution resolution = PWMResolution::PWM_8_BIT);
    void pwmWrite(uint8_t pin, uint32_t value);
    void pwmWriteHR(uint8_t pin, uint32_t value);
    void pwmWriteFrequency(uint8_t pin, uint32_t frequency);
    void pwmWriteResolution(uint8_t pin, PWMResolution resolution);
    uint32_t pwmGetMaxValue(uint8_t pin) const;
    uint32_t pwmGetResolution(uint8_t pin) const;
    uint32_t pwmGetFrequency(uint8_t pin) const;
    void pwmSetClockSource(PWMClockSource source);
    void pwmEnd(uint8_t pin);
    void analogWrite(uint8_t pin, uint8_t value);

    // Statistics and Debug
    TransferStats getTransferStats() const;
    void resetTransferStats();

    // Register Access Methods
    int readReg(uint8_t reg);
    int writeReg(uint8_t reg, uint8_t value);
    uint8_t regAddr(uint8_t reg, uint8_t channel) const;

    #if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)
    bool setPins(int sda, int scl);
    bool setI2CFrequency(uint32_t frequency);
    #endif

protected:
    //=====================================================================
    // Protected Methods
    //=====================================================================
    
    bool verifyRegisterWrite(uint8_t reg, uint8_t value);
    bool isValidChannel(uint8_t channel) const;
    bool isValidGPIOPin(uint8_t pin) const;

private:
    //=====================================================================
    // Private Data Structures
    //=====================================================================

    struct PWMState {
        bool enabled;
        uint32_t frequency;
        uint32_t dutyCycle;
        uint32_t prescaler;
        PWMResolution resolution;
        PWMClockSource clockSource;
        
        PWMState() : 
            enabled(false), 
            frequency(1000), 
            dutyCycle(0), 
            prescaler(1),
            resolution(PWMResolution::PWM_8_BIT),
            clockSource(PWMClockSource::XTAL) {}
    };

    struct TransferStates {
        static const uint8_t CHUNK_SIZE = 8;
        static const uint8_t BUFFER_SIZE = 16;
        static const uint8_t LSR_READY = 0x60;
        static const uint8_t LSR_BUSY = 0x21;
        static const uint8_t LSR_DATA = 0x61;
        static const uint8_t FIFO_FULL = 16;
    };

    //=====================================================================
    // Private Members
    //=====================================================================

    TwoWire& _wire;                   // I2C interface
    uint8_t _i2cAddr;                 // I2C address
    bool _initialized;                // Initialization flag
    uint32_t _currentBaudRate;        // Current baud rate
    uint8_t _gpioDirection;           // Cache GPIO direction register
    uint8_t _gpioState;              // Cache GPIO state register
    PWMState _pwmStates[8];          // PWM state for each pin
    TransferStats _stats;             // Performance tracking
    TransferTimings _timings;         // Timing configuration

    #if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)
    I2CPins _pins;                    // Platform-specific pins
    uint32_t _i2cFreq;               // I2C frequency
    #endif

    //=====================================================================
    // Private Methods
    //=====================================================================

    // I2C and Platform-specific methods
    #if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)
    bool initI2C();                   // Initialize I2C interface
    #endif

    // Buffer and timing management
    unsigned long calculateByteTransmitTime(uint32_t baudRate);
    bool waitForRxData(uint8_t channel, unsigned long timeoutMs);
    bool waitForTxSpace(uint8_t channel, size_t required, unsigned long timeoutMs);
    bool waitForTransmissionComplete(uint8_t channel);
    int clearChannelBuffers(uint8_t channel);
    
    // Enhanced transfer helpers
    bool waitForTxReady(uint8_t channel, unsigned long timeoutMs);
    bool waitForRxReady(uint8_t channel, unsigned long timeoutMs);
    bool verifyTransfer(uint8_t channel, const uint8_t* txBuffer, size_t length);
    uint8_t getOptimalChunkSize(int lsr, int txlvl, size_t remaining, bool burstMode);

    // GPIO helpers
    void updateGPIORegister(uint8_t reg, uint8_t pin, uint8_t value);
    uint8_t readGPIORegister(uint8_t reg, uint8_t pin);

    // PWM helpers
    void updatePWMPrescaler(uint8_t pin);
    void updatePWMSettings(uint8_t pin);
    uint32_t calculatePrescaler(uint32_t targetFreq, PWMResolution resolution) const;
    uint32_t getClockFrequency() const;

    // Performance optimization
    void adjustDelaysBasedOnPerformance();
    void updateTransferStatistics(bool success, size_t bytes, uint32_t timeMs);

    // Utility functions
    template<typename T>
    inline T min(T a, T b) const {
        return (a < b) ? a : b;
    }

    template<typename T>
    inline T max(T a, T b) const {
        return (a > b) ? a : b;
    }
};

#endif // SC16IS752_H

