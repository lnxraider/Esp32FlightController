/**
 * @file sc16is752.hpp
 * @brief PX4 driver for SC16IS752 Dual UART with I2C interface
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <lib/parameters/param.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>

class SC16IS752 : public device::I2C, public I2CSPIDriver<SC16IS752>
{
public:
    /* Device limits */
    static constexpr uint8_t MAX_CHANNELS = 2;
    static constexpr uint8_t MAX_GPIO_PINS = 8;
    static constexpr uint8_t FIFO_DEPTH = 64;
    static constexpr uint32_t DEFAULT_I2C_FREQUENCY = 400000;

    /* Channel definitions */
    static constexpr uint8_t CHANNEL_A = 0;
    static constexpr uint8_t CHANNEL_B = 1;

    /* GPIO pin definitions */
    static constexpr uint8_t GPIO_0 = 0;
    static constexpr uint8_t GPIO_1 = 1;
    static constexpr uint8_t GPIO_2 = 2;
    static constexpr uint8_t GPIO_3 = 3;
    static constexpr uint8_t GPIO_4 = 4;
    static constexpr uint8_t GPIO_5 = 5;
    static constexpr uint8_t GPIO_6 = 6;
    static constexpr uint8_t GPIO_7 = 7;

    /* GPIO directions */
    static constexpr uint8_t GPIO_INPUT = 0x01;
    static constexpr uint8_t GPIO_OUTPUT = 0x00;

    /* GPIO states */
    static constexpr uint8_t GPIO_LOW = 0x00;
    static constexpr uint8_t GPIO_HIGH = 0x01;

    /* Register definitions */
    static constexpr uint8_t REG_RHR = 0x00;        // Receive Holding Register (Read)
    static constexpr uint8_t REG_THR = 0x00;        // Transmit Holding Register (Write)
    static constexpr uint8_t REG_IER = 0x01;        // Interrupt Enable Register
    static constexpr uint8_t REG_FCR = 0x02;        // FIFO Control Register (Write)
    static constexpr uint8_t REG_IIR = 0x02;        // Interrupt Identification Register (Read)
    static constexpr uint8_t REG_LCR = 0x03;        // Line Control Register
    static constexpr uint8_t REG_MCR = 0x04;        // Modem Control Register
    static constexpr uint8_t REG_LSR = 0x05;        // Line Status Register
    static constexpr uint8_t REG_MSR = 0x06;        // Modem Status Register
    static constexpr uint8_t REG_SPR = 0x07;        // Scratch Pad Register
    static constexpr uint8_t REG_TXLVL = 0x08;      // Transmit FIFO Level
    static constexpr uint8_t REG_RXLVL = 0x09;      // Receive FIFO Level
    static constexpr uint8_t REG_IODIR = 0x0A;      // GPIO Direction Register
    static constexpr uint8_t REG_IOSTATE = 0x0B;    // GPIO State Register
    static constexpr uint8_t REG_IOCTRL = 0x0E;     // GPIO Control Register
    static constexpr uint8_t REG_EFCR = 0x0F;       // Extra Features Control Register
    static constexpr uint8_t REG_DLL = 0x00;        // Divisor Latch LSB (LCR[7] = 1)
    static constexpr uint8_t REG_DLH = 0x01;        // Divisor Latch MSB (LCR[7] = 1)

    /* Line Control Register bits */
    static constexpr uint8_t LCR_WORD_LEN_5 = 0x00;
    static constexpr uint8_t LCR_WORD_LEN_6 = 0x01;
    static constexpr uint8_t LCR_WORD_LEN_7 = 0x02;
    static constexpr uint8_t LCR_WORD_LEN_8 = 0x03;
    static constexpr uint8_t LCR_STOP_1 = 0x00;
    static constexpr uint8_t LCR_STOP_2 = 0x04;
    static constexpr uint8_t LCR_PARITY_NONE = 0x00;
    static constexpr uint8_t LCR_PARITY_ODD = 0x08;
    static constexpr uint8_t LCR_PARITY_EVEN = 0x18;
    static constexpr uint8_t LCR_DIVISOR_ENABLE = 0x80;

    /* FIFO Control Register bits */
    static constexpr uint8_t FCR_FIFO_ENABLE = 0x01;
    static constexpr uint8_t FCR_RX_FIFO_RESET = 0x02;
    static constexpr uint8_t FCR_TX_FIFO_RESET = 0x04;
    static constexpr uint8_t FCR_TX_TRIGGER_LVL0 = 0x00;
    static constexpr uint8_t FCR_TX_TRIGGER_LVL1 = 0x10;
    static constexpr uint8_t FCR_TX_TRIGGER_LVL2 = 0x20;
    static constexpr uint8_t FCR_TX_TRIGGER_LVL3 = 0x30;
    static constexpr uint8_t FCR_RX_TRIGGER_LVL0 = 0x00;
    static constexpr uint8_t FCR_RX_TRIGGER_LVL1 = 0x40;
    static constexpr uint8_t FCR_RX_TRIGGER_LVL2 = 0x80;
    static constexpr uint8_t FCR_RX_TRIGGER_LVL3 = 0xC0;

    /* Line Status Register bits */
    static constexpr uint8_t LSR_DATA_READY = 0x01;
    static constexpr uint8_t LSR_OVERRUN_ERROR = 0x02;
    static constexpr uint8_t LSR_PARITY_ERROR = 0x04;
    static constexpr uint8_t LSR_FRAMING_ERROR = 0x08;
    static constexpr uint8_t LSR_BREAK_INTERRUPT = 0x10;
    static constexpr uint8_t LSR_THR_EMPTY = 0x20;
    static constexpr uint8_t LSR_TRANSMITTER_EMPTY = 0x40;
    static constexpr uint8_t LSR_FIFO_ERROR = 0x80;

    /* PWM Configuration */
    enum class PWMResolution : uint8_t {
        PWM_8_BIT = 8,
        PWM_10_BIT = 10,
        PWM_12_BIT = 12,
        PWM_16_BIT = 16
    };

    /* PWM Register definitions */
    static constexpr uint8_t REG_PWM_CTRL = 0x10;   // PWM Control Register
    static constexpr uint8_t REG_PWM_DUTY = 0x11;   // PWM Duty Cycle Register Base
    static constexpr uint8_t REG_PWM_PRE = 0x12;    // PWM Prescaler Register Base

    /* Device structures */
    struct UARTStatus {
        uint8_t lsr{0};      // Line Status Register
        uint8_t msr{0};      // Modem Status Register
        uint8_t txlvl{0};    // TX FIFO Level
        uint8_t rxlvl{0};    // RX FIFO Level
        int error{0};        // Error status
    };

    struct TransferResult {
        size_t bytes_transferred{0};
        int error{0};
        uint64_t time_us{0};
        bool complete{false};
    };

    struct PWMConfig {
        bool enabled{false};
        uint32_t frequency{1000};
        uint32_t duty_cycle{0};
        PWMResolution resolution{PWMResolution::PWM_8_BIT};
    };

    /* Constructor / Destructor */
    SC16IS752(const I2CSPIDriverConfig &config);
    ~SC16IS752() override;

    /* Base class overrides */
    static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config,
                                       int runtime_instance);
    static void print_usage();
    int init() override;
    void print_status() override;

    /* UART Methods */
    int initializeUART(uint8_t channel, uint32_t baud_rate);
    int setBaudRate(uint8_t channel, uint32_t baud);
    int setLineControl(uint8_t channel, uint8_t databits, uint8_t stopbits, uint8_t parity);
    int configureFIFO(uint8_t channel, uint8_t tx_trigger, uint8_t rx_trigger);
    UARTStatus getStatus(uint8_t channel);
    bool isTxEmpty(uint8_t channel);
    bool isRxAvailable(uint8_t channel);

    /* Transfer Methods */
    TransferResult writeBufferOptimized(uint8_t channel, const uint8_t* buffer, size_t length);
    TransferResult readBufferOptimized(uint8_t channel, uint8_t* buffer, size_t length);
    size_t writeBuffer(uint8_t channel, const uint8_t* buffer, size_t length);
    size_t readBuffer(uint8_t channel, uint8_t* buffer, size_t length);
    int writeByte(uint8_t channel, uint8_t data);
    int readByte(uint8_t channel);

    /* GPIO Methods */
    int setPinMode(uint8_t pin, uint8_t mode);
    int writePin(uint8_t pin, bool state);
    bool readPin(uint8_t pin);

    /* PWM Methods */
    int pwmBegin(uint8_t pin, uint32_t frequency = 1000, PWMResolution resolution = PWMResolution::PWM_8_BIT);
    int pwmWrite(uint8_t pin, uint32_t value);
    int pwmWriteHR(uint8_t pin, uint32_t value);
    int pwmEnd(uint8_t pin);
    int analogWrite(uint8_t pin, uint8_t value);
    uint32_t getPWMMaxValue(uint8_t pin) const;
    bool isPWMEnabled(uint8_t pin) const;

protected:
    void RunImpl() override;

private:
    /* Private device interface */
    int probe() override;
    int readRegister(uint8_t reg, uint8_t &val);
    int writeRegister(uint8_t reg, uint8_t val);
    uint8_t regAddr(uint8_t reg, uint8_t channel) const;
    bool isValidChannel(uint8_t channel) const;
    bool isValidPin(uint8_t pin) const;

    /* GPIO helpers */
    struct GPIOState {
        bool is_pwm{false};
        bool is_input{false};
        uint8_t state{0};
    };

    int configureGPIOPin(uint8_t pin, bool is_input);
    bool isPWMPin(uint8_t pin) const;
    void updateGPIOState(uint8_t pin, bool is_input, uint8_t state);
    void updateGPIORegister(uint8_t reg, uint8_t pin, uint8_t value);
    uint8_t readGPIORegister(uint8_t reg);

    /* PWM helpers */
    int updatePWMPrescaler(uint8_t pin);
    int updatePWMSettings(uint8_t pin);
    int configurePWMPin(uint8_t pin);
    uint32_t calculatePWMPrescaler(uint32_t target_freq, PWMResolution resolution) const;

    /* Transfer helpers */
    bool waitForTxEmpty(uint8_t channel, uint32_t timeout_ms);
    bool waitForRxData(uint8_t channel, uint32_t timeout_ms);

    /* Device state */
    bool _initialized{false};
    uint8_t _gpio_direction{0};
    uint8_t _gpio_state{0};
    uint32_t _current_baudrate{0};
    GPIOState _gpio_states[MAX_GPIO_PINS]{};
    PWMConfig _pwm_configs[MAX_GPIO_PINS]{};

    /* Performance monitoring */
    perf_counter_t _comms_errors{nullptr};
    perf_counter_t _sample_perf{nullptr};
    perf_counter_t _transfer_perf{nullptr};

    /* Buffer handling */
    static constexpr size_t BUFFER_SIZE = 256;
    uint8_t _tx_buffer[BUFFER_SIZE];
    uint8_t _rx_buffer[BUFFER_SIZE];

    /* Timing constants */
    static constexpr uint32_t PROBE_TIMEOUT_MS = 1000;
    static constexpr uint32_t INIT_TIMEOUT_MS = 2000;
    static constexpr uint32_t TRANSFER_TIMEOUT_MS = 1000;
    static constexpr uint32_t REGISTER_TIMEOUT_MS = 100;
    static constexpr uint32_t XTAL_FREQ = 14745600;  // 14.7456 MHz crystal
};

