/**
 * @file sc16is752.cpp
 * @brief PX4 driver implementation for SC16IS752 Dual UART with I2C interface
 */

#include "sc16is752.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

using namespace time_literals;

SC16IS752::SC16IS752(const I2CSPIDriverConfig &config) :
    I2C(config),
    I2CSPIDriver(config)
{
    // Initialize performance counters
    _comms_errors = perf_alloc(PC_COUNT, MODULE_NAME": comm_errors");
    _sample_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": read");
    _transfer_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": transfer");
}

SC16IS752::~SC16IS752()
{
    // Clean up PWM
    for (uint8_t i = 0; i < MAX_GPIO_PINS; i++) {
        if (_pwm_configs[i].enabled) {
            pwmEnd(i);
        }
    }

    // Free performance counters
    perf_free(_comms_errors);
    perf_free(_sample_perf);
    perf_free(_transfer_perf);
}

int SC16IS752::init()
{
    int ret = I2C::init();
    if (ret != PX4_OK) {
        PX4_ERR("I2C initialization failed");
        return ret;
    }

    // Test device presence with scratch register
    uint8_t test_val = 0x55;
    ret = writeRegister(REG_SPR, test_val);
    if (ret != PX4_OK) {
        PX4_ERR("Device test write failed");
        return PX4_ERROR;
    }

    uint8_t read_val = 0;
    ret = readRegister(REG_SPR, read_val);
    if (ret != PX4_OK || read_val != test_val) {
        PX4_ERR("Device verification failed");
        return PX4_ERROR;
    }

    // Initialize GPIO states
    _gpio_direction = 0;
    _gpio_state = 0;
    for (uint8_t i = 0; i < MAX_GPIO_PINS; i++) {
        _gpio_states[i] = {};
        _pwm_configs[i] = {};
    }

    // Reset FIFOs for both channels
    for (uint8_t channel = 0; channel < MAX_CHANNELS; channel++) {
        ret = writeRegister(regAddr(REG_FCR, channel),
                          FCR_TX_FIFO_RESET | FCR_RX_FIFO_RESET);
        if (ret != PX4_OK) {
            PX4_ERR("FIFO reset failed on channel %d", channel);
            return ret;
        }
        px4_usleep(1000);

        // Configure default FIFO settings
        ret = writeRegister(regAddr(REG_FCR, channel),
                          FCR_FIFO_ENABLE | FCR_TX_TRIGGER_LVL2 | FCR_RX_TRIGGER_LVL2);
        if (ret != PX4_OK) {
            PX4_ERR("FIFO configuration failed on channel %d", channel);
            return ret;
        }
    }

    _initialized = true;
    PX4_INFO("Initialized successfully");
    return PX4_OK;
}

int SC16IS752::probe()
{
    // Try to verify device presence via scratch register
    uint8_t val = 0x55;
    int ret = writeRegister(REG_SPR, val);
    if (ret != PX4_OK) {
        return ret;
    }

    uint8_t read_val = 0;
    ret = readRegister(REG_SPR, read_val);
    return (ret == PX4_OK && read_val == val) ? PX4_OK : PX4_ERROR;
}

void SC16IS752::RunImpl()
{
    if (!_initialized) {
        return;
    }

    perf_begin(_sample_perf);

    // Check each UART channel
    for (uint8_t channel = 0; channel < MAX_CHANNELS; channel++) {
        UARTStatus status = getStatus(channel);
        if (status.error != 0) {
            perf_count(_comms_errors);
            continue;
        }

        // Handle received data if available
        if (status.rxlvl > 0) {
            size_t bytes_read = readBuffer(channel, _rx_buffer,
                                         math::min(status.rxlvl, sizeof(_rx_buffer)));
            if (bytes_read == 0) {
                perf_count(_comms_errors);
            }
        }
    }

    // Check PWM updates if needed
    for (uint8_t i = 0; i < MAX_GPIO_PINS; i++) {
        if (_pwm_configs[i].enabled) {
            // Handle any pending PWM updates
        }
    }

    perf_end(_sample_perf);
}

int SC16IS752::initializeUART(uint8_t channel, uint32_t baud_rate)
{
    if (!isValidChannel(channel)) {
        return -EINVAL;
    }

    // Reset FIFOs
    int ret = writeRegister(regAddr(REG_FCR, channel), 0x00);
    if (ret != PX4_OK) {
        return ret;
    }
    px4_usleep(10000);

    // Set baud rate
    ret = setBaudRate(channel, baud_rate);
    if (ret != PX4_OK) {
        return ret;
    }
    px4_usleep(10000);

    // Configure 8N1 by default
    ret = writeRegister(regAddr(REG_LCR, channel), LCR_WORD_LEN_8);
    if (ret != PX4_OK) {
        return ret;
    }
    px4_usleep(10000);

    // Enable and configure FIFOs
    ret = writeRegister(regAddr(REG_FCR, channel),
                       FCR_FIFO_ENABLE | FCR_TX_TRIGGER_LVL2 | FCR_RX_TRIGGER_LVL2);
    if (ret != PX4_OK) {
        return ret;
    }

    // Disable interrupts
    ret = writeRegister(regAddr(REG_IER, channel), 0x00);
    if (ret == PX4_OK) {
        _current_baudrate = baud_rate;
    }

    return ret;
}

int SC16IS752::setBaudRate(uint8_t channel, uint32_t baud)
{
    if (!isValidChannel(channel) || baud == 0) {
        return -EINVAL;
    }

    // Calculate divisor for crystal frequency
    uint16_t divisor = XTAL_FREQ / (16 * baud);
    if (divisor == 0) {
        return -EINVAL;
    }

    // Save current LCR value
    uint8_t lcr = 0;
    int ret = readRegister(regAddr(REG_LCR, channel), lcr);
    if (ret != PX4_OK) {
        return ret;
    }

    // Enable divisor latch
    ret = writeRegister(regAddr(REG_LCR, channel), lcr | LCR_DIVISOR_ENABLE);
    if (ret != PX4_OK) {
        return ret;
    }

    // Set divisor
    ret = writeRegister(regAddr(REG_DLL, channel), divisor & 0xFF);
    if (ret != PX4_OK) {
        return ret;
    }

    ret = writeRegister(regAddr(REG_DLH, channel), (divisor >> 8) & 0xFF);
    if (ret != PX4_OK) {
        return ret;
    }

    // Restore original LCR value
    return writeRegister(regAddr(REG_LCR, channel), lcr);
}

int SC16IS752::setLineControl(uint8_t channel, uint8_t databits, uint8_t stopbits, uint8_t parity)
{
    if (!isValidChannel(channel)) {
        return -EINVAL;
    }
    return writeRegister(regAddr(REG_LCR, channel), databits | stopbits | parity);
}

int SC16IS752::configureFIFO(uint8_t channel, uint8_t tx_trigger, uint8_t rx_trigger)
{
    if (!isValidChannel(channel)) {
        return -EINVAL;
    }

    // First disable FIFO
    int ret = writeRegister(regAddr(REG_FCR, channel), 0);
    if (ret != PX4_OK) {
        return ret;
    }
    px4_usleep(10000);

    // Reset both FIFOs
    ret = writeRegister(regAddr(REG_FCR, channel),
                       FCR_FIFO_ENABLE | FCR_RX_FIFO_RESET | FCR_TX_FIFO_RESET);
    if (ret != PX4_OK) {
        return ret;
    }
    px4_usleep(10000);

    // Configure trigger levels
    return writeRegister(regAddr(REG_FCR, channel),
                        FCR_FIFO_ENABLE | tx_trigger | rx_trigger);
}

SC16IS752::UARTStatus SC16IS752::getStatus(uint8_t channel)
{
    UARTStatus status{};

    if (!isValidChannel(channel)) {
        status.error = -EINVAL;
        return status;
    }

    uint8_t val = 0;
    if (readRegister(regAddr(REG_LSR, channel), val) == PX4_OK) {
        status.lsr = val;
    } else {
        status.error = -EIO;
        return status;
    }

    if (readRegister(regAddr(REG_MSR, channel), val) == PX4_OK) {
        status.msr = val;
    }

    if (readRegister(regAddr(REG_TXLVL, channel), val) == PX4_OK) {
        status.txlvl = val;
    }

    if (readRegister(regAddr(REG_RXLVL, channel), val) == PX4_OK) {
        status.rxlvl = val;
    }

    return status;
}

bool SC16IS752::isTxEmpty(uint8_t channel)
{
    if (!isValidChannel(channel)) {
        return false;
    }

    uint8_t lsr = 0;
    if (readRegister(regAddr(REG_LSR, channel), lsr) != PX4_OK) {
        return false;
    }

    return (lsr & LSR_THR_EMPTY) != 0;
}

bool SC16IS752::isRxAvailable(uint8_t channel)
{
    if (!isValidChannel(channel)) {
        return false;
    }

    uint8_t lsr = 0;
    if (readRegister(regAddr(REG_LSR, channel), lsr) != PX4_OK) {
        return false;
    }

    return (lsr & LSR_DATA_READY) != 0;
}

// Transfer Methods Implementation

TransferResult SC16IS752::writeBufferOptimized(uint8_t channel,
                                              const uint8_t* buffer,
                                              size_t length)
{
    TransferResult result{};

    if (!isValidChannel(channel) || !buffer || length == 0) {
        result.error = -EINVAL;
        return result;
    }

    perf_begin(_transfer_perf);

    const uint64_t start_time = hrt_absolute_time();
    size_t bytes_remaining = length;
    size_t current_index = 0;

    while (bytes_remaining > 0) {
        // Check TX FIFO level
        UARTStatus status = getStatus(channel);
        if (status.error != 0) {
            result.error = status.error;
            break;
        }

        if (status.txlvl == 0) {
            // Wait for space in FIFO
            if (!waitForTxEmpty(channel, TRANSFER_TIMEOUT_MS)) {
                result.error = -ETIMEDOUT;
                break;
            }
            continue;
        }

        // Write as many bytes as possible
        size_t chunk_size = math::min(bytes_remaining, (size_t)status.txlvl);
        for (size_t i = 0; i < chunk_size; i++) {
            int ret = writeRegister(regAddr(REG_THR, channel),
                                  buffer[current_index + i]);
            if (ret != PX4_OK) {
                result.error = ret;
                perf_count(_comms_errors);
                break;
            }
            result.bytes_transferred++;
            bytes_remaining--;
        }

        if (result.error != 0) {
            break;
        }

        current_index += chunk_size;
    }

    result.time_us = hrt_elapsed_time(&start_time);
    result.complete = (result.bytes_transferred == length);

    perf_end(_transfer_perf);
    return result;
}

TransferResult SC16IS752::readBufferOptimized(uint8_t channel,
                                             uint8_t* buffer,
                                             size_t length)
{
    TransferResult result{};

    if (!isValidChannel(channel) || !buffer || length == 0) {
        result.error = -EINVAL;
        return result;
    }

    perf_begin(_transfer_perf);

    const uint64_t start_time = hrt_absolute_time();
    size_t bytes_remaining = length;
    size_t current_index = 0;

    while (bytes_remaining > 0) {
        // Check RX FIFO level
        UARTStatus status = getStatus(channel);
        if (status.error != 0) {
            result.error = status.error;
            break;
        }

        if (status.rxlvl == 0) {
            // Wait for data
            if (!waitForRxData(channel, TRANSFER_TIMEOUT_MS)) {
                result.error = -ETIMEDOUT;
                break;
            }
            continue;
        }

        // Read available data
        size_t chunk_size = math::min(bytes_remaining, (size_t)status.rxlvl);
        for (size_t i = 0; i < chunk_size; i++) {
            uint8_t val = 0;
            int ret = readRegister(regAddr(REG_RHR, channel), val);
            if (ret != PX4_OK) {
                result.error = ret;
                perf_count(_comms_errors);
                break;
            }
            buffer[current_index + i] = val;
            result.bytes_transferred++;
            bytes_remaining--;
        }

        if (result.error != 0) {
            break;
        }

        current_index += chunk_size;
    }

    result.time_us = hrt_elapsed_time(&start_time);
    result.complete = (result.bytes_transferred == length);

    perf_end(_transfer_perf);
    return result;
}

size_t SC16IS752::writeBuffer(uint8_t channel, const uint8_t* buffer, size_t length)
{
    if (!isValidChannel(channel) || !buffer || length == 0) {
        return 0;
    }

    size_t bytes_written = 0;
    const uint64_t start_time = hrt_absolute_time();

    while (bytes_written < length) {
        if (hrt_elapsed_time(&start_time) > TRANSFER_TIMEOUT_MS * 1000) {
            break;
        }

        if (!waitForTxEmpty(channel, TRANSFER_TIMEOUT_MS)) {
            break;
        }

        int ret = writeRegister(regAddr(REG_THR, channel), buffer[bytes_written]);
        if (ret != PX4_OK) {
            perf_count(_comms_errors);
            break;
        }

        bytes_written++;
    }

    return bytes_written;
}

size_t SC16IS752::readBuffer(uint8_t channel, uint8_t* buffer, size_t length)
{
    if (!isValidChannel(channel) || !buffer || length == 0) {
        return 0;
    }

    size_t bytes_read = 0;
    const uint64_t start_time = hrt_absolute_time();

    while (bytes_read < length) {
        if (hrt_elapsed_time(&start_time) > TRANSFER_TIMEOUT_MS * 1000) {
            break;
        }

        if (!waitForRxData(channel, TRANSFER_TIMEOUT_MS)) {
            break;
        }

        uint8_t val = 0;
        int ret = readRegister(regAddr(REG_RHR, channel), val);
        if (ret != PX4_OK) {
            perf_count(_comms_errors);
            break;
        }

        buffer[bytes_read] = val;
        bytes_read++;
    }

    return bytes_read;
}

int SC16IS752::writeByte(uint8_t channel, uint8_t data)
{
    if (!isValidChannel(channel)) {
        return -EINVAL;
    }

    if (!waitForTxEmpty(channel, TRANSFER_TIMEOUT_MS)) {
        return -ETIMEDOUT;
    }

    return writeRegister(regAddr(REG_THR, channel), data);
}

int SC16IS752::readByte(uint8_t channel)
{
    if (!isValidChannel(channel)) {
        return -EINVAL;
    }

    if (!waitForRxData(channel, TRANSFER_TIMEOUT_MS)) {
        return -ETIMEDOUT;
    }

    uint8_t val = 0;
    int ret = readRegister(regAddr(REG_RHR, channel), val);
    return (ret == PX4_OK) ? val : ret;
}

// GPIO Methods Implementation

int SC16IS752::setPinMode(uint8_t pin, uint8_t mode)
{
    int ret = configureGPIOPin(pin, mode == GPIO_INPUT);
    if (ret == PX4_OK) {
        updateGPIOState(pin, mode == GPIO_INPUT, 0);
    }
    return ret;
}

int SC16IS752::writePin(uint8_t pin, bool state)
{
    if (!isValidPin(pin) || _gpio_states[pin].is_input || _gpio_states[pin].is_pwm) {
        return -EINVAL;
    }

    int ret = updateGPIORegister(REG_IOSTATE, pin, state ? 1 : 0);
    if (ret == PX4_OK) {
        _gpio_states[pin].state = state ? GPIO_HIGH : GPIO_LOW;
    }
    return ret;
}

bool SC16IS752::readPin(uint8_t pin)
{
    if (!isValidPin(pin)) {
        return false;
    }

    uint8_t gpio_state = readGPIORegister(REG_IOSTATE);
    return (gpio_state & (1 << pin)) != 0;
}

int SC16IS752::configureGPIOPin(uint8_t pin, bool is_input)
{
    if (!isValidPin(pin)) {
        return -EINVAL;
    }

    // Cannot configure if pin is in PWM mode
    if (_pwm_configs[pin].enabled) {
        return -EBUSY;
    }

    // Configure pin for GPIO operation first
    uint8_t io_control = 0;
    int ret = readRegister(REG_IOCTRL, io_control);
    if (ret != PX4_OK) {
        return ret;
    }

    io_control |= (1 << pin);  // Set pin for GPIO mode
    ret = writeRegister(REG_IOCTRL, io_control);
    if (ret != PX4_OK) {
        return ret;
    }

    // Set direction
    ret = updateGPIORegister(REG_IODIR, pin, is_input ? 1 : 0);
    if (ret == PX4_OK) {
        _gpio_states[pin].is_input = is_input;
        _gpio_states[pin].is_pwm = false;
    }
    return ret;
}

// PWM Methods Implementation

int SC16IS752::pwmBegin(uint8_t pin, uint32_t frequency, PWMResolution resolution)
{
    if (!isValidPin(pin)) {
        return -EINVAL;
    }

    // Configure pin for GPIO output first
    int ret = setPinMode(pin, GPIO_OUTPUT);
    if (ret != PX4_OK) {
        return ret;
    }

    // Initialize PWM configuration
    _pwm_configs[pin].enabled = true;
    _pwm_configs[pin].frequency = frequency;
    _pwm_configs[pin].resolution = resolution;
    _pwm_configs[pin].duty_cycle = 0;

    // Configure the PWM pin
    ret = configurePWMPin(pin);
    if (ret != PX4_OK) {
        _pwm_configs[pin].enabled = false;
        return ret;
    }

    // Calculate and set prescaler
    ret = updatePWMPrescaler(pin);
    if (ret != PX4_OK) {
        _pwm_configs[pin].enabled = false;
        return ret;
    }

    // Configure PWM settings
    ret = updatePWMSettings(pin);
    if (ret != PX4_OK) {
        _pwm_configs[pin].enabled = false;
        return ret;
    }

    // Set initial duty cycle to 0
    return pwmWrite(pin, 0);
}

int SC16IS752::pwmWrite(uint8_t pin, uint32_t value)
{
    if (!isValidPin(pin) || !_pwm_configs[pin].enabled) {
        return -EINVAL;
    }

    // Clamp value to resolution
    uint32_t max_value = getPWMMaxValue(pin);
    value = math::min(value, max_value);

    _pwm_configs[pin].duty_cycle = value;

    // Calculate register values based on resolution
    uint32_t reg_value = (value * max_value) >>
                        static_cast<uint8_t>(_pwm_configs[pin].resolution);

    // Write duty cycle registers
    int ret = writeRegister(REG_PWM_DUTY + (pin * 2), reg_value & 0xFF);
    if (ret != PX4_OK) {
        return ret;
    }

    return writeRegister(REG_PWM_DUTY + (pin * 2) + 1, (reg_value >> 8) & 0xFF);
}

int SC16IS752::pwmWriteHR(uint8_t pin, uint32_t value)
{
    if (!isValidPin(pin) || !_pwm_configs[pin].enabled) {
        return -EINVAL;
    }

    // Direct write without scaling
    uint32_t max_value = getPWMMaxValue(pin);
    value = math::min(value, max_value);

    _pwm_configs[pin].duty_cycle = value;

    // Write duty cycle registers directly
    int ret = writeRegister(REG_PWM_DUTY + (pin * 2), value & 0xFF);
    if (ret != PX4_OK) {
        return ret;
    }

    return writeRegister(REG_PWM_DUTY + (pin * 2) + 1, (value >> 8) & 0xFF);
}

int SC16IS752::pwmEnd(uint8_t pin)
{
    if (!isValidPin(pin)) {
        return -EINVAL;
    }

    // Disable PWM mode
    uint8_t pwm_ctrl = 0;
    int ret = readRegister(REG_PWM_CTRL, pwm_ctrl);
    if (ret != PX4_OK) {
        return ret;
    }

    pwm_ctrl &= ~(1 << pin);  // Disable PWM for this pin
    ret = writeRegister(REG_PWM_CTRL, pwm_ctrl);
    if (ret != PX4_OK) {
        return ret;
    }

    // Reset configuration
    _pwm_configs[pin].enabled = false;
    _pwm_configs[pin].duty_cycle = 0;

    // Return pin to normal GPIO output mode
    ret = setPinMode(pin, GPIO_OUTPUT);
    if (ret != PX4_OK) {
        return ret;
    }

    return writePin(pin, false);
}

int SC16IS752::analogWrite(uint8_t pin, uint8_t value)
{
    // Initialize PWM if not already enabled
    if (!_pwm_configs[pin].enabled) {
        int ret = pwmBegin(pin);
        if (ret != PX4_OK) {
            return ret;
        }
    }

    // Convert 8-bit value to current resolution
    uint32_t max_value = getPWMMaxValue(pin);
    uint32_t scaled_value = (value * max_value) / 255;

    return pwmWrite(pin, scaled_value);
}

uint32_t SC16IS752::getPWMMaxValue(uint8_t pin) const
{
    if (!isValidPin(pin) || !_pwm_configs[pin].enabled) {
        return 0;
    }

    return (1UL << static_cast<uint8_t>(_pwm_configs[pin].resolution)) - 1;
}

bool SC16IS752::isPWMEnabled(uint8_t pin) const
{
    return isValidPin(pin) && _pwm_configs[pin].enabled;
}

int SC16IS752::updatePWMPrescaler(uint8_t pin)
{
    if (!isValidPin(pin) || !_pwm_configs[pin].enabled) {
        return -EINVAL;
    }

    uint32_t prescaler = calculatePWMPrescaler(_pwm_configs[pin].frequency,
                                             _pwm_configs[pin].resolution);

    // Write prescaler registers
    int ret = writeRegister(REG_PWM_PRE + (pin * 2), prescaler & 0xFF);
    if (ret != PX4_OK) {
        return ret;
    }
    return writeRegister(REG_PWM_PRE + (pin * 2) + 1, (prescaler >> 8) & 0xFF);
}

int SC16IS752::updatePWMSettings(uint8_t pin)
{
    if (!isValidPin(pin) || !_pwm_configs[pin].enabled) {
        return -EINVAL;
    }

    uint8_t pwm_ctrl = 0;
    int ret = readRegister(REG_PWM_CTRL, pwm_ctrl);
    if (ret != PX4_OK) {
        return ret;
    }

    // Clear resolution bits for this pin (2 bits per pin)
    uint8_t pin_shift = (pin * 2);
    pwm_ctrl &= ~(0x03 << pin_shift);

    // Set new resolution
    uint8_t res_value = 0;
    switch (_pwm_configs[pin].resolution) {
        case PWMResolution::PWM_8_BIT: res_value = 0; break;
        case PWMResolution::PWM_10_BIT: res_value = 1; break;
        case PWMResolution::PWM_12_BIT: res_value = 2; break;
        case PWMResolution::PWM_16_BIT: res_value = 3; break;
    }

    pwm_ctrl |= (res_value << pin_shift);
    return writeRegister(REG_PWM_CTRL, pwm_ctrl);
}

int SC16IS752::configurePWMPin(uint8_t pin)
{
    if (!isValidPin(pin)) {
        return -EINVAL;
    }

    // Enable PWM functionality for the pin
    uint8_t pwm_ctrl = 0;
    int ret = readRegister(REG_PWM_CTRL, pwm_ctrl);
    if (ret != PX4_OK) {
        return ret;
    }

    pwm_ctrl |= (1 << pin);  // Enable PWM for this pin
    return writeRegister(REG_PWM_CTRL, pwm_ctrl);
}

uint32_t SC16IS752::calculatePWMPrescaler(uint32_t target_freq, PWMResolution resolution) const
{
    uint32_t steps = 1UL << static_cast<uint8_t>(resolution);
    uint32_t prescaler = (XTAL_FREQ / (target_freq * steps)) - 1;
    return math::min(prescaler, 65535UL);
}

// Helper Functions Implementation

void SC16IS752::updateGPIORegister(uint8_t reg, uint8_t pin, uint8_t value)
{
    uint8_t current_val = readGPIORegister(reg);

    if (value) {
        current_val |= (1 << pin);
    } else {
        current_val &= ~(1 << pin);
    }

    writeRegister(reg, current_val);
}

uint8_t SC16IS752::readGPIORegister(uint8_t reg)
{
    uint8_t val = 0;
    readRegister(reg, val);
    return val;
}

bool SC16IS752::waitForTxEmpty(uint8_t channel, uint32_t timeout_ms)
{
    const uint64_t start = hrt_absolute_time();

    while (hrt_elapsed_time(&start) < timeout_ms * 1000) {
        if (isTxEmpty(channel)) {
            return true;
        }
        px4_usleep(100);
    }

    return false;
}

bool SC16IS752::waitForRxData(uint8_t channel, uint32_t timeout_ms)
{
    const uint64_t start = hrt_absolute_time();

    while (hrt_elapsed_time(&start) < timeout_ms * 1000) {
        if (isRxAvailable(channel)) {
            return true;
        }
        px4_usleep(100);
    }

    return false;
}

int SC16IS752::readRegister(uint8_t reg, uint8_t &val)
{
    uint8_t cmd[] = { reg };
    return transfer(&cmd[0], 1, &val, 1);
}

int SC16IS752::writeRegister(uint8_t reg, uint8_t val)
{
    uint8_t cmd[] = { reg, val };
    return transfer(&cmd[0], 2, nullptr, 0);
}

uint8_t SC16IS752::regAddr(uint8_t reg, uint8_t channel) const
{
    return (reg << 3) | (channel ? 0x02 : 0x00);
}

bool SC16IS752::isValidChannel(uint8_t channel) const
{
    return channel < MAX_CHANNELS;
}

bool SC16IS752::isValidPin(uint8_t pin) const
{
    return pin < MAX_GPIO_PINS;
}

void SC16IS752::print_status()
{
    I2CSPIDriver::print_status();

    PX4_INFO("Device Status:");
    PX4_INFO("  Initialized: %s", _initialized ? "YES" : "NO");
    PX4_INFO("  Current Baudrate: %u", _current_baudrate);

    // UART Status
    for (uint8_t channel = 0; channel < MAX_CHANNELS; channel++) {
        UARTStatus status = getStatus(channel);
        PX4_INFO("Channel %d:", channel);
        PX4_INFO("  LSR: 0x%02X", status.lsr);
        PX4_INFO("  MSR: 0x%02X", status.msr);
        PX4_INFO("  TX Level: %u", status.txlvl);
        PX4_INFO("  RX Level: %u", status.rxlvl);
    }

    // GPIO Status
    PX4_INFO("GPIO Status:");
    PX4_INFO("  Direction: 0x%02X", _gpio_direction);
    PX4_INFO("  State: 0x%02X", _gpio_state);

    // PWM Status
    PX4_INFO("PWM Status:");
    for (uint8_t i = 0; i < MAX_GPIO_PINS; i++) {
        if (_pwm_configs[i].enabled) {
            PX4_INFO("  Pin %d: %u Hz, Resolution: %u-bit, Duty: %u",
                    i, _pwm_configs[i].frequency,
                    static_cast<unsigned>(_pwm_configs[i].resolution),
                    _pwm_configs[i].duty_cycle);
        }
    }

    // Performance Counters
    perf_print_counter(_comms_errors);
    perf_print_counter(_sample_perf);
    perf_print_counter(_transfer_perf);
}

// Module Interface Implementation

namespace
{
static void print_usage()
{
    PX4_INFO("usage: sc16is752 <command> [options]");
    PX4_INFO("commands:");
    PX4_INFO("  start [-b <bus>] [-a <addr>] [-R <rot>]");
    PX4_INFO("  stop");
    PX4_INFO("  status");
    PX4_INFO("  test");
    PX4_INFO("\noptions:");
    PX4_INFO("  -b <bus>          : I2C bus number (default=1)");
    PX4_INFO("  -a <addr>         : I2C address (default=0x4D)");
    PX4_INFO("  -R <rotation>     : Rotation (default=0)");
}

static int custom_command(int argc, char *argv[])
{
    return print_usage();
}

static int sc16is752_main_internal(int argc, char *argv[])
{
    using ThisDriver = SC16IS752;
    BusCLIArguments cli{true, false};
    cli.default_i2c_frequency = ThisDriver::DEFAULT_I2C_FREQUENCY;

    const char *verb = cli.parseDefaultArguments(argc, argv);
    if (!verb) {
        return -1;
    }

    BusInstanceIterator iterator(MODULE_NAME, cli, DRV_UART_DEVTYPE_SC16IS752);

    if (!strcmp(verb, "start")) {
        return ThisDriver::module_start(cli, iterator);
    }

    if (!strcmp(verb, "stop")) {
        return ThisDriver::module_stop(iterator);
    }

    if (!strcmp(verb, "status")) {
        return ThisDriver::module_status(iterator);
    }

    if (!strcmp(verb, "test")) {
        return ThisDriver::test(iterator);
    }

    print_usage();
    return -1;
}

} // namespace

extern "C" __EXPORT int sc16is752_main(int argc, char *argv[])
{
    return sc16is752_main_internal(argc, argv);
}

// Need to implement the static instantiation method
static I2CSPIDriverBase *SC16IS752::instantiate(const I2CSPIDriverConfig &config, int runtime_instance) {
    return new SC16IS752(config);
}

// Need to implement GPIO state update
void SC16IS752::updateGPIOState(uint8_t pin, bool is_input, uint8_t state) {
    if (isValidPin(pin)) {
        _gpio_states[pin].is_input = is_input;
        _gpio_states[pin].state = state;
    }
}

// Need to implement PWM pin check
bool SC16IS752::isPWMPin(uint8_t pin) const {
    return isValidPin(pin) && _pwm_configs[pin].enabled;
}

