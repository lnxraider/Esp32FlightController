/**
 * @file sc16is752_gps.cpp
 * @brief Enhanced GPS driver implementation with improved safety and robustness
 */

#include "sc16is752_gps.hpp"
#include <px4_platform_common/log.h>
#include <cstring>

using namespace time_literals;

// Static interface methods
I2CSPIDriverBase *SC16IS752_GPS::instantiate(const I2CSPIDriverConfig &config,
                                            int runtime_instance)
{
    return new SC16IS752_GPS(config);
}

void SC16IS752_GPS::print_usage()
{
    PX4_INFO("Usage: sc16is752_gps <command> [arguments...]");
    PX4_INFO("Commands:");
    PX4_INFO("\tstart");
    PX4_INFO("\tstop");
    PX4_INFO("\ttest");
    PX4_INFO("\tinfo");
    PX4_INFO("\treset");
    PX4_INFO("\tstatus");
}

// Constructor
SC16IS752_GPS::SC16IS752_GPS(const I2CSPIDriverConfig &config) :
    SC16IS752(config),
    ModuleParams(nullptr)
{
    // Initialize parameters
    _param_baudrate = param_find("GPS_BAUD");
    _param_gps_type = param_find("GPS_TYPE");
    _param_rate = param_find("GPS_RATE");

    // Initialize performance counters
    _sample_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": sample");
    _comms_errors = perf_alloc(PC_COUNT, MODULE_NAME": com_err");
    _transfer_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": transfer");
}

// Destructor
SC16IS752_GPS::~SC16IS752_GPS()
{
    // Stop GPS updates and cleanup
    stopGPS();

    // Free performance counters
    perf_free(_sample_perf);
    perf_free(_comms_errors);
    perf_free(_transfer_perf);
}

void SC16IS752_GPS::stopGPS()
{
    std::lock_guard<std::mutex> lock(_mutex);

    if (_state.configured) {
        // Send protocol-specific stop commands
        uint8_t stop_cmd[GPS_TX_BUFFER_SIZE];
        size_t cmd_len = 0;

        switch (_config.protocol) {
            case GPSProtocol::UBX:
                cmd_len = buildUBXStopCommand(stop_cmd);
                break;
            case GPSProtocol::NMEA:
                cmd_len = buildNMEAStopCommand(stop_cmd);
                break;
            case GPSProtocol::SIRF:
                cmd_len = buildSiRFStopCommand(stop_cmd);
                break;
            default:
                break;
        }

        if (cmd_len > 0) {
            writeUART(stop_cmd, cmd_len);
            px4_usleep(COMMAND_WAIT_TIME_US);
        }
    }
}

size_t SC16IS752_GPS::buildUBXStopCommand(uint8_t *buffer)
{
    if (!buffer) return 0;

    // UBX stop command
    static const uint8_t cmd[] = {
        0xB5, 0x62,  // Header
        0x06, 0x00,  // Class/ID
        0x00, 0x00,  // Length
        0x06, 0x18   // Checksum
    };
    
    memcpy(buffer, cmd, sizeof(cmd));
    return sizeof(cmd);
}

size_t SC16IS752_GPS::buildNMEAStopCommand(uint8_t *buffer)
{
    if (!buffer) return 0;

    // NMEA stop command
    static const char *cmd = "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
    const size_t len = strlen(cmd);
    memcpy(buffer, cmd, len);
    return len;
}

size_t SC16IS752_GPS::buildSiRFStopCommand(uint8_t *buffer)
{
    if (!buffer) return 0;

    // SiRF stop command
    static const uint8_t cmd[] = {
        0xA0, 0xA2,  // Header
        0x00, 0x02,  // Length
        0x00, 0x00,  // Command
        0x00, 0x02,  // Checksum
        0xB0, 0xB3   // End sequence
    };
    
    memcpy(buffer, cmd, sizeof(cmd));
    return sizeof(cmd);
}

bool SC16IS752_GPS::validateGPSPacket(const uint8_t* data, size_t length)
{
    if (!data || length < MIN_PACKET_SIZE || length > MAX_PACKET_SIZE) {
        return false;
    }

    switch (_config.protocol) {
        case GPSProtocol::UBX:
            return validateUBXPacket(data, length);
        case GPSProtocol::NMEA:
            return validateNMEAPacket(data, length);
        case GPSProtocol::SIRF:
            return validateSiRFPacket(data, length);
        default:
            return false;
    }
}

bool SC16IS752_GPS::validateUBXPacket(const uint8_t* data, size_t length)
{
    if (length < 8) return false;  // Minimum UBX packet size

    // Check header
    if (data[0] != 0xB5 || data[1] != 0x62) return false;

    // Get payload length
    uint16_t payload_length = (data[5] << 8) | data[4];
    if (length != payload_length + 8) return false;  // Total length check

    // Verify checksum
    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 2; i < length - 2; i++) {
        ck_a += data[i];
        ck_b += ck_a;
    }

    return (ck_a == data[length-2] && ck_b == data[length-1]);
}

bool SC16IS752_GPS::validateNMEAPacket(const uint8_t* data, size_t length)
{
    if (length < 10) return false;  // Minimum NMEA sentence size
    
    // Check start character
    if (data[0] != '$') return false;

    // Find checksum position
    size_t asterisk_pos = 0;
    for (size_t i = length-5; i < length; i++) {
        if (data[i] == '*') {
            asterisk_pos = i;
            break;
        }
    }
    if (asterisk_pos == 0) return false;

    // Calculate checksum
    uint8_t checksum = 0;
    for (size_t i = 1; i < asterisk_pos; i++) {
        checksum ^= data[i];
    }

    // Convert expected checksum from hex chars
    char expected_str[3] = {(char)data[asterisk_pos+1], 
                           (char)data[asterisk_pos+2], 0};
    char* end;
    uint8_t expected = strtol(expected_str, &end, 16);

    return checksum == expected;
}

bool SC16IS752_GPS::validateSiRFPacket(const uint8_t* data, size_t length)
{
    if (length < 10) return false;  // Minimum SiRF packet size

    // Check start and end sequences
    if (data[0] != 0xA0 || data[1] != 0xA2) return false;
    if (data[length-2] != 0xB0 || data[length-1] != 0xB3) return false;

    // Get payload length
    uint16_t payload_length = (data[3] << 8) | data[2];
    if (length != payload_length + 8) return false;

    // Calculate checksum
    uint16_t checksum = 0;
    for (size_t i = 4; i < length-4; i++) {
        checksum += data[i];
    }
    checksum &= 0x7FFF;

    uint16_t expected = (data[length-4] << 8) | data[length-3];
    return checksum == expected;
}

bool SC16IS752_GPS::processIncomingData()
{
    std::lock_guard<std::mutex> lock(_uart_mutex);
    
    UARTStatus status = getStatus(GPS_UART_CHANNEL);
    if (status.error != 0) {
        setError(GPSError::UART_ERROR);
        return false;
    }

    if (status.rxlvl == 0) {
        return true;  // No data to process
    }

    perf_begin(_sample_perf);

    size_t bytes_read = readUART(_rx_buffer.get(), GPS_RX_BUFFER_SIZE);
    if (bytes_read == 0) {
        perf_end(_sample_perf);
        return false;
    }

    bool success = processGPSPacket(_rx_buffer.get(), bytes_read);
    
    perf_end(_sample_perf);
    return success;
}

void SC16IS752_GPS::publishGPSData()
{
    if (!_gps_helper) return;

    sensor_gps_s report{};
    _gps_helper->getReport(report);

    report.timestamp = hrt_absolute_time();
    report.device_id = get_device_id();

    _sensor_gps_pub.publish(report);
}

bool SC16IS752_GPS::validatePowerModeTransition(PowerMode new_mode)
{
    const PowerMode current_mode = _state.power_mode;

    // Check valid transitions
    switch (current_mode) {
        case PowerMode::FULL:
            return true;  // Can transition to any mode
            
        case PowerMode::ECO:
            return new_mode == PowerMode::FULL || 
                   new_mode == PowerMode::STANDBY;
            
        case PowerMode::STANDBY:
            return new_mode == PowerMode::FULL;
            
        case PowerMode::POWER_SAVE:
            return new_mode == PowerMode::FULL;
            
        default:
            return false;
    }
}

bool SC16IS752_GPS::configurePowerMode(PowerMode mode)
{
    bool success = false;
    
    switch (mode) {
        case PowerMode::FULL:
            success = configureFullPower();
            break;
            
        case PowerMode::ECO:
            success = configureEcoPower();
            break;
            
        case PowerMode::STANDBY:
            success = configureStandby();
            break;
            
        case PowerMode::POWER_SAVE:
            success = configurePowerSave();
            break;
    }

    if (success) {
        _state.power_mode = mode;
    }

    return success;
}

bool SC16IS752_GPS::configureFullPower()
{
    // Reset if coming from low power mode
    if (_state.power_mode == PowerMode::STANDBY || 
        _state.power_mode == PowerMode::POWER_SAVE) {
        if (!resetGPS()) {
            return false;
        }
    }

    // Configure for maximum update rate
    _config.rate_hz = DEFAULT_RATE;
    return configureGPS();
}

bool SC16IS752_GPS::configureEcoPower()
{
    // Reduce update rate
    _config.rate_hz = DEFAULT_RATE / 2;
    return configureGPS();
}

bool SC16IS752_GPS::configureStandby()
{
    std::lock_guard<std::mutex> lock(_uart_mutex);
    
    uint8_t cmd[GPS_TX_BUFFER_SIZE];
    size_t cmd_len = 0;

    switch (_config.protocol) {
        case GPSProtocol::UBX:
            cmd_len = buildUBXStandbyCommand(cmd);
            break;
        case GPSProtocol::NMEA:
            cmd_len = buildNMEAStandbyCommand(cmd);
            break;
        case GPSProtocol::SIRF:
            cmd_len = buildSiRFStandbyCommand(cmd);
            break;
        default:
            return false;
    }

    return (cmd_len > 0) && writeUART(cmd, cmd_len);
}

bool SC16IS752_GPS::configurePowerSave()
{
    // Similar to standby but with deeper sleep
    return configureStandby();
}

bool SC16IS752_GPS::resetGPS()
{
    std::lock_guard<std::mutex> lock(_uart_mutex);
    
    // Try hardware reset first
    if (setGPIOState(GPS_RESET_PIN, false) == PX4_OK) {
        px4_usleep(RESET_HOLD_TIME_US);
        
        if (setGPIOState(GPS_RESET_PIN, true) != PX4_OK) {
            setError(GPSError::RESET_FAILED);
            return false;
        }
        
        px4_usleep(RESET_WAIT_TIME_US);
        return true;
    }

    // Fall back to software reset
    uint8_t cmd[GPS_TX_BUFFER_SIZE];
    size_t cmd_len = 0;

    switch (_config.protocol) {
        case GPSProtocol::UBX:
            cmd_len = buildUBXResetCommand(cmd);
            break;
        case GPSProtocol::NMEA:
            cmd_len = buildNMEAResetCommand(cmd);
            break;
        case GPSProtocol::SIRF:
            cmd_len = buildSiRFResetCommand(cmd);
            break;
        default:
            return false;
    }

    if (cmd_len == 0 || !writeUART(cmd, cmd_len)) {
        setError(GPSError::RESET_FAILED);
        return false;
    }

    px4_usleep(RESET_WAIT_TIME_US);
    _state.configured = false;
    return true;
}

void SC16IS752_GPS::checkGPSHealth()
{
    const uint64_t now = hrt_absolute_time();

    // Check for timeout
    if (now - _state.last_message_time > DATA_TIMEOUT_MS * 1000) {
        setError(GPSError::TIMEOUT);
        _health.outage_count++;
        handleRecoveryState();
    }

    // Update statistics
    if (_gps_helper) {
        sensor_gps_s report{};
        _gps_helper->getReport(report);

        _health.signal_lock = (report.fix_type >= 2);
        _health.position_fix = (report.fix_type >= 3);
        _health.differential_fix = (report.fix_type == 4);
        _health.satellite_count = report.satellites_used;
        _health.satellites_used = report.satellites_used;
        _health.signal_quality = report.noise_per_ms;
        _health.last_fix_time = report.timestamp;
    }

    // Monitor error rates and trigger recovery if needed
    if (_health.outage_count > MAX_OUTAGE_COUNT ||
        _state.error_count > MAX_ERROR_COUNT) {
        handleRecoveryState();
    }
}

void SC16IS752_GPS::handleRecoveryState()
{
    std::lock_guard<std::mutex> lock(_mutex);

    if (_recovery.recovery_in_progress) {
        return;  // Recovery already in progress
    }

    _recovery.recovery_in_progress = true;
    _recovery.recovery_attempts++;

    if (_recovery.recovery_attempts > MAX_RECOVERY_ATTEMPTS) {
        PX4_ERR("GPS recovery failed after %d attempts", MAX_RECOVERY_ATTEMPTS);
        _recovery.recovery_in_progress = false;
        return;
    }

    // Attempt recovery sequence
    bool recovered = false;

    // Step 1: Try soft reset
    if (resetGPS()) {
        // Wait for initialization
        px4_usleep(RESET_WAIT_TIME_US);

        // Step 2: Reconfigure GPS
        if (configureGPS()) {
            // Step 3: Verify communication
            recovered = performSelfTest();
        }
    }

    if (recovered) {
        _recovery.successful_recoveries++;
        resetGPSStats();
    } else {
        // If soft recovery failed, try power cycling
        if (setPowerMode(PowerMode::POWER_SAVE) &&
            px4_usleep(POWER_CYCLE_WAIT_US) == 0 &&
            setPowerMode(PowerMode::FULL)) {

            recovered = performSelfTest();
            if (recovered) {
                _recovery.successful_recoveries++;
                resetGPSStats();
            }
        }
    }

    _recovery.recovery_in_progress = false;
    _recovery.last_recovery_time = hrt_absolute_time();
}

bool SC16IS752_GPS::performSelfTest()
{
    std::lock_guard<std::mutex> lock(_uart_mutex);

    // Check UART status
    UARTStatus status = getStatus(GPS_UART_CHANNEL);
    if (status.error != 0) {
        return false;
    }

    // Send test command based on protocol
    uint8_t test_cmd[GPS_TX_BUFFER_SIZE];
    size_t cmd_len = 0;
    bool response_required = true;

    switch (_config.protocol) {
        case GPSProtocol::UBX:
            cmd_len = buildUBXPollVersionCommand(test_cmd);
            break;
        case GPSProtocol::NMEA:
            cmd_len = buildNMEAPollVersionCommand(test_cmd);
            break;
        case GPSProtocol::SIRF:
            cmd_len = buildSiRFPollVersionCommand(test_cmd);
            break;
        default:
            return false;
    }

    if (cmd_len == 0 || !writeUART(test_cmd, cmd_len)) {
        return false;
    }

    if (!response_required) {
        return true;
    }

    // Wait for response
    return waitForAck(SELF_TEST_TIMEOUT_MS);
}

bool SC16IS752_GPS::waitForAck(uint32_t timeout_ms)
{
    const uint64_t start_time = hrt_absolute_time();
    const uint64_t timeout_us = timeout_ms * 1000ULL;

    while (hrt_elapsed_time(&start_time) < timeout_us) {
        UARTStatus status = getStatus(GPS_UART_CHANNEL);

        if (status.error != 0) {
            return false;
        }

        if (status.rxlvl > 0) {
            uint8_t buffer[GPS_RX_BUFFER_SIZE];
            size_t bytes_read = readUART(buffer, math::min(status.rxlvl, sizeof(buffer)));

            if (bytes_read > 0 && isAckPacket(buffer, bytes_read)) {
                return true;
            }
        }

        px4_usleep(10000);  // 10ms delay between checks
    }

    return false;
}

bool SC16IS752_GPS::isAckPacket(const uint8_t* data, size_t length)
{
    if (!data || length == 0) {
        return false;
    }

    switch (_config.protocol) {
        case GPSProtocol::UBX:
            return isUBXAck(data, length);
        case GPSProtocol::NMEA:
            return isNMEAAck(data, length);
        case GPSProtocol::SIRF:
            return isSiRFAck(data, length);
        default:
            return false;
    }
}

void SC16IS752_GPS::updateGPSConfiguration()
{
    std::lock_guard<std::mutex> lock(_config_mutex);

    bool config_changed = false;
    int32_t val = 0;

    // Update baudrate
    if (param_get(_param_baudrate, &val) == PX4_OK) {
        if (validateBaudRate(val) && static_cast<uint32_t>(val) != _config.baudrate) {
            _config.baudrate = val;
            config_changed = true;
        }
    }

    // Update GPS protocol
    if (param_get(_param_gps_type, &val) == PX4_OK) {
        GPSProtocol new_protocol;
        switch (val) {
            case 1: new_protocol = GPSProtocol::UBX; break;
            case 2: new_protocol = GPSProtocol::NMEA; break;
            case 3: new_protocol = GPSProtocol::SIRF; break;
            default: new_protocol = GPSProtocol::AUTO; break;
        }

        if (new_protocol != _config.protocol) {
            _config.protocol = new_protocol;
            config_changed = true;
        }
    }

    // Update update rate
    if (param_get(_param_rate, &val) == PX4_OK) {
        if (validateUpdateRate(val) && static_cast<uint8_t>(val) != _config.rate_hz) {
            _config.rate_hz = val;
            config_changed = true;
        }
    }

    if (config_changed) {
        configureGPSWithRetry();
    }
}

bool SC16IS752_GPS::configureGPSWithRetry()
{
    std::lock_guard<std::mutex> lock(_config_mutex);

    for (uint32_t attempt = 0; attempt < MAX_CONFIG_ATTEMPTS; attempt++) {
        if (configureGPS()) {
            return true;
        }

        // Wait before retry
        px4_usleep(CONFIG_RETRY_DELAY_US);
    }

    setError(GPSError::CONFIG_INVALID);
    return false;
}

bool SC16IS752_GPS::configureGPS()
{
    if (!_gps_helper) {
        return false;
    }

    bool success = false;

    switch (_config.protocol) {
        case GPSProtocol::UBX:
            success = configureUBX();
            break;
        case GPSProtocol::NMEA:
            success = configureNMEA();
            break;
        case GPSProtocol::SIRF:
            success = configureSiRF();
            break;
        case GPSProtocol::AUTO:
            success = detectAndConfigureProtocol();
            break;
    }

    if (success) {
        _state.configured = true;
        _state.last_config_check = hrt_absolute_time();
    }

    return success;
}

bool SC16IS752_GPS::detectAndConfigureProtocol()
{
    // Try each protocol in sequence
    static const GPSProtocol protocols[] = {
        GPSProtocol::UBX,
        GPSProtocol::NMEA,
        GPSProtocol::SIRF
    };

    for (const auto protocol : protocols) {
        _config.protocol = protocol;

        // Configure UART for current protocol
        if (configureCOM(GPS_UART_CHANNEL, _config.baudrate) != PX4_OK) {
            continue;
        }

        // Try to configure GPS
        if (configureGPS()) {
            return true;
        }

        px4_usleep(PROTOCOL_DETECT_DELAY_US);
    }

    return false;
}

void SC16IS752_GPS::print_status()
{
    PX4_INFO("GPS Status:");
    PX4_INFO("===========");

    const char* protocol_str = "Unknown";
    switch (_config.protocol) {
        case GPSProtocol::UBX: protocol_str = "UBX"; break;
        case GPSProtocol::NMEA: protocol_str = "NMEA"; break;
        case GPSProtocol::SIRF: protocol_str = "SiRF"; break;
        case GPSProtocol::AUTO: protocol_str = "Auto"; break;
    }

    PX4_INFO("Protocol: %s", protocol_str);
    PX4_INFO("Baudrate: %d", _config.baudrate);
    PX4_INFO("Update rate: %d Hz", _config.rate_hz);
    PX4_INFO("Configured: %s", _state.configured ? "Yes" : "No");
    PX4_INFO("Satellites used: %d", _health.satellites_used);
    PX4_INFO("Signal quality: %.1f%%", _health.signal_quality);
    PX4_INFO("Fix type: %s",
             _health.differential_fix ? "DGPS" :
             _health.position_fix ? "3D" :
             _health.signal_lock ? "2D" : "None");
    PX4_INFO("Errors: %d", _state.error_count.load());
    PX4_INFO("Last error: %s", getErrorString(_state.last_error));
    PX4_INFO("Recovery attempts: %d", _recovery.recovery_attempts);
    PX4_INFO("Successful recoveries: %d", _recovery.successful_recoveries);

    // Print performance counters
    perf_print_counter(_sample_perf);
    perf_print_counter(_comms_errors);
    perf_print_counter(_transfer_perf);
}

// Main entry point
extern "C" __EXPORT int sc16is752_gps_main(int argc, char *argv[])
{
    return SC16IS752_GPS::main(argc, argv);
}


