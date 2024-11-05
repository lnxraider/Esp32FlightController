/**
 * @file sc16is752_gps.hpp
 * @brief Complete GPS implementation using SC16IS752 UART with enhanced features
 */

#pragma once

#include "sc16is752.hpp"
#include <drivers/gps/gps_helper.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/gps_dump.h>
#include <uORB/topics/gps_inject_data.h>
#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module_params.h>
#include <mathlib/mathlib.h>

using namespace time_literals;

/**
 * @class SC16IS752_GPS
 * @brief GPS driver implementation using SC16IS752 UART
 */
class SC16IS752_GPS : public SC16IS752, public ModuleParams
{
public:
    // Device configuration
    static constexpr uint32_t DEFAULT_BAUDRATE = 9600;
    static constexpr uint32_t DEFAULT_RATE = 5;
    static constexpr uint32_t MAX_RATE = 10;
    static constexpr size_t GPS_RX_BUFFER_SIZE = 256;
    static constexpr size_t GPS_TX_BUFFER_SIZE = 128;
    static constexpr size_t GPS_CONFIG_BUFFER_SIZE = 128;
    static constexpr uint8_t GPS_SAT_INFO_MAX_SATELLITES = 20;
    static constexpr uint8_t GPS_UART_CHANNEL = 0;

    // Timeouts and intervals
    static constexpr uint64_t HEALTH_CHECK_INTERVAL = 1_s;
    static constexpr uint64_t CONFIG_CHECK_INTERVAL = 5_s;
    static constexpr uint64_t RECOVERY_WAIT_INTERVAL = 2_s;
    static constexpr uint32_t DATA_TIMEOUT_MS = 2000;
    static constexpr uint32_t ACK_TIMEOUT_MS = 1000;
    static constexpr uint32_t RECOVERY_TIMEOUT_MS = 5000;

    /** @brief Supported GPS protocols */
    enum class GPSProtocol : uint8_t {
        UBX = 0,    ///< u-blox binary protocol
        NMEA = 1,   ///< NMEA protocol
        SIRF = 2,   ///< SiRF binary protocol
        AUTO = 3    ///< Auto-detect protocol
    };

    /** @brief Power management modes */
    enum class PowerMode : uint8_t {
        FULL,       ///< Full power operation
        ECO,        ///< Reduced power, standard operation
        STANDBY,    ///< Minimal power, quick recovery
        POWER_SAVE  ///< Lowest power, slow recovery
    };

    /** @brief GPS configuration */
    struct GPSConfig {
        GPSProtocol protocol{GPSProtocol::AUTO};
        uint32_t baudrate{DEFAULT_BAUDRATE};
        uint8_t rate_hz{DEFAULT_RATE};
    };

    /** @brief GPS health status */
    struct GPSHealth {
        bool signal_lock{false};
        bool position_fix{false};
        bool differential_fix{false};
        uint8_t satellite_count{0};
        uint8_t satellites_used{0};
        float signal_quality{0.0f};
        uint32_t outage_count{0};
        uint64_t last_fix_time{0};
    };

    /** @brief Recovery status */
    struct RecoveryStatus {
        uint32_t recovery_attempts{0};
        uint32_t successful_recoveries{0};
        uint64_t last_recovery_time{0};
        bool recovery_in_progress{false};
    };

    /**
     * @brief Constructor
     * @param config Driver configuration
     */
    SC16IS752_GPS(const I2CSPIDriverConfig &config);

    /**
     * @brief Destructor
     */
    ~SC16IS752_GPS() override;

    // Static methods
    static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config,
                                       int runtime_instance);
    static void print_usage();

    // Virtual method overrides
    int init() override;
    void print_status() override;

    // Main processing methods
    void RunImpl() override;
    void parametersUpdate();
    void updateParams() override;

    /**
     * @brief Configure the GPS receiver
     * @return true if successful
     */
    bool configureGPS();

    /**
     * @brief Detect the GPS protocol automatically
     * @return true if protocol detected
     */
    bool detectGPSProtocol();

    /**
     * @brief Configure GPS for UBX protocol
     * @return true if successful
     */
    bool configureUBX();

    /**
     * @brief Configure GPS for NMEA protocol
     * @return true if successful
     */
    bool configureNMEA();

    /**
     * @brief Configure GPS for SiRF protocol
     * @return true if successful
     */
    bool configureSiRF();

    /**
     * @brief Update GPS configuration
     */
    void updateGPSConfiguration();

    /**
     * @brief Check current GPS configuration
     */
    void checkGPSConfiguration();

    // Data handling methods
    void processGPSData(const uint8_t *buffer, size_t length);
    void collectGPSSatInfo();
    void updateGPSRate();
    void handleGPSInject();

    /**
     * @brief Send configuration to GPS
     * @param cfg Configuration buffer
     * @param length Buffer length
     * @return true if successful
     */
    bool sendGPSConfig(const uint8_t *cfg, size_t length);

    /**
     * @brief Wait for acknowledgment from GPS
     * @param timeout_ms Timeout in milliseconds
     * @return true if ack received
     */
    bool waitForAck(uint32_t timeout_ms);

    // Power management methods
    bool setPowerMode(PowerMode mode);
    bool handlePowerState();
    PowerMode getCurrentPowerMode() const { return _power_mode; }

    // Error recovery methods
    bool attemptRecovery();
    bool resetGPS();
    bool performSelfTest();
    void checkGPSHealth();

    // Debug and monitoring methods
    void printGPSInfo();
    void dumpConfiguration();
    void dumpGPSData(const uint8_t *data, size_t len);
    void resetGPSStats();

protected:
    // Protected helper methods
    bool validateConfiguration() const;
    bool isHealthy() const;
    void updateHealth();
    void logError(const char *msg);
    void logInfo(const char *msg);

    // Utility methods
    bool validateProtocol(GPSProtocol protocol) const;
    bool validateBaudRate(uint32_t baudrate) const;
    bool validateUpdateRate(uint8_t rate) const;
    uint32_t calculateChecksum(const uint8_t *buffer, size_t length) const;

private:
    // GPS helper and configuration
    GPSHelper *_gps_helper{nullptr};
    GPSConfig _config{};
    GPSHealth _health{};
    RecoveryStatus _recovery{};
    PowerMode _power_mode{PowerMode::FULL};

    // Performance counters
    perf_counter_t _sample_perf{nullptr};
    perf_counter_t _comms_errors{nullptr};
    perf_counter_t _transfer_perf{nullptr};

    // uORB publications
    uORB::Publication<sensor_gps_s> _sensor_gps_pub{ORB_ID(sensor_gps)};
    uORB::Publication<gps_dump_s> _gps_dump_pub{ORB_ID(gps_dump)};

    // uORB subscriptions
    uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
    uORB::Subscription _gps_inject_data_sub{ORB_ID(gps_inject_data)};

    // Parameters
    param_t _param_baudrate{PARAM_INVALID};
    param_t _param_gps_type{PARAM_INVALID};
    param_t _param_rate{PARAM_INVALID};

    // Buffers
    uint8_t _rx_buffer[GPS_RX_BUFFER_SIZE];
    uint8_t _tx_buffer[GPS_TX_BUFFER_SIZE];
    uint8_t _cfg_buffer[GPS_CONFIG_BUFFER_SIZE];

    // Statistics and state tracking
    float _receiver_rate{0.0f};
    size_t _last_rate_count{0};
    uint64_t _last_rate_time{0};
    uint64_t _last_config_check{0};
    uint64_t _last_message_time{0};
    uint64_t _last_inject_time{0};
    uint64_t _last_health_check{0};
    bool _configured{false};
    bool _receive_in_progress{false};
    uint8_t _sat_info[GPS_SAT_INFO_MAX_SATELLITES];

    // Utility methods
    void initializeBuffers();
    void handleRecoveryState();
};

