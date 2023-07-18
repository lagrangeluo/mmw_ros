#ifndef MMW_BASE_HPP
#define MMW_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <functional>
#include <atomic>

#include "mmw_sdk/async_io/async_can.hpp"
#include "mmw_sdk/async_io/async_serial.hpp"

#include "mmw_protocol.h"
#include "mmw_can_parser.h"
#include "mmw_types.hpp"

namespace wescore
{
class MmwBase
{
public:
    MmwBase() = default;
    ~MmwBase();

    // do not allow copy
    MmwBase(const MmwBase &mmw) = delete;
    MmwBase &operator=(const MmwBase &mmw) = delete;

public:
    // connect to roboot from CAN or serial
    void Connect(std::string dev_name, int32_t baud_rate = 0);

    // cmd thread runs at 100Hz (10ms) by default
    void SetCmdThreadPeriodMs(int32_t period_ms) { cmd_thread_period_ms_ = period_ms; };

    // get robot state
    MmwState GetMmwState();

    // ask background thread to shutdown properly
    void Terminate();

private:
    // hardware communication interface
    std::shared_ptr<ASyncCAN> can_if_;

    // CAN priority higher than serial if both connected
    bool can_connected_ = false;

    // cmd/status update related variables
    std::thread cmd_thread_;
    std::mutex mmw_state_mutex_;

    MmwState mmw_state_;

    int32_t cmd_thread_period_ms_ = 10;
    bool cmd_thread_started_ = false;

    // internal functions
    void ConfigureCANBus(const std::string &can_if_name = "can1");

    void ParseCANFrame(can_frame *rx_frame);

    void NewStatusMsgReceivedCallback(const MmwMessage &msg);

public:
    static void UpdateMmwState(const MmwMessage &status_msg, MmwState &state);
};
} // namespace wescore

#endif /* MMW_BASE_HPP */
