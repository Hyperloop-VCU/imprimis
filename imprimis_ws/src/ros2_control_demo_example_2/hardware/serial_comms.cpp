// serial_comms.cpp
#include "ros2_control_demo_example_2/serial_comms.hpp"

#include <serial/serial.h>
#include <sstream>
#include <iomanip>
#include <iostream> // TODO
#include <cstring>
#include <memory>

namespace 
{
    std::unique_ptr<serial::Serial> g_ser; // single underlying link
}




SerialLink::SerialLink(unsigned int baud, float timeout_s)
    : connected_(false), timeout_s_(timeout_s), baud_(baud) {}

SerialLink::~SerialLink() 
{
    if (g_ser && g_ser->isOpen()) {
        try { g_ser->close(); } catch (...) {}
    }
    connected_ = false;
}




SerialLink::Status SerialLink::initialize_link(const char* port_name) 
{
    // if the pointer is null, or the string is empty, or we're already connected
    if (!port_name || !*port_name || this->is_connected()) return Status::InvalidArg;

    try 
    {
        serial::Timeout timeout = serial::Timeout::simpleTimeout(static_cast<uint32_t>(timeout_s_ * 1000.0f));
        g_ser = std::make_unique<serial::Serial>(port_name, baud_, timeout);
        if (!g_ser->isOpen()) return Status::OpenFailed;
        connected_ = true;
        return Status::Ok;
    } 
    catch (const serial::IOException&) 
    {
        return Status::OpenFailed;
    } 
    catch (...) 
    {
        return Status::ConfigureFailed;
    }
}




SerialLink::Status SerialLink::read_current_angvels(float& leftAngVel, float& rightAngVel) 
{
    if (!is_connected()) return Status::NotConnected;

    try 
    {
        const size_t need = sizeof(float) * 2; // 8 bytes
        std::string bytes = g_ser->read(need); // timeout is already applied
        if (bytes.size() != need) return Status::ReadTimeout;
        
        std::memcpy(&leftAngVel,  bytes.data(), sizeof(float)); // assuming left, right
        std::memcpy(&rightAngVel, bytes.data()+sizeof(float), sizeof(float));
        return Status::Ok;
    } 
    catch (...) 
    {
        return Status::ReadFailed;
    }
}




SerialLink::Status SerialLink::write_angvel_commands(float leftAngvel, float rightAngvel) 
{
    if (!is_connected()) return Status::NotConnected;

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6) << "s " << leftAngvel << ' ' << rightAngvel << '\n';
    const std::string cmd = ss.str();

    try {
        size_t n = g_ser->write(reinterpret_cast<const uint8_t*>(cmd.data()), cmd.size());
        return (n == cmd.size()) ? Status::Ok : Status::WriteFailed;
    } catch (...) {
        return Status::WriteFailed;
    }
}




SerialLink::Status SerialLink::write_reset_encoders() 
{
    if (!is_connected()) return Status::NotConnected;

    const char* cmd = "r\n";
    try {
        size_t n = g_ser->write(reinterpret_cast<const uint8_t*>(cmd), 2);
        return (n == 2) ? Status::Ok : Status::WriteFailed;
    } catch (...) {
        return Status::WriteFailed;
    }
}




SerialLink::Status SerialLink::write_PID(float p, float i, float d, bool right) 
{
    if (!is_connected()) return Status::NotConnected;

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3) << "e " << p << ' ' << i << ' ' << d << ' ';
    if (right) ss << 2 << '\n'; // integer 2 tells the board to update the right controller
    else ss << 1 << '\n';       // integer 1 tells the board to update the left controller
                                // using an enum here was a big hassle, it would have to be consistent across everything

    const std::string cmd = ss.str();

    try {
        size_t n = g_ser->write(reinterpret_cast<const uint8_t*>(cmd.data()), cmd.size());
        return (n == cmd.size()) ? Status::Ok : Status::WriteFailed;
    } catch (...) {
        return Status::WriteFailed;
    }
}




bool SerialLink::is_connected() const 
{
    return connected_ && g_ser && g_ser->isOpen();
}
