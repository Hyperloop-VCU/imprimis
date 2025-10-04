// This SerialLink class handles all the serial communication with the ESP32.

#ifndef SERIAL_COMMS_HPP
#define SERIAL_COMMS_HPP

#include <string>
#include <cstdint>

class SerialLink
{
public:

    enum class Status : int 
    {
        Ok = 0,
        NotInitialized,
        OpenFailed,
        ConfigureFailed,
        NotConnected,
        InvalidArg,
        WriteFailed,
        ReadFailed,
        ReadTimeout
    };

    // The constructor shouldn't try to initialize the link. initialize connected_ to false
    SerialLink(unsigned int baud, float timeout_s);

    // The destructor should disconnect the link.
    ~SerialLink();

    // initializes the link with the given port name.
    Status initialize_link(const char* port_name);

    // read two floats (left, right) written by the ESP32 as raw bytes
    // The esp32 writes like this: 
    // Serial.write((byte*)(&data.currLeftAngvel), sizeof(float));
    // Serial.write((byte*)(&data.currRightAngvel), sizeof(float));
    Status read_current_angvels(float& leftAngVel, float& rightAngVel);

    // send: s [leftAngvel] [rightAngvel]\n
    // example: "s 0.31 0.52\n"
    Status write_angvel_commands(float leftAngvel, float rightAngvel);

    // sends the character 'r' + newline
    Status write_reset_encoders();


    // if right is true, change the PID gains of the right controller
    // if right is false, change the PID gains of the left controller
    // send: e [p] [i] [d] [gainChange: 1 or 2]\n
    // example: "e 2.3 0.1 0.3 1\n"
    Status write_PID(float p, float i, float d, bool right);

    // returns true if the serial connection is up, false if it isn't
    bool is_connected() const;

private:
    bool connected_;
    float timeout_s_;
    unsigned int baud_;
};

#endif
