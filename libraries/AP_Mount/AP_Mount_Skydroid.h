#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_SKYDROID_ENABLED

#include "AP_Mount_Backend_Serial.h"

class AP_Mount_Skydroid : public AP_Mount_Backend_Serial
{
public:
    AP_Mount_Skydroid(AP_Mount &frontend, AP_Mount_Params &params, uint8_t instance, uint8_t serial_instance) :
        AP_Mount_Backend_Serial(frontend, params, instance, serial_instance) {}

    // init - performs any required initialisation
    void init() override;

    // update - should be called periodically
    void update() override;

    // has_pan_control - returns true if we have yaw control
    bool has_pan_control() const override { return true; }

    // get_attitude_quaternion - not supported, returns false
    bool get_attitude_quaternion(Quaternion& att_quat) override { return false; }

private:
    // send a packet to the gimbal
    // cmd example: "#TPUD2rVER00"  (without checksum)
    void send_packet(const char* cmd);

    // request firmware version — ping команда
    void request_version();

    // read and log any incoming bytes
    void read_incoming();

    uint32_t _last_version_req_ms;
    bool     _got_response;
};

#endif // HAL_MOUNT_SKYDROID_ENABLED