#include "AP_Mount_Skydroid.h"

#if HAL_MOUNT_SKYDROID_ENABLED

#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

void AP_Mount_Skydroid::init()
{
    AP_Mount_Backend_Serial::init();
    _last_version_req_ms = 0;
    _got_response = false;
}

void AP_Mount_Skydroid::update()
{
    if (_uart == nullptr) {
        return;
    }

    // читаємо відповідь якщо є
    read_incoming();

    // раз на 2 секунди шлемо ping (запит версії)
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_version_req_ms >= 2000) {
        _last_version_req_ms = now_ms;
        request_version();
    }
}

void AP_Mount_Skydroid::request_version()
{
    send_packet("#TPUD2rVER00");
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Skydroid: sending VER request");
}

void AP_Mount_Skydroid::send_packet(const char* cmd)
{
    if (_uart == nullptr) {
        return;
    }

    // рахуємо довжину і checksum одночасно
    uint8_t crc = 0;
    uint8_t len = 0;
    while (cmd[len] != '\0') {
        crc += (uint8_t)cmd[len];
        len++;
    }

    // конвертуємо crc в два hex символи вручну (як SIYI)
    const char hex[] = "0123456789ABCDEF";
    char buf[32];
    memcpy(buf, cmd, len);
    buf[len]   = hex[(crc >> 4) & 0x0F];
    buf[len+1] = hex[crc & 0x0F];
    buf[len+2] = '\0';

    _uart->write((const uint8_t*)buf, len + 2);
}

void AP_Mount_Skydroid::read_incoming()
{
    uint32_t nbytes = MIN(_uart->available(), 32U);
    if (nbytes == 0) {
        return;
    }

    uint8_t buf[33] {};
    for (uint32_t i = 0; i < nbytes; i++) {
        uint8_t b;
        if (!_uart->read(b)) {
            break;
        }
        buf[i] = b;
    }

    if (!_got_response) {
        _got_response = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Skydroid: got response: %.20s", (const char*)buf);
    }
}

#endif // HAL_MOUNT_SKYDROID_ENABLED