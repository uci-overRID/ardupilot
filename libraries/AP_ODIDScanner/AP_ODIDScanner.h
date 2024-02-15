
#pragma once

#include <cstdint>
#define ODID_SCANNER_ENABLED 1
#ifdef ODID_SCANNER_ENABLED

#include "AP_Common/AP_Common.h"
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Common/Location.h>

bool mac_eq(uint8_t a[6], uint8_t b[6]);

struct Loc : Location {

    uint64_t epoch_us;  // microseconds since 1970-01-01
    uint64_t epoch_from_rtc_us;  // microseconds since 1970-01-01
    bool have_epoch_from_rtc_us;
    uint8_t satellites;

    float horizontal_pos_accuracy;
    bool horizontal_pos_accuracy_is_valid;

    float vertical_pos_accuracy;
    bool vertical_pos_accuracy_is_valid;

    float horizontal_vel_accuracy;
    bool horizontal_vel_accuracy_is_valid;

    Vector3f vel_ned;

    float vertRateD;  // m/s down
    bool vertRateD_is_valid;

    // methods to make us look much like the AP::gps() singleton:
    const Vector3f &velocity() const {
        return vel_ned;
    }
    uint64_t time_epoch_usec() const { return epoch_us; }

    bool speed_accuracy(float &sacc) const;
    bool horizontal_accuracy(float &hacc) const;
    bool vertical_accuracy(float &vacc) const;

    uint8_t num_sats() const { return satellites; }

    // methods to make us look like the AP::ahrs() singleton:
    const Vector2f &groundspeed_vector() const { return vel_ned.xy(); }
    bool get_vert_pos_rate_D(float &velocity) const {
        velocity = vertRateD;
        return vertRateD_is_valid;
    }

    // data from a pressure sensor:
    bool baro_is_healthy;
    float baro_alt_press_diff_sea_level;
};

struct rid_vehicle_t {
    mavlink_open_drone_id_location_t loc;
    mavlink_open_drone_id_basic_id_t info;
    uint32_t last_update_ms;
};
#define DRONE_TRACK_MAX 10

class AP_ODIDScanner
{
public:
    struct {
        // list management
        AP_Int16    list_size_param;
        uint16_t    list_size_allocated;
        rid_vehicle_t *vehicle_list;
        uint16_t    vehicle_count;
        AP_Int32    list_radius;
        AP_Int16    list_altitude;

        // index of and distance to furthest vehicle in list
        uint16_t    furthest_vehicle_index;
        float       furthest_vehicle_distance;

        // streamrate stuff
        uint32_t    send_start_ms[MAVLINK_COMM_NUM_BUFFERS];
        uint16_t    send_index[MAVLINK_COMM_NUM_BUFFERS];
    } in_state;

    rid_vehicle_t vehicles[DRONE_TRACK_MAX];
    int count = 0;
    int get_count();
    bool update_vehicle(mavlink_open_drone_id_location_t &);
    bool update_vehicle(mavlink_open_drone_id_basic_id_t &);
    bool add_vehicle(mavlink_open_drone_id_location_t &);
    bool add_vehicle(mavlink_open_drone_id_basic_id_t &);
    rid_vehicle_t&  get_vehicle(int i);
    Location get_vehicle_location(int i);


    AP_ODIDScanner();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_ODIDScanner);
    void init();
    void update();
    void update_recv();
    void update_collide();
    Location get_location(const rid_vehicle_t &vehicle);
    void delete_vehicle(const uint16_t index);
    void handle_msg(mavlink_message_t);
    Location get_location(rid_vehicle_t &);
    bool enabled();
    // mavlink_channel_t _chan; // mavlink channel that communicates with the remote id transceiver
    uint8_t _mav_port;
    bool message_from_rx(mavlink_channel_t& chan);

    mavlink_uav_found_t found_msg;
    mavlink_channel_t _chan;
    AP_HAL::UARTDriver* _port;
    bool _initialised;
    uint32_t last_send_ms;
    uint32_t last_dev_hb_ms;
    uint32_t last_dev_hb_msg_ms;
    uint32_t last_hb_send_ms;

    mavlink_message_t _channel_buffer;
    mavlink_status_t _channel_status;

    mavlink_message_t *channel_buffer() { return &_channel_buffer; }
    mavlink_status_t *channel_status() { return &_channel_status; }
};


#endif
