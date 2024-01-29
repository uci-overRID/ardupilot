#include "AP_ODIDScanner.h"
#include "GCS_MAVLink/GCS_MAVLink.h"
#include "GCS_MAVLink/GCS.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Parachute/AP_Parachute.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Common/Location.h>

// TODO: Random default for mav_port needs fix
AP_ODIDScanner::AP_ODIDScanner() : _mav_port(1){

}
void AP_ODIDScanner::init() {
   _chan = mavlink_channel_t(gcs().get_channel_from_port_number(_mav_port));
    _initialised = true;
    _port = AP::serialmanager().get_serial_by_id(_mav_port);
    if (_port != nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Scanner: Found RID Device");
        _port->begin(57600, 512, 512);
    }

    if (in_state.vehicle_list == nullptr) {
        // sanity check param
        // in_state.list_size_param.set(constrain_int16(in_state.list_size_param, 1, INT16_MAX));

        in_state.list_size_param = RID_MAX_INSTANCES;

        in_state.vehicle_list = new rid_vehicle_t[in_state.list_size_param];

        if (in_state.vehicle_list == nullptr) {
            // dynamic RAM allocation of in_state.vehicle_list[] failed
            _init_failed = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RID: Unable to initialize RID vehicle list");
            return;
        }
        in_state.list_size_allocated = in_state.list_size_param;
    }
}

void AP_ODIDScanner::update_recv() {
    mavlink_message_t msg;
    mavlink_status_t status;
    uint32_t now_ms = AP_HAL::millis();

    status.packet_rx_drop_count = 0;
    if(_port == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"_port is null");
        return;
    } 

    const uint16_t nbytes = _port->available();
    if (nbytes > 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"available Bytes: %d", nbytes);
    }
    for (uint16_t i=0; i<nbytes; i++)
    {
        const uint8_t c = (uint8_t)_port->read();
        if (mavlink_frame_char_buffer(channel_buffer(), channel_status(), c, &msg, &status) == MAVLINK_FRAMING_OK) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Scanner: Found message");
            switch(msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: 
                {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Scanner: Recv'd heartbeat");
                    mavlink_heartbeat_t packet;
                    mavlink_msg_heartbeat_decode(&msg, &packet);
                    last_dev_hb_ms = now_ms;
                }
            }
        }
    }
}

void AP_ODIDScanner::handle_msg(mavlink_message_t msg) {
    static int i = 0;
    switch(msg.msgid) {
        case MAVLINK_MSG_ID_UAV_FOUND: 
        {
            // mavlink_msg_uav_found_t uav;
            // mavlink_msg_uav_found_decode(&msg, &uav);
            i+=1;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ODIDScanner: uav's here %d", i);
            break;
        }
    }
}
void AP_ODIDScanner::update() {
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_dev_hb_ms > 5000 && now_ms - last_dev_hb_msg_ms > 5000) {
        last_dev_hb_msg_ms = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Scanner: Device Not Found");
        _port->printf("Scanner: Where is this printing?");
    }
    if (now_ms - last_hb_send_ms > 1000) {
        _port->printf("Scanner: HB Where is this printing?");

        last_hb_send_ms = now_ms;
    mavlink_msg_heartbeat_send(
        _chan,
        gcs().frame_type(),
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        0,
        gcs().custom_mode(),
        0);
        } 
}

void AP_ODIDScannner::update_collide() {
    Loc loc{};
    if (!AP::ahrs().get_location(loc)) {
        loc.zero();
    }

    const AP_GPS &gps = AP::gps();

    loc.fix_type = (AP_GPS_FixType)gps.status();
    loc.epoch_us = gps.time_epoch_usec();
#if AP_RTC_ENABLED
    loc.have_epoch_from_rtc_us = AP::rtc().get_utc_usec(loc.epoch_from_rtc_us);
#endif

    loc.satellites = gps.num_sats();

    loc.horizontal_pos_accuracy_is_valid = gps.horizontal_accuracy(loc.horizontal_pos_accuracy);
    loc.vertical_pos_accuracy_is_valid = gps.vertical_accuracy(loc.vertical_pos_accuracy);
    loc.horizontal_vel_accuracy_is_valid = gps.speed_accuracy(loc.horizontal_vel_accuracy);


    loc.vel_ned = gps.velocity();

    loc.vertRateD_is_valid = AP::ahrs().get_vert_pos_rate_D(loc.vertRateD);

    const auto &baro = AP::baro();
    loc.baro_is_healthy = baro.healthy();

    // Altitude difference between sea level pressure and current
    // pressure (in metres)
    loc.baro_alt_press_diff_sea_level = baro.get_altitude_difference(SSL_AIR_PRESSURE, baro.get_pressure());

    const uint32_t now = AP_HAL::millis();
    uint16_t index = 0;
    while (index < in_state.vehicle_count) {
        // check list and drop stale vehicles. When disabled, the list will get flushed
        if (now - in_state.vehicle_list[index].last_update_ms > VEHICLE_TIMEOUT_MS) {
            // don't increment index, we want to check this same index again because the contents changed
            // also, if we're disabled then clear the list
            delete_vehicle(index);
        } else {
            index++;
        }
    }
}

void AP_ODIDScanner::delete_vehicle(const uint16_t index)
{
    if (index >= in_state.vehicle_count) {
        // index out of range
        return;
    }

    // if the vehicle is the furthest, invalidate it. It has been bumped
    if (index == in_state.furthest_vehicle_index && in_state.furthest_vehicle_distance > 0) {
        in_state.furthest_vehicle_distance = 0;
        in_state.furthest_vehicle_index = 0;
    }
    if (index != (in_state.vehicle_count-1)) {
        in_state.vehicle_list[index] = in_state.vehicle_list[in_state.vehicle_count-1];
    }
    // TODO: is memset needed? When we decrement the index we essentially forget about it
    memset(&in_state.vehicle_list[in_state.vehicle_count-1], 0, sizeof(adsb_vehicle_t));
    in_state.vehicle_count--;
}

Location AP_ODIDScanner::get_location(rid_vehicle_t &vehicle) {
    const Location loc = Location(
        vehicle.info.lat,
        vehicle.info.lon,
        vehicle.info.altitude * 0.1f,
        Location::AltFrame::ABSOLUTE);
    return loc;
}
