/*
 * MIT License
 *
 * Copyright (c) 2024 Kouhei Ito
 * Copyright (c) 2024 M5Stack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "telemetry.hpp"
#include "rc.hpp"
#include "led.hpp"
#include "sensor.hpp"
#include "flight_control.hpp"
#include <ArduinoJson.h>

uint8_t Telem_mode     = 0;
uint8_t Telem_cnt      = 0;
const uint8_t MAXINDEX = 120;
const uint8_t MININDEX = 30;

void telemetry_sequence(void);
void make_telemetry_header_data(JsonDocument& doc);
void make_telemetry_data(JsonDocument& doc);

void telemetry(void) {
    // StaticJsonDocument<1024> doc;  // サイズはデータ量に応じて調整
    JsonDocument doc;

    if (Telem_mode == 0) {
        // ヘッダデータを構築して送信
        Telem_mode = 1;
        make_telemetry_header_data(doc);
        telemetry_send(doc);
    } else if (Mode > AVERAGE_MODE) {
        const uint8_t N = 10;
        // N回に一度送信
        if (Telem_cnt == 0) telemetry_sequence();
        Telem_cnt++;
        if (Telem_cnt > N - 1) Telem_cnt = 0;
    }
}

void telemetry_sequence(void) {
    // StaticJsonDocument<1024> doc;  // 必要に応じてサイズを調整
    JsonDocument doc;

    switch (Telem_mode) {
        case 1:
            make_telemetry_data(doc);
            // JSON形式でSend
            telemetry_send(doc);

            // Telem_mode = 2;
            break;
    }
}

void make_telemetry_header_data(JsonDocument& doc) {
    doc["header"] = 99;

    doc["Roll_rate_kp"] = Roll_rate_kp;
    doc["Roll_rate_ti"] = Roll_rate_ti;
    doc["Roll_rate_td"] = Roll_rate_td;
    doc["Roll_rate_eta"] = Roll_rate_eta;
    doc["Pitch_rate_kp"] = Pitch_rate_kp;
    doc["Pitch_rate_ti"] = Pitch_rate_ti;
    doc["Pitch_rate_td"] = Pitch_rate_td;
    doc["Pitch_rate_eta"] = Pitch_rate_eta;
    doc["Yaw_rate_kp"] = Yaw_rate_kp;
    doc["Yaw_rate_ti"] = Yaw_rate_ti;
    doc["Yaw_rate_td"] = Yaw_rate_td;
    doc["Yaw_rate_eta"] = Yaw_rate_eta;
    doc["Rall_angle_kp"] = Rall_angle_kp;
    doc["Rall_angle_ti"] = Rall_angle_ti;
    doc["Rall_angle_td"] = Rall_angle_td;
    doc["Rall_angle_eta"] = Rall_angle_eta;
    doc["Pitch_angle_kp"] = Pitch_angle_kp;
    doc["Pitch_angle_ti"] = Pitch_angle_ti;
    doc["Pitch_angle_td"] = Pitch_angle_td;
    doc["Pitch_angle_eta"] = Pitch_angle_eta;
}

void make_telemetry_data(JsonDocument& doc) {
    // テレメトリーデータをJSONオブジェクトに追加
    doc["header"] = 88;

    doc["Elapsed_time"] = Elapsed_time;
    doc["Interval_time"] = Interval_time;
    doc["Roll_angle"] = (Roll_angle - Roll_angle_offset) * 180 / PI;
    doc["Pitch_angle"] = (Pitch_angle - Pitch_angle_offset) * 180 / PI;
    doc["Yaw_angle"] = (Yaw_angle - Yaw_angle_offset) * 180 / PI;
    doc["Roll_rate"] = Roll_rate * 180 / PI;
    doc["Pitch_rate"] = Pitch_rate * 180 / PI;
    doc["Yaw_rate"] = Yaw_rate * 180 / PI;
    doc["Roll_angle_reference"] = Roll_angle_reference * 180 / PI;
    doc["Pitch_angle_reference"] = Pitch_angle_reference * 180 / PI;
    doc["Roll_rate_reference"] = Roll_rate_reference * 180 / PI;
    doc["Pitch_rate_reference"] = Pitch_rate_reference * 180 / PI;
    doc["Yaw_rate_reference"] = Yaw_rate_reference * 180 / PI;
    doc["Thrust_command"] = Thrust_command / BATTERY_VOLTAGE;
    doc["Voltage"] = Voltage;
    doc["Accel_x_raw"] = Accel_x_raw;
    doc["Accel_y_raw"] = Accel_y_raw;
    doc["Accel_z_raw"] = Accel_z_raw;
    doc["Alt_velocity"] = Alt_velocity;
    doc["Z_dot_ref"] = Z_dot_ref;
    doc["FrontLeft_motor_duty"] = FrontLeft_motor_duty;
    doc["RearRight_motor_duty"] = RearRight_motor_duty;
    doc["Alt_ref"] = Alt_ref;
    doc["Altitude2"] = Altitude2;
    doc["Altitude"] = Altitude;
    doc["Az"] = Az;
    doc["Az_bias"] = Az_bias;
    doc["Alt_flag"] = Alt_flag;
    doc["Mode"] = Mode;
    doc["RangeFront"] = RangeFront;
}
