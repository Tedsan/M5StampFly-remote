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

#include "rc.hpp"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "flight_control.hpp"
#include <zenoh-pico.h>
#include <ArduinoJson.h>

// WiFi-specific parameters
#define SSID "Buffalo-2G-EF00"
#define PASS "brnk34t5u75aa"

#define CLIENT_OR_PEER 1  // 0: Client mode; 1: Peer mode
#if CLIENT_OR_PEER == 0
#define MODE "client"
#define CONNECT ""  // If empty, it will scout
#elif CLIENT_OR_PEER == 1
#define MODE "peer"
#define CONNECT "udp/224.0.0.225:7447#iface=en0"
#else
#error "Unknown Zenoh operation mode. Check CLIENT_OR_PEER value."
#endif

#define SUB_PREFIX "control"
#define PUB_PREFIX "drone"

z_owned_session_t zenoh_session;
z_owned_publisher_t zenoh_pub;

// esp_now_peer_info_t slave;

volatile uint16_t Connect_flag = 0;

// RC
volatile float Stick[16];
volatile uint8_t Recv_MAC[3];

volatile uint8_t Rc_err_flag = 0;

// 受信コールバック
void zenoh_data_handler(const z_sample_t *sample, void *arg) {
    Connect_flag = 0;

    // 受信したペイロードを文字列として取得
    std::string payload((const char *)sample->payload.start, sample->payload.len);
    //USBSerial.println(payload.c_str());

    // JSONオブジェクトとしてパース
    JsonDocument doc;  // 256バイトのバッファを使用
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
        USBSerial.print("Failed to parse JSON: ");
        USBSerial.println(error.c_str());
        return;
    }

    // JSONオブジェクトから各値を取得
    if (doc.containsKey("rudder")) {
        Stick[RUDDER] = doc["rudder"].as<float>();
    }
    if (doc.containsKey("throttle")) {
        Stick[THROTTLE] = doc["throttle"].as<float>();
    }
    if (doc.containsKey("aileron")) {
        Stick[AILERON] = doc["aileron"].as<float>();
    }
    if (doc.containsKey("elevator")) {
        Stick[ELEVATOR] = doc["elevator"].as<float>();
    }
    if (doc.containsKey("button_arm")) {
        Stick[BUTTON_ARM] = doc["button_arm"].as<float>();
    }
    if (doc.containsKey("button_flip")) {
        Stick[BUTTON_FLIP] = doc["button_flip"].as<float>();
    }
    if (doc.containsKey("controlmode")) {
        Stick[CONTROLMODE] = doc["controlmode"].as<float>();
    }
    if (doc.containsKey("altcontrolmode")) {
        Stick[ALTCONTROLMODE] = doc["altcontrolmode"].as<float>();
    }

    Stick[LOG] = 0.0;

#if 0
    // デバッグ用に受信したJSONデータを出力
    USBSerial.println("Received JSON Data:");
    serializeJsonPretty(doc, USBSerial);
#endif
}

void rc_init(void) {
    // Initialize Stick list
    for (uint8_t i = 0; i < 16; i++) Stick[i] = 0.0;

    // WiFiの設定
    // wifi接続カウント
    uint8_t Cnt = 0;
    USBSerial.print("Connecting to WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.begin(SSID, PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        // 一定時間経過後に再起動
        if (Cnt++ > 10) {
            USBSerial.println("Unable to connect to WiFi!");
            ESP.restart();
        }
    }
    USBSerial.println("OK");

    // Zenohセッションの初期化
    z_owned_config_t config = z_config_default();
    zp_config_insert(z_config_loan(&config), Z_CONFIG_MODE_KEY, z_string_make(MODE));
    if (strcmp(CONNECT, "") != 0) {
        zp_config_insert(z_config_loan(&config), Z_CONFIG_CONNECT_KEY, z_string_make(CONNECT));
    }

    // Zenohセッションのオープン
    Serial.print("Opening Zenoh Session...");
    zenoh_session = z_open(z_config_move(&config));
    if (!z_session_check(&zenoh_session)) {
        USBSerial.println("Unable to open Zenoh session!");
        ESP.restart();
    }
    USBSerial.println("OK");

    // // パブリッシャーの宣言
    // zenoh_pub = z_declare_publisher(z_session_loan(&zenoh_session), z_keyexpr(PUB_PREFIX), NULL);
    // if (!z_publisher_check(&zenoh_pub)) {
    //     USBSerial.println("Unable to declare Zenoh publisher!");
    //     ESP.restart();
    // }

    // Start the receive and the session lease loop for zenoh-pico
    zp_start_read_task(z_session_loan(&zenoh_session), NULL);
    zp_start_lease_task(z_session_loan(&zenoh_session), NULL);

    // サブスクライバーの宣言
    USBSerial.print("Declaring Subscriber on ");
    USBSerial.print(SUB_PREFIX);
    USBSerial.println(" ...");
    z_owned_closure_sample_t callback = z_closure_sample(zenoh_data_handler, NULL, NULL);
    z_owned_subscriber_t sub = z_declare_subscriber(z_session_loan(&zenoh_session), z_keyexpr(SUB_PREFIX), z_closure_sample_move(&callback), NULL);
    if (!z_subscriber_check(&sub)) {
        USBSerial.println("Unable to declare Zenoh subscriber.");
        ESP.restart();
    }
    USBSerial.println("OK");
    USBSerial.println("Zenoh-Pico Ready.");

    delay(300);
}

void telemetry_send(JsonDocument& doc) {
    // JSONオブジェクトをシリアル化して文字列に変換
    char payload[1024];
    size_t len = serializeJson(doc, payload, sizeof(payload));

#if 1
    // デバッグ用にシリアル出力
    Serial.println("Sending JSON Data:");
    Serial.println(payload);
#endif

    // Zenohにデータを送信
    if (z_publisher_put(z_publisher_loan(&zenoh_pub), (const uint8_t *)payload, len, NULL) < 0) {
        Serial.println("Error while publishing JSON data");
    } else {
        Serial.println("Successfully published JSON data");
    }
}

void rc_end(void) {
    // Zenohセッションのクローズ処理などを行います
    z_close(z_session_move(&zenoh_session));
}

uint8_t rc_isconnected(void) {
    Connect_flag++;
    return (Connect_flag < 40) ? 1 : 0;
}

void rc_demo() {
    // デモ用の処理を追加できます
}