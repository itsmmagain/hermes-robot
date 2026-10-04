// Copy this file to secrets.h and fill in your own values.
// secrets.h is listed in .gitignore and must never be committed.
#pragma once

// Telegram bot created with @BotFather
#define BOT_TOKEN "YOUR_TELEGRAM_BOT_TOKEN"

// Telegram chat IDs of the two office users
#define CHAT_ID_HOD  "YOUR_HOD_CHAT_ID"
#define CHAT_ID_EXAM "YOUR_EXAM_OFFICE_CHAT_ID"

// UIDs of the two station RFID tags (uppercase hex, no spaces, as printed on the serial monitor at scan, e.g. "A1B2C3D4")
const String TAG_HOD_OFFICE  = "A1B2C3D4";
const String TAG_EXAM_OFFICE = "E5F60718";

// Wi-Fi network
const char* ssid     = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
