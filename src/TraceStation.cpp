/*
 * SoftRF(.ino) firmware
 * Copyright (C) 2016-2019 Linar Yusupov
 *
 * Author: Linar Yusupov, linar.r.yusupov@gmail.com
 *
 * Web: http://github.com/lyusupov/SoftRF
 *
 * Credits:
 *   Arduino core for ESP8266 is developed/supported by ESP8266 Community (support-esp8266@esp8266.com)
 *   AVR/Arduino nRF905 Library/Driver is developed by Zak Kemble, contact@zakkemble.co.uk
 *   flarm_decode is developed by Stanislaw Pusep, http://github.com/creaktive
 *   Arduino Time Library is developed by Paul Stoffregen, http://github.com/PaulStoffregen
 *   "Aircraft" and MAVLink Libraries are developed by Andy Little
 *   TinyGPS++ and PString Libraries are developed by Mikal Hart
 *   Adafruit NeoPixel Library is developed by Phil Burgess, Michael Miller and others
 *   TrueRandom Library is developed by Peter Knight
 *   IBM LMIC framework is maintained by Matthijs Kooijman
 *   ESP8266FtpServer is developed by David Paiva
 *   Lib_crc is developed by Lammert Bies
 *   OGN library is developed by Pawel Jalocha
 *   NMEA library is developed by Timur Sinitsyn, Tobias Simon, Ferry Huberts
 *   ADS-B encoder C++ library is developed by yangbinbin (yangbinbin_ytu@163.com)
 *   Adafruit BMP085 library is developed by Limor Fried and Ladyada
 *   Adafruit BMP280 library is developed by Kevin Townsend
 *   Adafruit MPL3115A2 library is developed by Limor Fried and Kevin Townsend
 *   U8g2 monochrome LCD, OLED and eInk library is developed by Oliver Kraus
 *   NeoPixelBus library is developed by Michael Miller
 *   jQuery library is developed by JS Foundation
 *   EGM96 data is developed by XCSoar team
 *   BCM2835 C library is developed by Mike McCauley
 *   SimpleNetwork library is developed by Dario Longobardi
 *   ArduinoJson library is developed by Benoit Blanchon
 *   Flashrom library is part of the flashrom.org project
 *   EasyLink library is developed by Robert Wessels and Tony Cave
 *   Dump978 library is developed by Oliver Jowett
 *   FEC library is developed by Phil Karn
 *   AXP202X library is developed by Lewis He
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <Arduino.h>
#include "OTAHelper.h"
#include "TimeHelper.h"
#include "LEDHelper.h"
#include "GNSSHelper.h"
#include "EEPROMHelper.h"
#include "BatteryHelper.h"
#include "NMEAHelper.h"
#include "SoCHelper.h"
#include "WiFiHelper.h"
#include "WebHelper.h"


#include "TraceStation.h"


#define DEBUG 0
#define DEBUG_TIMING 0

//#define isTimeToDisplay() (millis() - LEDTimeMarker > 1000)
//#define isTimeToExport() (millis() - ExportTimeMarker > 1000)

ufo_t ThisStation;
hardware_info_t hw_info = {
  .model    = SOFTRF_MODEL_PRIME_MK2,
  .revision = 2,
  .soc      = SOC_NONE,
  .rf       = 0,
  .gnss     = GNSS_MODULE_NONE,
  .baro     = 0,
  .display  = DISPLAY_NONE
};

//unsigned long LEDTimeMarker = 0;
//unsigned long ExportTimeMarker = 0;

void normal_loop();
//void uav_loop();
//void bridge_loop();
//void watchout_loop();
//void txrx_test_loop();

void setup()
{
  rst_info *resetInfo;

  hw_info.soc = SoC_setup(); // Has to be very first procedure in the execution order

  resetInfo = (rst_info *) SoC->getResetInfoPtr();

  Serial.begin(SERIAL_OUT_BR);
  delay(900);
  Serial.println();

  Serial.print(F(TRACESTATION_IDENT));
  Serial.print(String((SoC->getChipId() & 0xFFFFFF), HEX));
  Serial.print(F(" FW.REV: " TRACESTATION_FIRMWARE_VERSION));
  
  Serial.flush();

  Serial.println(""); Serial.print(F("Reset reason: ")); Serial.println(resetInfo->reason);
  Serial.println(SoC->getResetReason());
  Serial.print(F("Free heap size: ")); Serial.println(ESP.getFreeHeap());
#if defined(ESP32_DEVEL_CORE)
  Serial.print(F("PSRAM: ")); Serial.println(psramFound() ? F("found") : F("not found"));
#endif
  Serial.println(SoC->getResetInfo()); Serial.println("");

  EEPROM_setup();

  ThisStation.addr = SoC->getChipId() & 0x00FFFFFF;

  //hw_info.rf = RF_setup();
#if 0
  if (hw_info.model    == SOFTRF_MODEL_PRIME_MK2 &&
      hw_info.revision == 2                      &&
      RF_SX1276_RST_is_connected)
  {
    hw_info.revision = 5;
  }
  #endif

  delay(100);

  //hw_info.baro = Baro_setup();
  hw_info.baro = 0;

  hw_info.display = SoC->Display_setup();
  hw_info.display = DISPLAY_NONE;

#if 0
  if (settings->mode == SOFTRF_MODE_UAV) {
    Serial.begin(57600);
    MAVLink_setup();
    ThisStation.aircraft_type = AIRCRAFT_TYPE_UAV;  
  }  else {
    hw_info.gnss = GNSS_setup();
    ThisStation.aircraft_type = settings->aircraft_type;
  }
  #endif
  ThisStation.protocol = settings->rf_protocol;
  ThisStation.stealth  = settings->stealth;
  ThisStation.no_track = settings->no_track;

  hw_info.gnss = GNSS_setup();
  ThisStation.aircraft_type = settings->aircraft_type;
  
  Battery_setup();
  //Traffic_setup();

  SoC->swSer_enableRx(false);

  //LED_setup();

  WiFi_setup();

  if (SoC->Bluetooth) {
     SoC->Bluetooth->setup();
  }

  OTA_setup();
  Web_setup();
  NMEA_setup();

#if defined(ENABLE_TTN)
  TTN_setup();
#endif

  delay(1000);

  /* expedite restart on WDT reset */
  if (resetInfo->reason != REASON_WDT_RST) {
    //LED_test();
  }

  //SoC->Sound_test(resetInfo->reason);

  switch (settings->mode)
  {
  case SOFTRF_MODE_TXRX_TEST:
  case SOFTRF_MODE_WATCHOUT:
    Time_setup();
    break;
  case SOFTRF_MODE_BRIDGE:
    break;
  case SOFTRF_MODE_NORMAL:
  case SOFTRF_MODE_UAV:
  default:
    SoC->swSer_enableRx(true);
    break;
  }

  SoC->WDT_setup();
}

void loop()
{
  // Do common RF stuff first
  //RF_loop();

  switch (settings->mode)
  {
  case SOFTRF_MODE_TXRX_TEST:
    //txrx_test_loop();
    break;
  case SOFTRF_MODE_UAV:
    //uav_loop();
    break;
  case SOFTRF_MODE_BRIDGE:
    //bridge_loop();
    break;
  case SOFTRF_MODE_WATCHOUT:
   // watchout_loop();
    break;
  case SOFTRF_MODE_NORMAL:
  default:
    normal_loop();
    break;
  }

  // Show status info on tiny OLED display
 // SoC->Display_loop();

  // battery status LED
  LED_loop();

  // Handle DNS
  WiFi_loop();

  // Handle Web
  Web_loop();

  // Handle OTA update.
  OTA_loop();

#if LOGGER_IS_ENABLED
  Logger_loop();
#endif /* LOGGER_IS_ENABLED */

  SoC->loop();

  Battery_loop();

  yield();
}

void shutdown(const char *msg)
{
  SoC->WDT_fini();

  SoC->swSer_enableRx(false);

  NMEA_fini();

  Web_fini();

  WiFi_fini();

  if (settings->mode != SOFTRF_MODE_UAV) {
    GNSS_fini();
  }

  //SoC->Display_fini(msg);

  //RF_Shutdown();

  SoC_fini();
}

void normal_loop()
{
  //bool success;

  //Baro_loop();

  PickGNSSFix();

  GNSSTimeSync();

  ThisStation.timestamp = now();
  if (isValidFix()) {
    ThisStation.latitude = gnss.location.lat();
    ThisStation.longitude = gnss.location.lng();
    ThisStation.altitude = gnss.altitude.meters();
    ThisStation.course = gnss.course.deg();
    ThisStation.speed = gnss.speed.knots();
    ThisStation.hdop = (uint16_t) gnss.hdop.value();
    ThisStation.geoid_separation = gnss.separation.meters();

    /*
     * When geoidal separation is zero or not available - use approx. EGM96 value
     */
    if (ThisStation.geoid_separation == 0.0) {
      ThisStation.geoid_separation = (float) LookupSeparation(
                                                ThisStation.latitude,
                                                ThisStation.longitude
                                              );
      /* we can assume the GPS unit is giving ellipsoid height */
      ThisStation.altitude -= ThisStation.geoid_separation;
    }

    //RF_Transmit(RF_Encode(&ThisStation), true);
  }

  //success = RF_Receive();

  #if DEBUG
    success = true;
  #endif

  //if (success && isValidFix()) ParseData();
// Handle Air Connect
  NMEA_loop();

  //ClearExpired();

}

#if 0
void uav_loop()
{
  bool success = false;

  PickMAVLinkFix();

  MAVLinkTimeSync();
  MAVLinkSetWiFiPower();

  ThisStation.timestamp = now();

  if (isValidMAVFix()) {
    ThisStation.latitude = the_aircraft.location.gps_lat / 1e7;
    ThisStation.longitude = the_aircraft.location.gps_lon / 1e7;
    ThisStation.altitude = the_aircraft.location.gps_alt / 1000.0;
    ThisStation.course = the_aircraft.location.gps_cog;
    ThisStation.speed = (the_aircraft.location.gps_vog / 100.0) / _GPS_MPS_PER_KNOT;
    ThisStation.pressure_altitude = the_aircraft.location.baro_alt;
    ThisStation.hdop = the_aircraft.location.gps_hdop;

    RF_Transmit(RF_Encode(&ThisStation), true);
  }

  success = RF_Receive();

  if (success && isValidMAVFix()) ParseData();

  if (isTimeToExport() && isValidMAVFix()) {
    MAVLinkShareTraffic();
    ExportTimeMarker = millis();
  }

  ClearExpired();
}

void bridge_loop()
{
  bool success;

  size_t tx_size = Raw_Receive_UDP(&TxBuffer[0]);

  if (tx_size > 0) {
    RF_Transmit(tx_size, true);
  }

  success = RF_Receive();

  if(success)
  {
    size_t rx_size = RF_Payload_Size(settings->rf_protocol);
    rx_size = rx_size > sizeof(fo.raw) ? sizeof(fo.raw) : rx_size;

    memset(fo.raw, 0, sizeof(fo.raw));
    memcpy(fo.raw, RxBuffer, rx_size);

    if (settings->nmea_p) {
      StdOut.print(F("$PSRFI,"));
      StdOut.print((unsigned long) now());    StdOut.print(F(","));
      StdOut.print(Bin2Hex(fo.raw, rx_size)); StdOut.print(F(","));
      StdOut.println(RF_last_rssi);
    }

    Raw_Transmit_UDP();
  }

  if (isTimeToDisplay()) {
    LED_Clear();
    LEDTimeMarker = millis();
  }
}

void watchout_loop()
{
  bool success;

  success = RF_Receive();

  if (success) {
    size_t rx_size = RF_Payload_Size(settings->rf_protocol);
    rx_size = rx_size > sizeof(fo.raw) ? sizeof(fo.raw) : rx_size;

    memset(fo.raw, 0, sizeof(fo.raw));
    memcpy(fo.raw, RxBuffer, rx_size);

    if (settings->nmea_p) {
      StdOut.print(F("$PSRFI,"));
      StdOut.print((unsigned long) now());    StdOut.print(F(","));
      StdOut.print(Bin2Hex(fo.raw, rx_size)); StdOut.print(F(","));
      StdOut.println(RF_last_rssi);
    }
  }

  if (isTimeToDisplay()) {
    LED_Clear();
    LEDTimeMarker = millis();
  }
}

unsigned int pos_ndx = 0;
unsigned long TxPosUpdMarker = 0;

void txrx_test_loop()
{
  bool success = false;
  #if DEBUG_TIMING
    unsigned long baro_start_ms, baro_end_ms;
    unsigned long tx_start_ms, tx_end_ms, rx_start_ms, rx_end_ms;
    unsigned long parse_start_ms, parse_end_ms, led_start_ms, led_end_ms;
    unsigned long export_start_ms, export_end_ms;
    unsigned long oled_start_ms, oled_end_ms;
  #endif
  ThisStation.timestamp = now();

  if (TxPosUpdMarker == 0 || (millis() - TxPosUpdMarker) > 4000 ) {
    ThisStation.latitude =  pgm_read_float( &txrx_test_positions[pos_ndx][0]);
    ThisStation.longitude =  pgm_read_float( &txrx_test_positions[pos_ndx][1]);
    pos_ndx = (pos_ndx + 1) % TXRX_TEST_NUM_POSITIONS;
    TxPosUpdMarker = millis();
  }
  ThisStation.altitude = TXRX_TEST_ALTITUDE;
  ThisStation.course = TXRX_TEST_COURSE;
  ThisStation.speed = TXRX_TEST_SPEED;
  ThisStation.vs = TXRX_TEST_VS;

  #if DEBUG_TIMING
    baro_start_ms = millis();
  #endif
    Baro_loop();
  #if DEBUG_TIMING
    baro_end_ms = millis();
  #endif

  #if DEBUG_TIMING
    tx_start_ms = millis();
  #endif
    RF_Transmit(RF_Encode(&ThisStation), true);
  #if DEBUG_TIMING
    tx_end_ms = millis();
    rx_start_ms = millis();
  #endif
    success = RF_Receive();
  #if DEBUG_TIMING
    rx_end_ms = millis();
  #endif

  #if DEBUG_TIMING
    parse_start_ms = millis();
  #endif
    if (success) ParseData();
  #if DEBUG_TIMING
    parse_end_ms = millis();
  #endif

  #if defined(ENABLE_TTN)
    TTN_loop();
  #endif

    Traffic_loop();

  #if DEBUG_TIMING
    led_start_ms = millis();
  #endif
    if (isTimeToDisplay()) {
      LED_DisplayTraffic();
      LEDTimeMarker = millis();
    }
  #if DEBUG_TIMING
    led_end_ms = millis();
  #endif

  #if DEBUG_TIMING
    export_start_ms = millis();
  #endif
    if (isTimeToExport()) {
      NMEA_Position();
      NMEA_Export();
      GDL90_Export();
      D1090_Export();
      ExportTimeMarker = millis();
    }
  #if DEBUG_TIMING
    export_end_ms = millis();
  #endif

  #if DEBUG_TIMING
    oled_start_ms = millis();
  #endif
    SoC->Display_loop();
  #if DEBUG_TIMING
    oled_end_ms = millis();
  #endif

  #if DEBUG_TIMING
    if (baro_start_ms - baro_end_ms) {
      Serial.print(F("Baro start: "));
      Serial.print(baro_start_ms);
      Serial.print(F(" Baro stop: "));
      Serial.println(baro_end_ms);
    }
    if (tx_end_ms - tx_start_ms) {
      Serial.print(F("TX start: "));
      Serial.print(tx_start_ms);
      Serial.print(F(" TX stop: "));
      Serial.println(tx_end_ms);
    }
    if (rx_end_ms - rx_start_ms) {
      Serial.print(F("RX start: "));
      Serial.print(rx_start_ms);
      Serial.print(F(" RX stop: "));
      Serial.println(rx_end_ms);
    }
    if (parse_end_ms - parse_start_ms) {
      Serial.print(F("Parse start: "));
      Serial.print(parse_start_ms);
      Serial.print(F(" Parse stop: "));
      Serial.println(parse_end_ms);
    }
    if (led_end_ms - led_start_ms) {
      Serial.print(F("LED start: "));
      Serial.print(led_start_ms);
      Serial.print(F(" LED stop: "));
      Serial.println(led_end_ms);
    }
    if (export_end_ms - export_start_ms) {
      Serial.print(F("Export start: "));
      Serial.print(export_start_ms);
      Serial.print(F(" Export stop: "));
      Serial.println(export_end_ms);
    }
    if (oled_end_ms - oled_start_ms) {
      Serial.print(F("OLED start: "));
      Serial.print(oled_start_ms);
      Serial.print(F(" OLED stop: "));
      Serial.println(oled_end_ms);
    }
  #endif

  // Handle Air Connect
  NMEA_loop();

  ClearExpired();
}
#endif