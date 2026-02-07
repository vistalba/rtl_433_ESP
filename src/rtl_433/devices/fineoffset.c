/** @file
    Fine Offset Electronics sensor protocol.

    Copyright (C) 2017 Tommy Vestermark
    Enhanced (C) 2019 Christian W. Zuckschwerdt <zany@triq.net>
    Added WH51 Soil Moisture Sensor (C) 2019 Marco Di Leo

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#include "decoder.h"

static int const wind_dir_degr[]= {0, 23, 45, 68, 90, 113, 135, 158, 180, 203, 225, 248, 270, 293, 315, 338};

// Forward declaration
static int fineoffset_WH2_callback(r_device *decoder, bitbuffer_t *bitbuffer);

/**
Fine Offset Electronics WH2 Temperature/Humidity sensor protocol,
also Agimex Rosenborg 66796 (sold in Denmark), collides with WH5,
also ClimeMET CM9088 (Sold in UK),
also TFA Dostmann/Wertheim 30.3157 (Temperature only!) (sold in Germany).

The sensor sends two identical packages of 48 bits each ~48s. The bits are PWM modulated with On Off Keying.

The data is grouped in 6 bytes / 12 nibbles.

    [pre] [pre] [type] [id] [id] [temp] [temp] [temp] [humi] [humi] [crc] [crc]

There is an extra, unidentified 7th byte in WH2A packages.

- pre is always 0xFF
- type is always 0x4 (may be different for different sensor type?)
- id is a random id that is generated when the sensor starts
- temp is 12 bit signed magnitude scaled by 10 celsius
- humi is 8 bit relative humidity percentage

Based on reverse engineering with gnu-radio and the nice article here:
http://lucsmall.com/2012/04/29/weather-station-hacking-part-2/
*/
#define MODEL_WH2 2
#define MODEL_WH2A 3
#define MODEL_WH5 5
#define MODEL_RB 6
#define MODEL_TP 7

static int fineoffset_WH2_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    bitrow_t *bb = bitbuffer->bb;
    uint8_t b[6] = {0};
    data_t *data;

    int model_num;
    int type;
    uint8_t id;
    int16_t temp;
    float temperature;
    uint8_t humidity;
    int user_data = 0; // Default: process normally

    if (bitbuffer->bits_per_row[0] == 48 &&
            bb[0][0] == 0xFF) { // WH2
        bitbuffer_extract_bytes(bitbuffer, 0, 8, b, 40);
        model_num = MODEL_WH2;

    } else if (bitbuffer->bits_per_row[0] == 55 &&
            bb[0][0] == 0xFE) { // WH2A
        bitbuffer_extract_bytes(bitbuffer, 0, 7, b, 48);
        model_num = MODEL_WH2A;

    } else if (bitbuffer->bits_per_row[0] == 47 &&
            bb[0][0] == 0xFE) { // WH5
        bitbuffer_extract_bytes(bitbuffer, 0, 7, b, 40);
        model_num = MODEL_WH5;
        user_data = 0;

    } else if (bitbuffer->bits_per_row[0] == 49 &&
            bb[0][0] == 0xFF && (bb[0][1]&0x80) == 0x80) { // Telldus
        bitbuffer_extract_bytes(bitbuffer, 0, 9, b, 40);
        model_num = MODEL_TP;

    } else
        return DECODE_ABORT_LENGTH;

    // Validate package
    if (b[4] != crc8(&b[0], 4, 0x31, 0))
        return DECODE_FAIL_MIC;

    // Nibble 2 contains type, must be 0x04
    type = b[0] >> 4;
    if (type != 4) {
        decoder_logf(decoder, 1, __func__, "Unknown type: (%d) %d", model_num, type);
        return DECODE_FAIL_SANITY;
    }

    // Nibble 3,4 contains id
    id = ((b[0]&0x0F) << 4) | ((b[1]&0xF0) >> 4);

    // Nibble 5,6,7 contains 12 bits of temperature
    temp = ((b[1] & 0x0F) << 8) | b[2];
    if (bitbuffer->bits_per_row[0] != 47 || user_data) { // WH2, Telldus, WH2A
        // The temperature is signed magnitude and scaled by 10
        if (temp & 0x800) {
            temp &= 0x7FF;
            temp = -temp;
        }
    } else { // WH5
        // The temperature is unsigned offset by 40 C and scaled by 10
        temp -= 400;
    }
    temperature = temp * 0.1f;

    // Nibble 8,9 contains humidity
    humidity = b[3];

    /* clang-format off */
    data = data_make(
            "model",            "",             DATA_COND, model_num == MODEL_WH2,  DATA_STRING, "Fineoffset-WH2",
            "model",            "",             DATA_COND, model_num == MODEL_WH2A, DATA_STRING, "Fineoffset-WH2A",
            "model",            "",             DATA_COND, model_num == MODEL_WH5,  DATA_STRING, "Fineoffset-WH5",
            "model",            "",             DATA_COND, model_num == MODEL_RB,   DATA_STRING, "Rosenborg-66796",
            "model",            "",             DATA_COND, model_num == MODEL_TP,   DATA_STRING, "Fineoffset-TelldusProove",
            "id",               "ID",           DATA_INT, id,
            "temperature_C",    "Temperature",  DATA_FORMAT, "%.1f C", DATA_DOUBLE, temperature,
            "humidity",         "Humidity",     DATA_COND, humidity != 0xff, DATA_FORMAT, "%u %%", DATA_INT, humidity,
            "mic",              "Integrity",    DATA_STRING, "CRC",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

#define MODEL_WH24 24
#define MODEL_WH65B 65

static int fineoffset_WH24_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    data_t *data;
    uint8_t const preamble[] = {0xAA, 0x2D, 0xD4};
    uint8_t b[17];
    unsigned bit_offset;
    int type;

    // Validate package
    if (bitbuffer->bits_per_row[0] < 190 || bitbuffer->bits_per_row[0] > 215) {
        return DECODE_ABORT_LENGTH;
    }

    // Find a data package and extract data buffer
    bit_offset = bitbuffer_search(bitbuffer, 0, 0, preamble, sizeof(preamble) * 8) + sizeof(preamble) * 8;
    if (bit_offset + sizeof(b) * 8 > bitbuffer->bits_per_row[0]) {
        decoder_logf(decoder, 1, __func__, "Fineoffset_WH24: short package. Header index: %u", bit_offset);
        return DECODE_ABORT_LENGTH;
    }

    // Classification heuristics
    if (bitbuffer->bits_per_row[0] - bit_offset - sizeof(b) * 8 < 8)
        if (bit_offset < 61)
            type = MODEL_WH24;
        else
            type = MODEL_WH65B;
    else
        type = MODEL_WH65B;

    bitbuffer_extract_bytes(bitbuffer, 0, bit_offset, b, sizeof(b) * 8);
    decoder_log_bitrow(decoder, 1, __func__, b, sizeof(b) * 8, "Raw @ bit_offset [%u]", bit_offset);

    if (b[0] != 0x24)
        return DECODE_FAIL_SANITY;

    // Verify checksum
    uint8_t crc = crc8(b, 15, 0x31, 0x00);
    uint8_t checksum = 0;
    for (unsigned n = 0; n < 16; ++n) {
        checksum += b[n];
    }
    if (crc != b[15] || checksum != b[16]) {
        decoder_logf(decoder, 1, __func__, "Fineoffset_WH24: Checksum error: %02x %02x", crc, checksum);
        return DECODE_FAIL_MIC;
    }

    // Decode data
    int id              = b[1];
    int wind_dir        = b[2] | (b[3] & 0x80) << 1;
    int low_battery     = (b[3] & 0x08) >> 3;
    int temp_raw        = (b[3] & 0x07) << 8 | b[4];
    float temperature   = (temp_raw - 400) * 0.1f;
    int humidity        = b[5];
    int wind_speed_raw  = b[6] | (b[3] & 0x10) << 4;
    float wind_speed_factor, rain_cup_count;

    if (type == MODEL_WH24) {
        wind_speed_factor = 1.12f;
        rain_cup_count = 0.3f;
    } else {
        wind_speed_factor = 0.51f;
        rain_cup_count = 0.254f;
    }

    float wind_speed_ms = wind_speed_raw * 0.125f * wind_speed_factor;
    int gust_speed_raw  = b[7];
    float gust_speed_ms = gust_speed_raw * wind_speed_factor;
    int rainfall_raw    = b[8] << 8 | b[9];
    float rainfall_mm   = rainfall_raw * rain_cup_count;
    int uv_raw          = b[10] << 8 | b[11];
    int light_raw       = b[12] << 16 | b[13] << 8 | b[14];
    double light_lux    = light_raw * 0.1;

    int uvi_upper[] = {432, 851, 1210, 1570, 2017, 2450, 2761, 3100, 3512, 3918, 4277, 4650, 5029};
    int uv_index   = 0;
    while (uv_index < 13 && uvi_upper[uv_index] < uv_raw) ++uv_index;

    /* clang-format off */
    data = data_make(
            "model",            "",                 DATA_STRING, type == MODEL_WH24 ? "Fineoffset-WH24" : "Fineoffset-WH65B",
            "id",               "ID",               DATA_INT,    id,
            "battery_ok",       "Battery",          DATA_INT,    !low_battery,
            "temperature_C",    "Temperature",      DATA_COND, temp_raw != 0x7ff, DATA_FORMAT, "%.1f C", DATA_DOUBLE, temperature,
            "humidity",         "Humidity",         DATA_COND, humidity != 0xff, DATA_FORMAT, "%u %%", DATA_INT, humidity,
            "wind_dir_deg",     "Wind direction",   DATA_COND, wind_dir != 0x1ff, DATA_INT, wind_dir,
            "wind_avg_m_s",     "Wind speed",       DATA_COND, wind_speed_raw != 0x1ff, DATA_FORMAT, "%.1f m/s", DATA_DOUBLE, wind_speed_ms,
            "wind_max_m_s",     "Gust speed",       DATA_COND, gust_speed_raw != 0xff, DATA_FORMAT, "%.1f m/s", DATA_DOUBLE, gust_speed_ms,
            "rain_mm",          "Rainfall",         DATA_FORMAT, "%.1f mm", DATA_DOUBLE, rainfall_mm,
            "uv",               "UV",               DATA_COND, uv_raw != 0xffff, DATA_INT, uv_raw,
            "uvi",              "UVI",              DATA_COND, uv_raw != 0xffff, DATA_INT, uv_index,
            "light_lux",        "Light",            DATA_COND, light_raw != 0xffffff, DATA_FORMAT, "%.1f lux", DATA_DOUBLE, light_lux,
            "mic",              "Integrity",        DATA_STRING, "CRC",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

static int fineoffset_WH0290_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    data_t *data;
    uint8_t const preamble[] = {0xAA, 0x2D, 0xD4};
    uint8_t b[8];
    unsigned bit_offset;

    bit_offset = bitbuffer_search(bitbuffer, 0, 0, preamble, sizeof(preamble) * 8) + sizeof(preamble) * 8;
    if (bit_offset + sizeof(b) * 8 > bitbuffer->bits_per_row[0]) {
        decoder_logf(decoder, 1, __func__, "short package. Row length: %u. Header index: %u", bitbuffer->bits_per_row[0], bit_offset);
        return DECODE_ABORT_LENGTH;
    }
    bitbuffer_extract_bytes(bitbuffer, 0, bit_offset, b, sizeof(b) * 8);

    // Verify checksum
    uint8_t crc = crc8(b, 6, 0x31, 0x00);
    uint8_t checksum = 0;
    for (unsigned n = 0; n < 7; ++n) {
        checksum += b[n];
    }
    if (crc != b[6] || checksum != b[7]) {
        decoder_logf(decoder, 1, __func__, "Checksum error: %02x %02x", crc, checksum);
        return DECODE_FAIL_MIC;
    }

    // Decode data
    uint8_t family    = b[0];
    uint8_t id        = b[1];
    uint8_t unknown1  = (b[2] & 0x80) ? 1 : 0;
    int pm25          = (b[2] & 0x3f) << 8 | b[3];
    int pm100         = (b[4] & 0x3f) << 8 | b[5];
    int battery_bars  = (b[2] & 0x40) >> 4 | (b[4] & 0xC0) >> 6;
    float battery_ok  = battery_bars * 0.2f;

    /* clang-format off */
    data = data_make(
            "model",            "",             DATA_STRING, "Fineoffset-WH0290",
            "id",               "ID",           DATA_INT,    id,
            "battery_ok",       "Battery Level",  DATA_FORMAT, "%.1f", DATA_DOUBLE, battery_ok,
            "pm2_5_ug_m3",      "2.5um Fine Particulate Matter",  DATA_FORMAT, "%d ug/m3", DATA_INT, pm25/10,
            "estimated_pm10_0_ug_m3",     "Estimate of 10um Coarse Particulate Matter",  DATA_FORMAT, "%d ug/m3", DATA_INT, pm100/10,
            "family",           "FAMILY",       DATA_INT,    family,
            "unknown1",         "UNKNOWN1",     DATA_INT,    unknown1,
            "mic",              "Integrity",    DATA_STRING, "CRC",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

static int fineoffset_WH25_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    data_t *data;
    uint8_t const preamble[] = {0xAA, 0x2D, 0xD4};
    uint8_t b[8];
    int type = 25;
    unsigned bit_offset;

    // Validate package
    if (bitbuffer->bits_per_row[0] < 160) {
        return fineoffset_WH0290_callback(decoder, bitbuffer);
    }
    else if (bitbuffer->bits_per_row[0] < 190) {
        type = 32;
    }
    else if (bitbuffer->bits_per_row[0] < 440) {
        return fineoffset_WH24_callback(decoder, bitbuffer);
    }

    if (bitbuffer->bits_per_row[0] > 510) {
        type = 32;
    }

    // Find a data package
    bit_offset = bitbuffer_search(bitbuffer, 0, 0, preamble, sizeof(preamble) * 8) + sizeof(preamble) * 8;
    if (bit_offset + sizeof(b) * 8 > bitbuffer->bits_per_row[0]) {
        decoder_logf(decoder, 1, __func__, "short package. Header index: %u", bit_offset);
        return DECODE_ABORT_LENGTH;
    }
    bitbuffer_extract_bytes(bitbuffer, 0, bit_offset, b, sizeof(b) * 8);
    decoder_log_bitrow(decoder, 2, __func__, b, sizeof(b) * 8, "Packet");

    // Verify type code
    int msg_type = b[0] & 0xf0;
    if (type == 32 && msg_type == 0xd0) {
        type = 31;
    }
    else if (msg_type != 0xe0) {
        decoder_logf(decoder, 1, __func__, "Msg type unknown: %02x", b[0]);
        if (b[0] == 0x41) {
            return fineoffset_WH0290_callback(decoder, bitbuffer);
        }
        return DECODE_ABORT_EARLY;
    }

    // Verify checksum
    int sum = (add_bytes(b, 6) & 0xff) - b[6];
    if (sum) {
        decoder_log_bitrow(decoder, 1, __func__, b, sizeof (b) * 8, "Checksum error");
        return DECODE_FAIL_MIC;
    }

    // Verify xor-sum
    int bitsum = xor_bytes(b, 6);
    bitsum = ((bitsum & 0x0f) << 4) | (bitsum >> 4);
    if (type == 25 && bitsum != b[7]) {
        decoder_log_bitrow(decoder, 1, __func__, b, sizeof (b) * 8, "Bitsum error");
        return DECODE_FAIL_MIC;
    }

    // Decode data
    uint8_t id        = ((b[0]&0x0f) << 4) | (b[1] >> 4);
    int low_battery   = (b[1] & 0x08) >> 3;
    int temp_raw      = (b[1] & 0x03) << 8 | b[2];
    float temperature = (temp_raw - 400) * 0.1f;
    uint8_t humidity  = b[3];
    int pressure_raw  = (b[4] << 8 | b[5]);
    float pressure    = pressure_raw * 0.1f;

    /* clang-format off */
    data = data_make(
            "model",            "",             DATA_COND, type == 31, DATA_STRING, "Fineoffset-WH32",
            "model",            "",             DATA_COND, type == 32, DATA_STRING, "Fineoffset-WH32B",
            "model",            "",             DATA_COND, type == 25, DATA_STRING, "Fineoffset-WH25",
            "id",               "ID",           DATA_INT,    id,
            "battery_ok",       "Battery",      DATA_INT,    !low_battery,
            "temperature_C",    "Temperature",  DATA_FORMAT, "%.1f C", DATA_DOUBLE, temperature,
            "humidity",         "Humidity",     DATA_FORMAT, "%u %%", DATA_INT, humidity,
            "pressure_hPa",     "Pressure",     DATA_COND,   pressure_raw != 0xffff, DATA_FORMAT, "%.1f hPa", DATA_DOUBLE, pressure,
            "mic",              "Integrity",    DATA_STRING, "CRC",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

static int fineoffset_WH51_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    data_t *data;
    uint8_t const preamble[] = {0xAA, 0x2D, 0xD4};
    uint8_t b[14];
    unsigned bit_offset;

    // Validate package
    if (bitbuffer->bits_per_row[0] < 120) {
        return DECODE_ABORT_LENGTH;
    }

    // Find a data package
    bit_offset = bitbuffer_search(bitbuffer, 0, 0, preamble, sizeof(preamble) * 8) + sizeof(preamble) * 8;
    if (bit_offset + sizeof(b) * 8 > bitbuffer->bits_per_row[0]) {
        decoder_logf(decoder, 1, __func__, "short package. Header index: %u", bit_offset);
        return DECODE_ABORT_LENGTH;
    }
    bitbuffer_extract_bytes(bitbuffer, 0, bit_offset, b, sizeof(b) * 8);

    // Verify family code
    if (b[0] != 0x51) {
        decoder_logf(decoder, 1, __func__, "Msg family unknown: %02x", b[0]);
        return DECODE_ABORT_EARLY;
    }

    // Verify checksum
    if ((add_bytes(b, 13) & 0xff) != b[13]) {
        decoder_log_bitrow(decoder, 1, __func__, b, sizeof (b) * 8, "Checksum error");
        return DECODE_FAIL_MIC;
    }

    // Verify crc
    if (crc8(b, 12, 0x31, 0) != b[12]) {
        decoder_log_bitrow(decoder, 1, __func__, b, sizeof (b) * 8, "Bitsum error");
        return DECODE_FAIL_MIC;
    }

    // Decode data
    char id[7];
    snprintf(id, sizeof(id), "%02x%02x%02x", b[1], b[2], b[3]);
    int boost           = (b[4] & 0xe0) >> 5;
    int battery_mv      = (b[4] & 0x1f) * 100;
    float battery_level = (battery_mv - 700) / 900.0f;
    int ad_raw          = (((int)b[7] & 0x01) << 8) | (int)b[8];
    int moisture        = b[6];

    /* clang-format off */
    data = data_make(
            "model",            "",                 DATA_STRING, "Fineoffset-WH51",
            "id",               "ID",               DATA_STRING, id,
            "battery_ok",       "Battery level",    DATA_DOUBLE, battery_level,
            "battery_mV",       "Battery",          DATA_FORMAT, "%d mV", DATA_INT, battery_mv,
            "moisture",         "Moisture",         DATA_FORMAT, "%u %%", DATA_INT, moisture,
            "boost",            "Transmission boost", DATA_INT, boost,
            "ad_raw",           "AD raw",           DATA_INT, ad_raw,
            "mic",              "Integrity",        DATA_STRING, "CRC",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}
/**
Alecto WS-1200 V1.0 decoder by Christian Zuckschwerdt, documentation by Andreas Untergasser, help by curlyel.

A Thermometer with clock and wireless rain unit with temperature sensor.

Manual available at
https://www.alecto.nl/media/blfa_files/WS-1200_manual_NL-FR-DE-GB_V2.2_8712412532964.pdf

Data layout:

    1111111 FFFFIIII IIIIB?TT TTTTTTTT RRRRRRRR RRRRRRRR 11111111 CCCCCCCC

- 1: 7 bit preamble of 1's
- F: 4 bit fixed message type (0x3)
- I: 8 bit random sensor ID, changes at battery change
- B: 1 bit low battery indicator
- T: 10 bit temperature in Celsius offset 40 scaled by 10
- R: 16 bit (little endian) rain count in 0.3 mm steps, absolute with wrap around at 65536
- C: 8 bit CRC-8 poly 0x31 init 0x0 for 7 bytes

Format string:

    PRE:7b TYPE:4b ID:8b BATT:1b ?:1b T:10d R:<16d ?:8h CRC:8h
*/
static int alecto_ws1200v1_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    data_t *data;
    bitrow_t *bb = bitbuffer->bb;
    uint8_t b[7];

    // Validate package
    if (bitbuffer->bits_per_row[0] != 63
            || (bb[0][0] >> 1) != 0x7F
            || (bb[0][1] >> 5) != 0x3)
        return DECODE_ABORT_LENGTH;

    bitbuffer_extract_bytes(bitbuffer, 0, 7, b, sizeof (b) * 8);

    // Verify checksum
    int crc = crc8(b, 7, 0x31, 0);
    if (crc) {
        decoder_log_bitrow(decoder, 1, __func__, b, sizeof (b) * 8, "Alecto WS-1200 v1.0: CRC error ");
        return DECODE_FAIL_MIC;
    }

    int id            = ((b[0] & 0x0f) << 4) | (b[1] >> 4);
    int battery_low   = (b[1] >> 3) & 0x1;
    int temp_raw      = (b[1] & 0x7) << 8 | b[2];
    float temperature = (temp_raw - 400) * 0.1f;
    int rainfall_raw  = b[4] << 8 | b[3];
    float rainfall    = rainfall_raw * 0.3f;

    /* clang-format off */
    data = data_make(
            "model",            "",             DATA_STRING, "Alecto-WS1200v1",
            "id",               "ID",           DATA_INT,    id,
            "battery_ok",       "Battery",      DATA_INT,    !battery_low,
            "temperature_C",    "Temperature",  DATA_FORMAT, "%.1f C", DATA_DOUBLE, temperature,
            "rain_mm",          "Rain",         DATA_FORMAT, "%.1f mm", DATA_DOUBLE, rainfall,
            "mic",              "Integrity",    DATA_STRING, "CRC",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/**
Alecto WS-1200 V2.0 DCF77 decoder by Christian Zuckschwerdt, documentation by Andreas Untergasser, help by curlyel.

A Thermometer with clock and wireless rain unit with temperature sensor.

Manual available at
https://www.alecto.nl/media/blfa_files/WS-1200_manual_NL-FR-DE-GB_V2.2_8712412532964.pdf

Data layout:

    1111111 FFFFFFFF IIIIIIII B??????? ..YY..YY ..MM..MM ..DD..DD ..HH..HH ..MM..MM ..SS..SS CCCCCCCC AAAAAAAA

- 1: 7 bit preamble of 1's
- F: 8 bit fixed message type (0x52)
- I: 8 bit random sensor ID, changes at battery change
- B: 1 bit low battery indicator
- ?: 7 bit unknown

- T: 10 bit temperature in Celsius offset 40 scaled by 10
- R: 16 bit (little endian) rain count in 0.3 mm steps, absolute with wrap around at 65536
- C: 8 bit CRC-8 poly 0x31 init 0x0 for 10 bytes
- A: 8 bit checksum (addition)

Format string:

    PRE:7b TYPE:8b ID:8b BATT:1b ?:1b ?:8b YY:4d YY:4d MM:4d MM:4d DD:4d DD:4d HH:4d HH:4d MM:4d MM:4d SS:4d SS:4d ?:16b

*/
static int alecto_ws1200v2_dcf_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    data_t *data;
    bitrow_t *bb = bitbuffer->bb;
    uint8_t b[11];

    // Validate package
    if (bitbuffer->bits_per_row[0] != 95
            || (bb[0][0] >> 1) != 0x7F
            || (bb[0][1] >> 1) != 0x52)
        return DECODE_ABORT_LENGTH;

    bitbuffer_extract_bytes(bitbuffer, 0, 7, b, sizeof (b) * 8);

    // Verify CRC
    int crc = crc8(b, 10, 0x31, 0);
    if (crc) {
        return DECODE_FAIL_MIC;
    }
    // Verify checksum
    int sum = add_bytes(b, 10) - b[10];
    if (sum & 0xff) {
        decoder_log_bitrow(decoder, 1, __func__, b, sizeof (b) * 8, "Alecto WS-1200 v2.0 DCF77: Checksum error ");
        return DECODE_FAIL_MIC;
    }

    int id          = (b[1]);
    int battery_low = (b[2] >> 7) & 0x1;
    int date_y      = b[4] + 0x2000;
    int date_m      = b[5];
    int date_d      = b[6];
    int time_h      = b[7];
    int time_m      = b[8];
    int time_s      = b[9];
    char clock_str[32];
    snprintf(clock_str, sizeof(clock_str), "%04x-%02x-%02xT%02x:%02x:%02x",
            date_y, date_m, date_d, time_h, time_m, time_s);

    /* clang-format off */
    data = data_make(
            "model",            "",             DATA_STRING, "Alecto-WS1200v2",
            "id",               "ID",           DATA_INT,    id,
            "battery_ok",       "Battery",      DATA_INT,    !battery_low,
            "radio_clock",      "Radio Clock",  DATA_STRING, clock_str,
            "mic",              "Integrity",    DATA_STRING, "CRC",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/**
Alecto WS-1200 V2.0 decoder by Christian Zuckschwerdt, documentation by Andreas Untergasser, help by curlyel.

A Thermometer with clock and wireless rain unit with temperature sensor.

Manual available at
https://www.alecto.nl/media/blfa_files/WS-1200_manual_NL-FR-DE-GB_V2.2_8712412532964.pdf

Data layout:

    1111111 FFFFIIII IIIIB?TT TTTTTTTT RRRRRRRR RRRRRRRR 11111111 CCCCCCCC AAAAAAAA DDDDDDDD DDDDDDDD DDDDDDDD

- 1: 7 bit preamble of 1's
- F: 4 bit fixed message type (0x3)
- I: 8 bit random sensor ID, changes at battery change
- B: 1 bit low battery indicator
- T: 10 bit temperature in Celsius offset 40 scaled by 10
- R: 16 bit (little endian) rain count in 0.3 mm steps, absolute with wrap around at 65536
- C: 8 bit CRC-8 poly 0x31 init 0x0 for 7 bytes
- A: 8 bit checksum (addition)
- D: 24 bit DCF77 time, all 0 while training for the station connection

Format string:

    PRE:7b TYPE:4b ID:8b BATT:1b ?:1b T:10d R:<16d ?:8h CRC:8h MAC:8h DATE:24b
*/
static int alecto_ws1200v2_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    data_t *data;
    bitrow_t *bb = bitbuffer->bb;
    uint8_t b[11];

    // Validate package
    if (bitbuffer->bits_per_row[0] != 95
            || (bb[0][0] >> 1) != 0x7F
            || (bb[0][1] >> 5) != 0x3)
        return alecto_ws1200v2_dcf_callback(decoder, bitbuffer);

    bitbuffer_extract_bytes(bitbuffer, 0, 7, b, sizeof (b) * 8);

    // Verify CRC
    int crc = crc8(b, 7, 0x31, 0);
    if (crc) {
        decoder_log_bitrow(decoder, 1, __func__, b, sizeof (b) * 8, "Alecto WS-1200 v2.0: CRC error ");
        return DECODE_FAIL_MIC;
    }
    // Verify checksum
    int sum = add_bytes(b, 7) - b[7];
    if (sum & 0xff) {
        decoder_log_bitrow(decoder, 1, __func__, b, sizeof (b) * 8, "Alecto WS-1200 v2.0: Checksum error ");
        return DECODE_FAIL_MIC;
    }

    int id            = ((b[0] & 0x0f) << 4) | (b[1] >> 4);
    int battery_low   = (b[1] >> 3) & 0x1;
    int temp_raw      = (b[1] & 0x7) << 8 | b[2];
    float temperature = (temp_raw - 400) * 0.1f;
    int rainfall_raw  = b[4] << 8 | b[3];
    float rainfall    = rainfall_raw * 0.3f;

    /* clang-format off */
    data = data_make(
            "model",            "",             DATA_STRING, "Alecto-WS1200v2",
            "id",               "ID",           DATA_INT,    id,
            "battery_ok",       "Battery",      DATA_INT,    !battery_low,
            "temperature_C",    "Temperature",  DATA_FORMAT, "%.1f C", DATA_DOUBLE, temperature,
            "rain_mm",          "Rain",         DATA_FORMAT, "%.1f mm", DATA_DOUBLE, rainfall,
            "mic",              "Integrity",    DATA_STRING, "CRC",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/**
Fine Offset Electronics WH0530 Temperature/Rain sensor protocol,
also Agimex Rosenborg 35926 (sold in Denmark).

The sensor sends two identical packages of 71 bits each ~48s. The bits are PWM modulated with On Off Keying.
Data consists of 7 bit preamble and 8 bytes.

Data layout:
    38 a2 8f 02 00 ff e7 51
    FI IT TT RR RR ?? CC AA

- F: 4 bit fixed message type (0x3)
- I: 8 bit Sensor ID (guess). Does not change at battery change.
- B: 1 bit low battery indicator
- T: 11 bit Temperature (+40*10) (Upper bit is Battery Low indicator)
- R: 16 bit (little endian) rain count in 0.3 mm steps, absolute with wrap around at 65536
- ?: 8 bit Always 0xFF (maybe reserved for humidity?)
- C: 8 bit CRC-8 with poly 0x31 init 0x00
- A: 8 bit Checksum of previous 7 bytes (addition truncated to 8 bit)
*/
static int fineoffset_WH0530_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    data_t *data;
    bitrow_t *bb = bitbuffer->bb;
    uint8_t b[8];

    // try Alecto WS-1200 (v1, v2, DCF)
    if (bitbuffer->bits_per_row[0] == 63)
        return alecto_ws1200v1_callback(decoder, bitbuffer);
    if (bitbuffer->bits_per_row[0] == 95)
        return alecto_ws1200v2_callback(decoder, bitbuffer);

    // Validate package
    if (bitbuffer->bits_per_row[0] != 71)
        return DECODE_ABORT_LENGTH;

    if ((bb[0][0] >> 1) != 0x7F
            || (bb[0][1] >> 5) != 0x3)
        return DECODE_ABORT_EARLY;

    bitbuffer_extract_bytes(bitbuffer, 0, 7, b, sizeof(b) * 8);

    // Verify checksum
    int crc = crc8(b, 7, 0x31, 0);
    int sum = (add_bytes(b, 7) & 0xff) - b[7];

    if (crc || sum) {
        decoder_log_bitrow(decoder, 1, __func__, b, sizeof (b) * 8, "Fineoffset_WH0530: Checksum error");
        return DECODE_FAIL_MIC;
    }

    int id            = ((b[0] & 0x0f) << 4) | (b[1] >> 4);
    int battery_low   = (b[1] >> 3) & 0x1;
    int temp_raw      = (b[1] & 0x7) << 8 | b[2];
    float temperature = (temp_raw - 400) * 0.1f;
    int rainfall_raw  = b[4] << 8 | b[3];
    float rainfall    = rainfall_raw * 0.3f;

    /* clang-format off */
    data = data_make(
            "model",            "",             DATA_STRING, "Fineoffset-WH0530",
            "id",               "ID",           DATA_INT,    id,
            "battery_ok",       "Battery",      DATA_INT,    !battery_low,
            "temperature_C",    "Temperature",  DATA_FORMAT, "%.1f C", DATA_DOUBLE, temperature,
            "rain_mm",          "Rain",         DATA_FORMAT, "%.1f mm", DATA_DOUBLE, rainfall,
            "mic",              "Integrity",    DATA_STRING, "CRC",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

// Output fields definitions
static char const *const output_fields[] = {
        "model",
        "id",
        "temperature_C",
        "humidity",
        "mic",
        NULL,
};

static char const *const output_fields_WH25[] = {
        "model",
        "id",
        "battery_ok",
        "temperature_C",
        "humidity",
        "pressure_hPa",
        "wind_dir_deg",
        "wind_avg_m_s",
        "wind_max_m_s",
        "rain_mm",
        "uv",
        "uvi",
        "light_lux",
        "pm2_5_ug_m3",
        "estimated_pm10_0_ug_m3",
        "mic",
        NULL,
};

static char const *const output_fields_WH51[] = {
        "model",
        "id",
        "battery_ok",
        "battery_mV",
        "moisture",
        "boost",
        "ad_raw",
        "mic",
        NULL,
};

static char const *const output_fields_WH0530[] = {
        "model",
        "id",
        "battery_ok",
        "temperature_C",
        "rain_mm",
        "radio_clock",
        "mic",
        NULL,
};

// Device definitions
r_device const fineoffset_WH2 = {
        .name        = "Fine Offset Electronics, WH2, WH5, Telldus Temperature/Humidity/Rain Sensor",
        .modulation  = OOK_PULSE_PWM,
        .short_width = 500,
        .long_width  = 1500,
        .reset_limit = 1200,
        .tolerance   = 160,
        .decode_fn   = &fineoffset_WH2_callback,
        .fields      = output_fields,
};

r_device const fineoffset_WH25 = {
        .name        = "Fine Offset Electronics, WH25, WH32, WH32B, WN32B, WH24, WH65B, HP1000, Misol WS2320 Temperature/Humidity/Pressure Sensor",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 58,
        .long_width  = 58,
        .reset_limit = 20000,
        .decode_fn   = &fineoffset_WH25_callback,
        .fields      = output_fields_WH25,
};

r_device const fineoffset_WH51 = {
        .name        = "Fine Offset Electronics/ECOWITT WH51, SwitchDoc Labs SM23 Soil Moisture Sensor",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 58,
        .long_width  = 58,
        .reset_limit = 5000,
        .decode_fn   = &fineoffset_WH51_callback,
        .fields      = output_fields_WH51,
};

r_device const fineoffset_WH0530 = {
        .name        = "Fine Offset Electronics, WH0530 Temperature/Rain Sensor",
        .modulation  = OOK_PULSE_PWM,
        .short_width = 504,
        .long_width  = 1480,
        .reset_limit = 1200,
        .sync_width  = 0,
        .tolerance   = 160,
        .decode_fn   = &fineoffset_WH0530_callback,
        .fields      = output_fields_WH0530,
};
