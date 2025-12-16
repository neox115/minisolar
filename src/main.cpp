#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <I2CDefinitions.h>
#include "generated/telemetry.pb.h"

#if MESHTASTIC_I2C_SENSOR_HOST

#include <I2CHost.h>
#include <Adafruit_MAX31865.h>

// ==== Pin definitions (XIAO ESP32S3) ====
#define I2C_SDA_PIN   5      // INA228 SDA
#define I2C_SCL_PIN   6      // INA228 SCL

#define SPI_SCK_PIN   7      // MAX31865 SCK
#define SPI_MISO_PIN  8      // MAX31865 MISO
#define SPI_MOSI_PIN  9      // MAX31865 MOSI
#define RTD_CS_PIN    4      // MAX31865 CS

// ==== PT100 / MAX31865 constants ====
#define PT100_RREF      430.0f   // 基準抵抗 (ストロベリーリナックス基板も430Ω)
#define PT100_RNOMINAL  100.0f   // 公称抵抗 100Ω (PT100)

// ==== INA228 defines ====
// I2C address (ジャンパ設定どおり)
#define INA228_ADDR         0x40

// レジスタアドレス（マニュアル記載どおり）
// CONFIG: 0x00, CURRENT: 0x07 を利用
#define INA228_REG_CONFIG   0x00
#define INA228_REG_CURRENT  0x07

meshtastic_EnvironmentMetrics metrics = meshtastic_EnvironmentMetrics_init_zero;

// ---- INA228 low-level helpers ----
static void ina228WriteRegister16(uint8_t reg, uint16_t value)
{
    Wire.beginTransmission(INA228_ADDR);
    Wire.write(reg);
    Wire.write((uint8_t)(value >> 8));
    Wire.write((uint8_t)(value & 0xFF));
    Wire.endTransmission();
}

static bool ina228ReadRegister16(uint8_t reg, uint16_t &value)
{
    Wire.beginTransmission(INA228_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) { // repeated start
        return false;
    }

    if (Wire.requestFrom(INA228_ADDR, (uint8_t)2) != 2) {
        return false;
    }

    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    value = (uint16_t(hi) << 8) | lo;
    return true;
}

static bool ina228ReadRegister24(uint8_t reg, uint8_t &b1, uint8_t &b2, uint8_t &b3)
{
    Wire.beginTransmission(INA228_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) { // repeated start
        return false;
    }

    if (Wire.requestFrom(INA228_ADDR, (uint8_t)3) != 3) {
        return false;
    }

    b1 = Wire.read();
    b2 = Wire.read();
    b3 = Wire.read();
    return true;
}

// CONFIGレジスタ bit4 = 1 で Low レンジに設定
static void ina228ConfigureLowRange()
{
    uint16_t config = 0;
    if (!ina228ReadRegister16(INA228_REG_CONFIG, config)) {
        Serial.println("INA228 CONFIG read failed");
        return;
    }

    // bit4 を 1 にして Low レンジへ
    config |= (1U << 4);

    ina228WriteRegister16(INA228_REG_CONFIG, config);

    Serial.print("INA228 CONFIG set to 0x");
    Serial.println(config, HEX);
}

// CURRENTレジスタ(0x07)から20bit符号付き値を読む
static int32_t ina228ReadRawCurrent20bit()
{
    uint8_t b1, b2, b3;
    if (!ina228ReadRegister24(INA228_REG_CURRENT, b1, b2, b3)) {
        Serial.println("INA228 CURRENT read failed");
        return 0;
    }

    // マニュアルどおり: 電流・電圧は24bit読み出し、下位4bitは捨てる
    // [b1 b2 b3] の上位20bitが有効
    uint32_t raw20 = (uint32_t(b1) << 12) | (uint32_t(b2) << 4) | (uint32_t(b3) >> 4);

    // 20bit 2の補数 → 32bit符号拡張
    if (raw20 & 0x80000) {         // bit19 が1なら負数
        raw20 |= 0xFFF00000;       // 上位ビットを1で埋める
    }

    return (int32_t)raw20;
}

// ==== MAX31865 instance ====
Adafruit_MAX31865 rtd(RTD_CS_PIN);

// ==== setup ====
void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("Initializing Meshtastic I2C Sensor host (XIAO ESP32S3)...");

    // I2C (master) for INA228 & Meshtastic client(0x11)
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    // SPI for MAX31865
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, RTD_CS_PIN);

    // MAX31865 (3線式PT100)
    if (!rtd.begin(MAX31865_3WIRE)) {
        Serial.println("MAX31865 init failed!");
    } else {
        Serial.println("MAX31865 init OK");
    }

    // INA228 Lowレンジ設定
    ina228ConfigureLowRange();
}

// ==== loop ====
void loop()
{
    // 1) INA228: 20bit生値
    int32_t raw20 = ina228ReadRawCurrent20bit();

    // Lowレンジ: 読み値 * 0.0390625 = mA直読
    //            読み値 / 25600 = A直読
    float current_A  = (float)raw20 / 25600.0f;      // ユーザ指定どおり
    float current_mA = current_A * 1000.0f;

    // 2) MAX31865 (PT100 裏面温度) [℃]
    float tempC = rtd.temperature(PT100_RNOMINAL, PT100_RREF);

    // 3) Meshtastic EnvironmentMetrics に詰める
    metrics = meshtastic_EnvironmentMetrics_init_zero;

    metrics.has_current = true;
    metrics.current     = current_A;   // 単位: A（InfluxDB側で"isc"として扱う）

    metrics.has_temperature = true;
    metrics.temperature     = tempC;   // 単位: ℃（InfluxDB側で"temp"）

    // デバッグ出力
    Serial.print("raw20=");
    Serial.print(raw20);
    Serial.print("  Isc[mA]=");
    Serial.print(current_mA, 3);
    Serial.print("  Temp[C]=");
    Serial.println(tempC, 2);

    // 4) Meshtastic ノード(I2Cクライアント, 0x11)へ送信
    sendMetrics(metrics);

    delay(1000);  // 計測周期
}

#else   // MESHTASTIC_I2C_SENSOR_HOST == 0 → クライアント例（元のまま）

#include <I2CClient.h>

void setup()
{
    Serial.begin(115200);
    Serial.println("Initializing Meshtastic I2C Sensor device (client)...");
    Wire.begin(MT_I2C_ADDRESS);
    Wire.onReceive(onReceiveMetrics);
}

void loop()
{
    delay(1000);
}

#endif
