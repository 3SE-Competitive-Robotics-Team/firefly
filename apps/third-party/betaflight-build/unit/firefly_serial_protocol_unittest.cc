// SPDX-License-Identifier: GPL-3.0-or-later
#include <array>
#include <algorithm>
#include <deque>
#include <vector>
#include "gtest/gtest.h"

extern "C" {
#include "platform.h"
#include "io/firefly_serial_protocol.h"
#include "io/serial.h"
#include "drivers/system.h"
#include "common/crc.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "rx/rx.h"
#include "sensors/battery.h"
#include "fc/runtime_config.h"
#include "msp/msp_serial.h"
#include "pg/msp.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"

acc_t acc;
gyro_t gyro;
uint8_t armingFlags;
uint16_t flightModeFlags;
attitudeEulerAngles_t attitude;
float motor[MAX_SUPPORTED_MOTORS];
float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
uint8_t getMotorCount(void) { return 4; }
uint16_t motorConvertToExternal(float value) { return (uint16_t)value; }
bool isRxReceivingSignal(void) { return true; }
armingDisableFlags_e getArmingDisableFlags(void) { return ARMING_DISABLED_RX_FAILSAFE; }
failsafePhase_e failsafePhase(void) { return FAILSAFE_IDLE; }
batteryState_e getBatteryState(void) { return BATTERY_WARNING; }
uint8_t getBatteryCellCount(void) { return 1; }
bool isBatteryVoltageConfigured(void) { return true; }
bool isAmperageConfigured(void) { return true; }
uint16_t getBatteryVoltageLatest(void) { return 381; }
int32_t getAmperageLatest(void) { return -12; }
int32_t getMAhDrawn(void) { return 43; }
int16_t getCoreTemperatureCelsius(void) { return 39; }
serialConfig_t serialConfig_System;
mspConfig_t mspConfig_System;
const uint32_t baudRates[BAUD_COUNT] = {};
}

static serialPort_t usb;
static serialPortConfig_t config;
static std::deque<uint8_t> input;
static std::vector<uint8_t> output;
static unsigned txSpace = 20;
static uint32_t usbConnection = 1;
static timeUs_t now;

extern "C" {
uint32_t usbVcpConnectionId(void) { return usbConnection; }
void usbVcpTryWriteFrame(uint32_t connection, const uint8_t *data, unsigned length)
{
    if (connection == usbConnection && length <= txSpace) {
        output.insert(output.end(), data, data + length);
    }
}
timeUs_t micros(void) { return now; }
timeMs_t millis(void) { return now / 1000; }
mspDescriptor_t mspDescriptorAlloc(void) { return 1; }
const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e) { return &config; }
const serialPortConfig_t *findNextSerialPortConfig(serialPortFunction_e) { return nullptr; }
const serialPortConfig_t *serialFindPortConfiguration(serialPortIdentifier_e) { return &config; }
bool isSerialPortShared(const serialPortConfig_t *, uint16_t, serialPortFunction_e) { return false; }
serialType_e serialType(serialPortIdentifier_e) { return SERIALTYPE_USB_VCP; }
void closeSerialPort(serialPort_t *) {}
void systemResetToBootloader(bootloaderRequestType_e) { ADD_FAILURE(); }
void waitForSerialPortToFinishTransmitting(serialPort_t *) {}
bool isSerialTransmitBufferEmpty(const serialPort_t *) { return true; }
void serialBeginWrite(serialPort_t *) {}
void serialEndWrite(serialPort_t *) {}

serialPort_t *openSerialPort(serialPortIdentifier_e id, serialPortFunction_e function,
    serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e)
{
    EXPECT_EQ(FUNCTION_MSP, function);
    usb.identifier = id;
    return &usb;
}

uint32_t serialRxBytesWaiting(const serialPort_t *) { return input.size(); }
uint32_t serialTxBytesFree(const serialPort_t *) { return txSpace; }
uint8_t serialRead(serialPort_t *)
{
    const auto byte = input.front();
    input.pop_front();
    return byte;
}
void serialWriteBuf(serialPort_t *, const uint8_t *bytes, int count)
{
    output.insert(output.end(), bytes, bytes + count);
}
void serialWriteBufNoFlush(serialPort_t *port, const uint8_t *bytes, int count)
{
    serialWriteBuf(port, bytes, count);
}
}

TEST(FireflySerial, ImuFrameAndBackpressure)
{
    fireflySerialStart(&usb, usbConnection, 0x12345678);
    acc.dev.acc_1G_rec = 1.0f / 2048;
    acc.accADC.x = 1024;
    acc.accADC.y = -512;
    acc.accADC.z = 2048;
    gyro.gyroADCf[0] = -12.3f;
    gyro.gyroADCf[1] = 0.5f;
    gyro.gyroADCf[2] = 2000;
    output.clear();
    txSpace = 19;
    fireflySerialSendImu(0x12345678);
    EXPECT_TRUE(output.empty());
    txSpace = 20;
    fireflySerialSendImu(0x12345678);
    const std::vector<uint8_t> expected = {
        0x46,0x49,0x78,0x56,0x34,0x12,0xf4,0x01,0x06,0xff,
        0xe8,0x03,0x85,0xff,0x05,0x00,0x20,0x4e,0x71,0x08
    };
    EXPECT_EQ(expected, output);
}

TEST(FireflySerial, SplitCommandsResyncExpiryAndWrap)
{
    const uint32_t start = UINT32_MAX - 1000;
    fireflySerialStart(&usb, usbConnection, start);
    input.clear();
    fireflySerialProcess(&usb, start);
    std::array<float, 5> radio = {1500, 1500, 1500, 1200, 1840};
    const auto original = radio;
    const std::vector<uint8_t> valid = {0x46,0x43,0xd2,0x04,0xdc,0x05,0xf1,0x06,0xe8,0x03,0x9c,0x75};
    input = {0x46,0x43,0x99,0x46,0x46,0x43,0xd2};
    fireflySerialProcess(&usb, start + 10);
    fireflySerialApplyControl(start + 10, radio.data(), true);
    EXPECT_EQ(original, radio);
    input.insert(input.end(), valid.begin() + 3, valid.end());
    fireflySerialProcess(&usb, start + 20);
    fireflySerialApplyControl(start + 2000, radio.data(), true);
    EXPECT_EQ((std::array<float,5>{1234,1500,1777,1000,1840}), radio);

    input.insert(input.end(), valid.begin(), valid.end());
    input.back() ^= 1;
    // Correct CRC, but the roll channel is outside the accepted range.
    input.insert(input.end(), {0x46,0x43,0xd1,0x07,0xdc,0x05,0xf1,0x06,0xe8,0x03,0x6b,0x65});
    fireflySerialProcess(&usb, start + 4000);
    radio = original;
    fireflySerialApplyControl(start + 100020, radio.data(), true);
    EXPECT_EQ(original, radio);
    // A scheduler stall discards commands already buffered by the host link.
    input.insert(input.end(), valid.begin(), valid.end());
    fireflySerialProcess(&usb, start + 105000);
    fireflySerialApplyControl(start + 105001, radio.data(), true);
    EXPECT_EQ(original, radio);
}

TEST(FireflySerial, StatusYieldsToControlAndImuAndSkipsMissedSlots)
{
    const uint32_t start = UINT32_MAX - 20000;
    fireflySerialStart(&usb, usbConnection, start);
    output.clear();
    input = {0x46,0x43,0xd2,0x04,0xdc,0x05,0xf1,0x06,0xe8,0x03,0x9c,0x75};
    txSpace = 68 + 8 * 20 - 1;
    fireflySerialProcess(&usb, start + 60000);
    input = {0x46,0x43,0xd2,0x04,0xdc,0x05,0xf1,0x06,0xe8,0x03,0x9c,0x75};
    fireflySerialObserveCurrent(1200);
    fireflySerialObserveVoltage(329);
    fireflySerialProcess(&usb, start + 100000);
    EXPECT_TRUE(output.empty());
    fireflySerialApplyControl(start + 100000, rcData, true);
    EXPECT_EQ(1234, rcData[0]);
    fireflySerialSendImu(start + 100000);
    EXPECT_EQ(20u, output.size());
    output.clear();
    txSpace = 228;
    fireflySerialProcess(&usb, start + 199999);
    EXPECT_TRUE(output.empty());
    attitude.values.roll = -123;
    fireflySerialRecordFailsafe(start + 41000, FAILSAFE_RX_LOSS_DETECTED);
    fireflySerialRecordDisarm(start + 42000, 6);
    fireflySerialProcess(&usb, start + 300000);
    ASSERT_EQ(68u, output.size());
    EXPECT_EQ('T', output[1]);
    EXPECT_EQ(3, output[6]);
    EXPECT_EQ(0x85, output[10]);
    EXPECT_EQ(0xff, output[11]);
    EXPECT_EQ(6, output[41]);
    EXPECT_EQ(FAILSAFE_RX_LOSS_DETECTED, output[50]);
    EXPECT_EQ(381, output[54] | output[55] << 8);
    EXPECT_EQ(0xfff4, output[56] | output[57] << 8);
    EXPECT_EQ(329, output[62] | output[63] << 8);
    EXPECT_EQ(1200, output[64] | output[65] << 8);
    EXPECT_EQ(crc16_ccitt_update(0, output.data(), 66), output[66] | output[67] << 8);
}

static const std::vector<uint8_t> enter = {'$', 'X', '<', 0, 1, 0x3f, 0, 0, 0xab};
static const std::vector<uint8_t> ack   = {'$', 'X', '>', 0, 1, 0x3f, 0, 0, 0xab};

class FireflyUsb : public ::testing::Test {
protected:
    void SetUp() override
    {
        ++usbConnection;
        fireflySerialProcess(&usb, 0);
        now = 100;
        config.identifier = SERIAL_PORT_USB_VCP;
        mspSerialInit();
        armingFlags = 0;
        txSpace = 64;
        input.clear();
        output.clear();
    }

    static void poll()
    {
        mspSerialProcess(MSP_EVALUATE_NON_MSP_DATA,
            [](mspDescriptor_t, mspPacket_t *cmd, mspPacket_t *reply, mspPostProcessFnPtr *) {
                reply->cmd = cmd->cmd;
                return MSP_RESULT_ACK;
            }, nullptr);
    }

    static void send(const std::vector<uint8_t>& bytes)
    {
        input.insert(input.end(), bytes.begin(), bytes.end());
        poll();
    }
};

TEST_F(FireflyUsb, OnlyDisarmedNativeUsbRequestCanEnter)
{
    fireflySerialSendImu(now);
    EXPECT_TRUE(output.empty());
    auto corrupt = enter;
    corrupt.back() ^= 1;
    send(corrupt);
    EXPECT_TRUE(output.empty());
    ASSERT_FALSE(fireflySerialOwnsPort(&usb));
    armingFlags = ARMED;
    send(enter);
    ASSERT_EQ(9U, output.size());
    EXPECT_EQ('!', output[2]);
    ASSERT_FALSE(fireflySerialOwnsPort(&usb));
    armingFlags = 0;
    output.clear();
    send({'$','X','<',1,1,0x3f,0,0,0x1d});
    send({'$','X','<',0,1,0x3f,1,0,0x11,0xcc});
    ASSERT_EQ(18U, output.size());
    EXPECT_EQ('!', output[2]);
    EXPECT_EQ('!', output[11]);
    EXPECT_FALSE(fireflySerialOwnsPort(&usb));
    usb.identifier = SERIAL_PORT_USART2;
    send(enter);
    EXPECT_FALSE(fireflySerialOwnsPort(&usb));
    usb.identifier = SERIAL_PORT_USB_VCP;
    txSpace = 8;
    output.clear();
    send(enter);
    EXPECT_TRUE(output.empty());
    EXPECT_FALSE(fireflySerialOwnsPort(&usb));
    txSpace = 64;
    send(std::vector<uint8_t>(enter.begin(), enter.begin() + 4));
    EXPECT_TRUE(output.empty());
    send(std::vector<uint8_t>(enter.begin() + 4, enter.end()));
    fireflySerialSendImu(now);
    ASSERT_EQ(ack.size() + 20, output.size());
    EXPECT_TRUE(std::equal(ack.begin(), ack.end(), output.begin()));
    EXPECT_EQ('F', output[ack.size()]);
    EXPECT_EQ('I', output[ack.size() + 1]);
}

TEST_F(FireflyUsb, OnlyValidUplinkRenewsSessionAndHeartbeatNeverRenewsControl)
{
    send(enter);
    send({0x46,0x43,0xd2,0x04,0xdc,0x05,0xf1,0x06,0xe8,0x03,0x9c,0x75});
    for (unsigned i = 0; i < 3; ++i) {
        now += 50000;
        send({'F','H',0xa6,0x6e});
    }
    std::array<float, 4> radio = {1500,1500,1500,1000};
    const auto original = radio;
    fireflySerialApplyControl(now, radio.data(), true);
    EXPECT_EQ(original, radio);
    for (unsigned i = 0; i < 19; ++i) {
        now += 50000;
        send({'F','H',0,0});
        ASSERT_TRUE(fireflySerialOwnsPort(&usb));
    }
    now += 50000;
    poll();
    EXPECT_FALSE(fireflySerialOwnsPort(&usb));
    output.clear();
    send(enter);
    EXPECT_EQ(ack, output);
}

TEST_F(FireflyUsb, NewEnumerationReceivesMspEvenBetweenPolls)
{
    send(enter);
    send({'F','C',0xd2});
    output.clear();
    ++usbConnection;
    fireflySerialSendImu(now);
    EXPECT_TRUE(output.empty());
    send({'$', 'M', '<', 0, 1, 1});
    EXPECT_FALSE(fireflySerialOwnsPort(&usb));
    EXPECT_EQ((std::vector<uint8_t>{'$', 'M', '>', 0, 1, 1}), output);
}

TEST_F(FireflyUsb, DisconnectRevokesControlBeforeNextSerialPoll)
{
    send(enter);
    send({0x46,0x43,0xd2,0x04,0xdc,0x05,0xf1,0x06,0xe8,0x03,0x9c,0x75});
    std::array<float, 4> radio = {1500,1500,1500,1000};
    const auto original = radio;
    fireflySerialApplyControl(now, radio.data(), true);
    EXPECT_NE(original, radio);
    usbConnection = 0;
    radio = original;
    fireflySerialApplyControl(now, radio.data(), true);
    EXPECT_EQ(original, radio);
    poll();
    EXPECT_FALSE(fireflySerialOwnsPort(&usb));
    usbConnection = 1;
}
