// SPDX-License-Identifier: GPL-3.0-or-later
#include <array>
#include <deque>
#include <vector>
#include "gtest/gtest.h"

extern "C" {
#include "platform.h"
#include "io/firefly_serial_protocol.h"
#include "io/serial.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"

acc_t acc;
gyro_t gyro;
}

static serialPort_t uart;
static serialPortConfig_t config;
static std::deque<uint8_t> input;
static std::vector<uint8_t> output;
static unsigned txSpace = 20;

extern "C" {
const serialPortConfig_t *serialFindPortConfiguration(serialPortIdentifier_e id)
{
    EXPECT_EQ(SERIAL_PORT_USART1, id);
    config.identifier = id;
    return &config;
}

serialPort_t *openSerialPort(serialPortIdentifier_e id, serialPortFunction_e function,
    serialReceiveCallbackPtr, void *, uint32_t baud, portMode_e mode, portOptions_e)
{
    EXPECT_EQ(SERIAL_PORT_USART1, id);
    EXPECT_EQ(FUNCTION_FIREFLY, function);
    EXPECT_EQ(460800U, baud);
    EXPECT_EQ(MODE_RXTX, mode);
    return &uart;
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
}

TEST(FireflySerial, ImuFrameAndBackpressure)
{
    fireflySerialInit();
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
    fireflySerialInit();
    const uint32_t start = UINT32_MAX - 1000;
    input.clear();
    fireflySerialReceive(start);
    std::array<float, 5> radio = {1500, 1500, 1500, 1200, 1840};
    const auto original = radio;
    const std::vector<uint8_t> valid = {0x46,0x43,0xd2,0x04,0xdc,0x05,0xf1,0x06,0xe8,0x03,0x9c,0x75};
    input = {0x46,0x43,0x99,0x46,0x46,0x43,0xd2};
    fireflySerialReceive(start + 10);
    fireflySerialApplyControl(start + 10, radio.data());
    EXPECT_EQ(original, radio);
    input.insert(input.end(), valid.begin() + 3, valid.end());
    fireflySerialReceive(start + 20);
    fireflySerialApplyControl(start + 2000, radio.data());
    EXPECT_EQ((std::array<float,5>{1234,1500,1777,1000,1840}), radio);

    input.insert(input.end(), valid.begin(), valid.end());
    input.back() ^= 1;
    // Correct CRC, but the roll channel is outside the accepted range.
    input.insert(input.end(), {0x46,0x43,0xd1,0x07,0xdc,0x05,0xf1,0x06,0xe8,0x03,0x6b,0x65});
    fireflySerialReceive(start + 4000);
    radio = original;
    fireflySerialApplyControl(start + 100020, radio.data());
    EXPECT_EQ(original, radio);
    // A scheduler stall discards commands already buffered in the UART.
    input.insert(input.end(), valid.begin(), valid.end());
    fireflySerialReceive(start + 105000);
    fireflySerialApplyControl(start + 105001, radio.data());
    EXPECT_EQ(original, radio);
}
