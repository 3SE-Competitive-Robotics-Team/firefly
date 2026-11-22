override TEST_SRCS := unit/firefly_serial_protocol_unittest.cc
vpath unit/%.cc ../../../betaflight-build

firefly_serial_protocol_unittest_DEFINES := USE_FIREFLY_SERIAL_PROTOCOL=
firefly_serial_protocol_unittest_SRC = $(addprefix $(USER_DIR)/, \
    io/firefly_serial_protocol.c msp/msp_serial.c common/crc.c common/streambuf.c)

include Makefile
