#include <unity.h>
#include <NativeTestHelper.h>

#include <cstring>

#include "../../src/ArcCommandBridge.h"

using namespace astra_rocket;

static void injectBytes(Stream &stream, const uint8_t *data, size_t len)
{
    stream.clearBuffer();
    if (len > sizeof(stream.inputBuffer))
        len = sizeof(stream.inputBuffer);
    std::memcpy(stream.inputBuffer, data, len);
    stream.inputLength = static_cast<int>(len);
    stream.inputCursor = 0;
}

static size_t buildEncodedFrame(uint8_t *out,
                                uint8_t src,
                                uint8_t dst,
                                uint8_t flags,
                                uint8_t session,
                                uint16_t seq,
                                uint8_t family,
                                uint8_t type,
                                const uint8_t *payload = nullptr,
                                size_t payloadLen = 0)
{
    uint8_t raw[ARC_MAX_FRAME_SIZE];
    int rawLen = arc_frame_build(raw,
                                 sizeof(raw),
                                 src,
                                 dst,
                                 flags,
                                 session,
                                 seq,
                                 family,
                                 type,
                                 payload,
                                 payloadLen);
    TEST_ASSERT_GREATER_OR_EQUAL(0, rawLen);
    int encodedLen = arc_cobs_encode(raw, static_cast<size_t>(rawLen), out, ARC_MAX_ENCODED_SIZE);
    TEST_ASSERT_GREATER_OR_EQUAL(0, encodedLen);
    return static_cast<size_t>(encodedLen);
}

static size_t readEncodedFrame(const Stream &stream, size_t offset, arc_frame_t *frame, uint8_t *decoded)
{
    size_t end = offset;
    while (end < static_cast<size_t>(stream.cursor) && stream.fakeBuffer[end] != '\0')
        ++end;
    TEST_ASSERT_LESS_THAN(static_cast<size_t>(stream.cursor), end);

    uint8_t encoded[ARC_MAX_ENCODED_SIZE];
    size_t encodedLen = end - offset + 1;
    TEST_ASSERT_LESS_OR_EQUAL(sizeof(encoded), encodedLen);
    std::memcpy(encoded, stream.fakeBuffer + offset, encodedLen);
    int decodedLen = arc_cobs_decode(encoded, encodedLen, decoded, ARC_MAX_FRAME_SIZE);
    TEST_ASSERT_GREATER_OR_EQUAL(0, decodedLen);
    TEST_ASSERT_EQUAL(ARC_OK, arc_frame_parse(decoded, static_cast<size_t>(decodedLen), frame));
    return end + 1;
}

void setUp(void)
{
}

void tearDown(void)
{
}

void test_reliable_heartbeat_receives_ack()
{
    Stream arcStream;
    ArcCommandBridge bridge(ARC_ADDR_FC_N, &arcStream);
    bridge.begin(3, 100);

    uint8_t encoded[ARC_MAX_ENCODED_SIZE];
    size_t encodedLen = buildEncodedFrame(encoded,
                                          ARC_ADDR_CONTROLLER,
                                          ARC_ADDR_FC_N,
                                          ARC_FLAG_RELIABLE,
                                          9,
                                          42,
                                          ARC_FAMILY_NETMGMT,
                                          ARC_NETMGMT_HEARTBEAT);
    injectBytes(arcStream, encoded, encodedLen);
    bridge.update();

    uint8_t decoded[ARC_MAX_FRAME_SIZE];
    arc_frame_t reply;
    readEncodedFrame(arcStream, 0, &reply, decoded);

    TEST_ASSERT_EQUAL_UINT8(ARC_ADDR_FC_N, reply.src);
    TEST_ASSERT_EQUAL_UINT8(ARC_ADDR_CONTROLLER, reply.dst);
    TEST_ASSERT_EQUAL_UINT8(ARC_FAMILY_NETMGMT, reply.family);
    TEST_ASSERT_EQUAL_UINT8(ARC_NETMGMT_ACK, reply.type);
    TEST_ASSERT_EQUAL_UINT8(2, reply.payload_len);
    TEST_ASSERT_EQUAL_UINT8(0, reply.payload[0]);
    TEST_ASSERT_EQUAL_UINT8(42, reply.payload[1]);
}

void test_get_status_replies_with_arc_status_report()
{
    Stream arcStream;
    ArcCommandBridge bridge(ARC_ADDR_FC_N, &arcStream);
    bridge.begin(3, 100);

    uint8_t encoded[ARC_MAX_ENCODED_SIZE];
    size_t encodedLen = buildEncodedFrame(encoded,
                                          ARC_ADDR_CONTROLLER,
                                          ARC_ADDR_FC_N,
                                          0,
                                          9,
                                          43,
                                          ARC_FAMILY_FC_COORD,
                                          ASTRA_ROCKET_ARC_GET_STATUS);
    injectBytes(arcStream, encoded, encodedLen);
    bridge.update();

    uint8_t decoded[ARC_MAX_FRAME_SIZE];
    arc_frame_t reply;
    readEncodedFrame(arcStream, 0, &reply, decoded);

    TEST_ASSERT_EQUAL_UINT8(ARC_ADDR_FC_N, reply.src);
    TEST_ASSERT_EQUAL_UINT8(ARC_ADDR_CONTROLLER, reply.dst);
    TEST_ASSERT_EQUAL_UINT8(ARC_FAMILY_FC_COORD, reply.family);
    TEST_ASSERT_EQUAL_UINT8(ASTRA_ROCKET_ARC_STATUS_REPORT, reply.type);
    char payload[ARC_MAX_PAYLOAD_SIZE + 1];
    std::memcpy(payload, reply.payload, reply.payload_len);
    payload[reply.payload_len] = '\0';
    TEST_ASSERT_NOT_NULL(std::strstr(payload, "name=AstraRocket"));
    TEST_ASSERT_NOT_NULL(std::strstr(payload, "ready=0"));
}

void test_reliable_text_ping_gets_ack_and_pong()
{
    Stream arcStream;
    ArcCommandBridge bridge(ARC_ADDR_FC_N, &arcStream);
    bridge.begin(3, 100);

    const char ping[] = "PING bench";
    uint8_t encoded[ARC_MAX_ENCODED_SIZE];
    size_t encodedLen = buildEncodedFrame(encoded,
                                          ARC_ADDR_CONTROLLER,
                                          ARC_ADDR_FC_N,
                                          ARC_FLAG_RELIABLE,
                                          9,
                                          44,
                                          ARC_FAMILY_FC_COORD,
                                          ASTRA_ROCKET_ARC_TEXT_COMMAND,
                                          reinterpret_cast<const uint8_t *>(ping),
                                          std::strlen(ping));
    injectBytes(arcStream, encoded, encodedLen);
    bridge.update();

    uint8_t decoded1[ARC_MAX_FRAME_SIZE];
    uint8_t decoded2[ARC_MAX_FRAME_SIZE];
    arc_frame_t ack;
    arc_frame_t pong;
    size_t next = readEncodedFrame(arcStream, 0, &ack, decoded1);
    readEncodedFrame(arcStream, next, &pong, decoded2);

    TEST_ASSERT_EQUAL_UINT8(ARC_FAMILY_NETMGMT, ack.family);
    TEST_ASSERT_EQUAL_UINT8(ARC_NETMGMT_ACK, ack.type);
    TEST_ASSERT_EQUAL_UINT8(ARC_FAMILY_FC_COORD, pong.family);
    TEST_ASSERT_EQUAL_UINT8(ASTRA_ROCKET_ARC_TEXT_RESPONSE, pong.type);
    TEST_ASSERT_EQUAL_STRING_LEN("PONG AstraRocket",
                                 reinterpret_cast<const char *>(pong.payload),
                                 pong.payload_len);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_reliable_heartbeat_receives_ack);
    RUN_TEST(test_get_status_replies_with_arc_status_report);
    RUN_TEST(test_reliable_text_ping_gets_ack_and_pong);
    return UNITY_END();
}
