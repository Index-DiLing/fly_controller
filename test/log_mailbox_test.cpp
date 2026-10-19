#include <stdio.h>
#include <string.h>
#include "app/log/app_log_mailbox.hpp"

static unsigned checks   = 0u;
static unsigned failures = 0u;
#define CHECK(condition)                                        \
    do {                                                        \
        ++checks;                                               \
        if (!(condition)) {                                     \
            ++failures;                                         \
            printf("FAIL line %d: %s\n", __LINE__, #condition); \
        }                                                       \
    } while (false)

struct Fixture {
    uint8_t first[dlx::textmbox::POOL_BYTES]{};
    uint8_t second[dlx::textmbox::POOL_BYTES]{};
    uint8_t output[1024]{};
    dlx::ByteBuffer firstBuffer{first, sizeof(first)};
    dlx::ByteBuffer secondBuffer{second, sizeof(second)};
    dlx::ByteBuffer buffer{output, sizeof(output)};
    dlx::TextMailbox text{firstBuffer, secondBuffer};
    dlx::ExceptionMailbox exceptions;
    dlx::LogChannel channel{&text, &exceptions};
    unsigned exceptionFrames{0u};
    unsigned textFrames{0u};
    uint16_t lastException{0u};
    uint16_t textBytes{0u};

    bool DLXExceptionW(uint16_t code)
    {
        if (buffer.remaining() < 4u) return false;
        ++exceptionFrames;
        lastException = code;
        buffer.write<uint16_t>(0u);
        buffer.write<uint16_t>(code);
        return true;
    }

    bool SimpleLogW(int8_t *data, uint16_t size)
    {
        if (buffer.remaining() < size + 4u) return false;
        ++textFrames;
        textBytes = size;
        buffer.write<uint32_t>(0u);
        buffer.write(reinterpret_cast<uint8_t *>(data), size);
        return true;
    }
};

static void testStartupAndHandoff()
{
    Fixture fixture;
    for (unsigned index = 0u; index < 40u; ++index) {
        CHECK(fixture.text.post("boot\r\n", 6u));
        dlx::appendLogMailbox(&fixture.channel, fixture, fixture.buffer);
        CHECK(fixture.buffer.used() == 10u);
        fixture.buffer.reset();
    }
    CHECK(fixture.textFrames == 40u);
    CHECK(fixture.text.droppedLines() == 0u);
    CHECK(fixture.text.droppedBytes() == 0u);
    CHECK(fixture.text.post("runtime\n", 8u));
    dlx::appendLogMailbox(&fixture.channel, fixture, fixture.buffer);
    CHECK(fixture.textFrames == 41u);
    CHECK(fixture.textBytes == 8u);
}

static void testBackpressure()
{
    Fixture fixture;
    CHECK(fixture.text.post("keep", 4u));
    fixture.buffer.cur += 800u;
    dlx::appendLogMailbox(&fixture.channel, fixture, fixture.buffer);
    CHECK(fixture.textFrames == 0u);
    CHECK(fixture.text.droppedBytes() == 0u);
    fixture.buffer.reset();
    dlx::appendLogMailbox(&fixture.channel, fixture, fixture.buffer);
    CHECK(fixture.textFrames == 1u);
    CHECK(fixture.textBytes == 4u);
    CHECK(memcmp(fixture.output + 4u, "keep", 4u) == 0);
}

static void testExceptions()
{
    Fixture fixture;
    for (uint16_t index = 0u; index < dlx::logchannel::EXCEPTION_SLOTS; ++index) {
        CHECK(fixture.exceptions.push(dlx::ExceptionCode::BARO_READ_FAILED));
    }
    CHECK(!fixture.exceptions.push(dlx::ExceptionCode::BARO_DATA_STALE));
    dlx::appendLogMailbox(&fixture.channel, fixture, fixture.buffer);
    CHECK(fixture.exceptionFrames == dlx::logchannel::EXCEPTION_SLOTS + 1u);
    CHECK(fixture.lastException == static_cast<uint16_t>(dlx::ExceptionCode::EXCEPTION_OVERFLOW));
    fixture.buffer.reset();
    dlx::appendLogMailbox(&fixture.channel, fixture, fixture.buffer);
    CHECK(fixture.buffer.used() == 0u);
    CHECK(fixture.exceptions.push(dlx::ExceptionCode::BARO_DATA_STALE));
    fixture.buffer.cur += 1022u;
    dlx::appendLogMailbox(&fixture.channel, fixture, fixture.buffer);
    fixture.buffer.reset();
    dlx::appendLogMailbox(&fixture.channel, fixture, fixture.buffer);
    CHECK(fixture.buffer.used() == 4u);
    CHECK(fixture.lastException == static_cast<uint16_t>(dlx::ExceptionCode::BARO_DATA_STALE));
    dlx::appendLogMailbox(nullptr, fixture, fixture.buffer);
}

int main()
{
    testStartupAndHandoff();
    testBackpressure();
    testExceptions();
    printf("log_mailbox_test: %u checks, %u failures\n", checks, failures);
    return failures == 0u ? 0 : 1;
}
