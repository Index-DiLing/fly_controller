#pragma once

#include "stm32f4xx.h"

#include "functional"

#define DL_LOG_BUFFER_MAX_SIZE  256

extern std::function<void(const char *msg)> log_func;
enum class LCMD {
    FLUSH,
    NFLUSH
};
class Logger
{
private:
    char *buf;
public:
    Logger(char* buffer);
    void flush();
    Logger &operator<<(LCMD cmd);
    Logger &operator<<(const char *msg);
    Logger &operator<<(uint32_t num);
    Logger &operator<<(int32_t num);
    Logger &operator<<(double num);
    Logger &operator<<(bool b);
    Logger &operator[](const char *msg);
};

extern Logger logger;




#ifdef DL_DEBUG_ENABLE
//Logger debugLogger(new char[512]);

#define DEBUG_FUNC(x) (x)
#else
#define DEBUG_FUNC(x)
#endif