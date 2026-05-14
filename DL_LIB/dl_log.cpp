
#include "stm32f4xx.h"

#include "functional"
#include "dl_log.h"
std::function<void(const char *msg)> log_func = nullptr;

Logger::Logger(char *buffer)
{
    buf    = buffer;
    buf[0] = '\0';
}
void Logger::flush()
{
    log_func(buf);
    buf[0] = '\0';
}
Logger &Logger::operator<<(LCMD cmd)
{
    if (cmd==LCMD::NFLUSH)
    {
        strcat(buf, "\n");
    }
    
    flush();
    return *this;
}
Logger &Logger::operator<<(const char *msg)
{
    if (strlen(buf) + strlen(msg) >= DL_LOG_BUFFER_MAX_SIZE) {
        flush();
    }

    strcat(buf, msg);
    return *this;
}
Logger &Logger::operator<<(uint32_t num)
{
    if (strlen(buf) + 10 >= DL_LOG_BUFFER_MAX_SIZE) {
        flush();
    }

    char num_buf[10];
    sprintf(num_buf, "%lu", num);
    strcat(buf, num_buf);
    return *this;
}
Logger &Logger::operator<<(int32_t num)
{
    if (strlen(buf) + 12 >= DL_LOG_BUFFER_MAX_SIZE) {
        flush();
    }
    char num_buf[12];
    sprintf(num_buf, "%ld", num);
    strcat(buf, num_buf);
    return *this;
}
Logger &Logger::operator<<(double num)
{
    if (strlen(buf) + 32 >= DL_LOG_BUFFER_MAX_SIZE) {
        flush();
    }
    char num_buf[32];
    sprintf(num_buf, "%f", num);
    strcat(buf, num_buf);
    return *this;
}
Logger &Logger::operator<<(bool b)
{
    if (strlen(buf) + 5 >= DL_LOG_BUFFER_MAX_SIZE) {
        flush();
    }
    if (b){
        strcat(buf, "true");
    }else{
        strcat(buf, "false");
    }
    return *this;
}

Logger &Logger::operator[](const char *msg){
    if (strlen(buf) + strlen(msg) >= DL_LOG_BUFFER_MAX_SIZE)
    {
        flush();
    }
    strcat(buf, msg);
    strcat(buf,"\n");
    flush();
    return *this;
}


Logger logger(new char[DL_LOG_BUFFER_MAX_SIZE]);

#ifdef DL_DEBUG_ENABLE
// Logger debugLogger(new char[512]);

#define DEBUG_FUNC(x) do{x}while(0)
#else
#define DEBUG_FUNC(x)
#endif