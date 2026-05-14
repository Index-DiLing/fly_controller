#include "dl_event.h"

uint16_t* dl::event::eventQueue = nullptr;

uint16_t* dl::event::eventQueueHead = nullptr;

uint8_t dl::event::eventQueueSize = 0;


bool dl::event::initEventModule(uint16_t* buffer, uint8_t size)
{
    if (size == 0 || buffer == nullptr)
    {
        return false;
    }
    eventQueue = buffer;
    eventQueueHead = buffer;
    eventQueueSize = size;
    return true;
}

void dl::event::addEvent(uint16_t code)
{
    if (eventQueue == nullptr || eventQueueSize == 0) {
        return;
    }
    *eventQueueHead++ = code;
}

bool dl::event::existEvent(uint16_t code)
{
    if (eventQueue == nullptr || eventQueueSize == 0) {
        return false;
    }
    uint16_t* ptr = eventQueue;
    while (ptr!= eventQueueHead)
    {
        if (*ptr == code) {
            return true;
        }
        ptr++;
    }
    return false;
}

void dl::event::clearEvents()
{
    if (eventQueue == nullptr || eventQueueSize == 0) {
        return;
    }
    eventQueueHead = eventQueue;
}
