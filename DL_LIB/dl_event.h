
#pragma once 

#include "dl_log.h"

#include <functional>

namespace dl{
    
    namespace event
    {
        enum class ExceptionCode : uint16_t
        {
            NONE = 0
        };
        extern std::function<void(uint16_t code)> eventCallbacks[];
        extern uint16_t*  eventQueue;
        extern uint16_t* eventQueueHead;
        extern uint8_t eventQueueSize;

        bool initEventModule(uint16_t* buffer,uint8_t size);

        void addEvent(uint16_t code);

        bool existEvent(uint16_t code);

        void processEvents();//未定义

        void clearEvents();

    } // namespace event
    
} // namespace dl
