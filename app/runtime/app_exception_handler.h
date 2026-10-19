#pragma once
#include <stdint.h>
#include "dlx_exception_recorder.hpp"
#include "dlx_delay.hpp"
#include "dlx.hpp"
#include "app/runtime/release_header.h"

void ENABLE_EXCEPTION_TEXT();

void DISABLE_EXCEPTION_TEXT();

bool IS_EXCEPTION_TEXT();

void TEXT_EXCEPTION_HANDLER(ExceptionCode code);

void ENABLE_EXCEPTION_DLX();

void DISABLE_EXCEPTION_DLX();

bool IS_EXCEPTION_DLX();

void DLX_EXCEPTION_HANDLER(ExceptionCode code);

void DLX_HANDLE_EXCEPTION(ExceptionCode code);

void DLX_HANDLE_EXCEPTION(ExceptionRecorder &rec);
using ExceptionHandler = void (*)(ExceptionCode);
extern ExceptionHandler EXCEPTION_HANDLE;
