#pragma once

#include <cstdlib>
#include <iostream>
#include <source_location>

#ifdef _WIN32
#include <windows.h>
#include <dbghelp.h>

namespace {

inline void printStackTrace() {
    void* frames[32];
    const USHORT count = CaptureStackBackTrace(1, 32, frames, nullptr);
    HANDLE process = GetCurrentProcess();
    SymSetOptions(SYMOPT_LOAD_LINES | SYMOPT_UNDNAME);
    if (!SymInitialize(process, nullptr, TRUE)) {
        for (USHORT i = 0; i < count; ++i) std::cerr << "  " << frames[i] << '\n';
        return;
    }
    alignas(SYMBOL_INFO) unsigned char storage[sizeof(SYMBOL_INFO) + MAX_SYM_NAME]{};
    auto* symbol = reinterpret_cast<PSYMBOL_INFO>(storage);
    symbol->SizeOfStruct = sizeof(SYMBOL_INFO);
    symbol->MaxNameLen = MAX_SYM_NAME;
    for (USHORT i = 0; i < count; ++i) {
        DWORD64 displacement = 0;
        if (SymFromAddr(process, reinterpret_cast<DWORD64>(frames[i]),
                        &displacement, symbol))
            std::cerr << "  " << symbol->Name;
        else
            std::cerr << "  " << frames[i];
        IMAGEHLP_LINE64 line{};
        line.SizeOfStruct = sizeof(line);
        DWORD lineDisplacement = 0;
        if (SymGetLineFromAddr64(process, reinterpret_cast<DWORD64>(frames[i]),
                                 &lineDisplacement, &line))
            std::cerr << " (" << line.FileName << ':' << line.LineNumber << ')';
        std::cerr << '\n';
    }
    SymCleanup(process);
}

}  // namespace
#endif

namespace test {

inline void check(bool condition, const char* errmsg,
                  std::source_location location = std::source_location::current()) {
    if (condition) return;
    std::cerr << "[FAIL] " << errmsg << "\n"
              << "  at " << location.file_name() << ':' << location.line() << '\n'
              << "  function: " << location.function_name() << '\n'
              << "  stack:\n";
#ifdef _WIN32
    printStackTrace();
#endif
    std::abort();
}

}  // namespace test
