// Thin wrapper around rapidjson/document.h that locally suppresses benign
// warnings triggered by rapidjson 1.1.0 internals under -Werror. The warnings
// fire from template instantiations inside the header (memcpy over nontrivial
// types, deprecated std::iterator), which SYSTEM includes cannot suppress.
// Using #pragma push/pop keeps our own code in the including translation unit
// under full warning coverage — only the rapidjson header itself is silenced.
#pragma once

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)  // std::iterator deprecated in C++17
#endif

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#if defined(__has_warning)
#if __has_warning("-Wnontrivial-memcall")
#pragma clang diagnostic ignored "-Wnontrivial-memcall"
#endif
#endif
#endif

#ifdef __GNUC__
#if !defined(__clang__)  // GCC but not Clang (which also defines __GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wclass-memaccess"
#endif
#endif

#include "rapidjson/document.h"

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#ifdef __clang__
#pragma clang diagnostic pop
#endif

#ifdef __GNUC__
#if !defined(__clang__)
#pragma GCC diagnostic pop
#endif
#endif
