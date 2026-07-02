// Thin wrapper around rapidjson/document.h that locally suppresses benign
// warnings triggered by rapidjson 1.1.0 internals under -Werror. The warnings
// fire from template instantiations inside the header (memcpy over nontrivial
// types, deprecated std::iterator), which SYSTEM includes cannot suppress.
// Using #pragma push/pop keeps our own code in the including translation unit
// under full warning coverage — only the rapidjson header itself is silenced.
#pragma once

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#pragma clang diagnostic ignored "-Wnontrivial-memcall"
#endif

#ifdef __GNUC__
#if !defined(__clang__)  // GCC but not Clang (which also defines __GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wclass-memaccess"
#endif
#endif

#include "rapidjson/document.h"

#ifdef __clang__
#pragma clang diagnostic pop
#endif

#ifdef __GNUC__
#if !defined(__clang__)
#pragma GCC diagnostic pop
#endif
#endif
