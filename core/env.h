#pragma once

#include <cstdlib>

namespace bf {

// Thin wrapper around std::getenv. MSVC's /W3 warns C4996 on getenv (the CRT
// "secure" alternatives like _dupenv_s are not portable). Suppress that single
// warning locally so callers stay clean without a global _CRT_SECURE_NO_WARNINGS.
inline const char* GetEnv(const char* name) {
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
  return std::getenv(name);
#ifdef _MSC_VER
#pragma warning(pop)
#endif
}

}  // namespace bf
