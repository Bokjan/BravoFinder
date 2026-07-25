// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <cstdlib>
#include <string>

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

// Thin wrapper around setenv (POSIX) / _putenv_s (MSVC). Sets `name` to `value`
// for the current process and any children it spawns afterwards. Used by tests
// to switch the navigation-data source at runtime. Returns true on success.
inline bool SetEnv(const char* name, const std::string& value) {
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
  const int rc =
#ifdef _MSC_VER
      _putenv_s(name, value.c_str());
#else
      setenv(name, value.c_str(), /*overwrite=*/1);
#endif
#ifdef _MSC_VER
#pragma warning(pop)
#endif
  return rc == 0;
}

}  // namespace bf
