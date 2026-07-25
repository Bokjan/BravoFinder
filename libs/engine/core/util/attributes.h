// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

// Portable attribute macros.

// BF_LIFETIMEBOUND: on a member function returning a view/pointer/reference that
// aliases *this (e.g. a std::string_view into the object's own storage), marks
// the return value's lifetime as bound to the receiver. clang then diagnoses
// `auto v = Temporary().View();` (the view dangles once the temporary dies) under
// -Wdangling. No effect on GCC (which lacks the attribute); ASan
// use-after-scope in the debug preset is the runtime backstop there.
#if defined(__clang__)
#define BF_LIFETIMEBOUND [[clang::lifetimebound]]
#elif defined(_MSC_VER)
#define BF_LIFETIMEBOUND [[msvc::lifetimebound]]
#else
#define BF_LIFETIMEBOUND
#endif
