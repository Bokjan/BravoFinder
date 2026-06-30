#pragma once

#include <string>
#include <type_traits>
#include <utility>
#include <variant>

namespace bf {

// Category of an error, used by callers to branch on failure kinds without
// parsing the human-readable message.
enum class ErrorCode {
  kUnknown,
  kInvalidArgument,
  kDataMissing,
  kParseError,
  kAirportNotFound,
  kNoRoute,
};

// A lightweight error value carried by Result on the failure path.
struct Error {
  ErrorCode code = ErrorCode::kUnknown;
  std::string message;

  Error() = default;
  Error(ErrorCode c, std::string msg) : code(c), message(std::move(msg)) {}
};

// Result<T, E> holds either a success value of type T or an error of type E.
//
// This is a deliberately small stand-in for std::expected (C++23): the project
// targets C++20 and avoids third-party dependencies. The API names mirror
// std::expected (has_value/value/error) so a future migration stays mechanical.
//
// Construct via the Ok/Err factories. The success and error types must differ
// so the active alternative is unambiguous.
template <class T, class E = Error>
class Result {
  static_assert(!std::is_same_v<T, E>, "Result<T, E> requires distinct value and error types");

 public:
  static Result Ok(T value) { return Result(std::in_place_index<0>, std::move(value)); }
  static Result Err(E error) { return Result(std::in_place_index<1>, std::move(error)); }

  // True when the result holds a success value.
  bool has_value() const noexcept { return data_.index() == 0; }
  explicit operator bool() const noexcept { return has_value(); }

  // Access the success value. Precondition: has_value() is true.
  const T& value() const& { return std::get<0>(data_); }
  T& value() & { return std::get<0>(data_); }
  T&& value() && { return std::get<0>(std::move(data_)); }

  // Access the error. Precondition: has_value() is false.
  const E& error() const& { return std::get<1>(data_); }
  E& error() & { return std::get<1>(data_); }

  // Return the success value if present, otherwise the supplied fallback.
  T value_or(T fallback) const& { return has_value() ? std::get<0>(data_) : std::move(fallback); }

 private:
  template <std::size_t I, class U>
  Result(std::in_place_index_t<I> tag, U&& v) : data_(tag, std::forward<U>(v)) {}

  std::variant<T, E> data_;
};

}  // namespace bf
