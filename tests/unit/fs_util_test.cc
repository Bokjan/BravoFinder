// SPDX-License-Identifier: MIT
#include "io/fs_util.h"

#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <vector>

namespace {

namespace fs = std::filesystem;

// A throwaway directory unique to a test tag, emptied so repeated runs start
// clean.
fs::path TempDir(const std::string& tag) {
  const fs::path dir = fs::temp_directory_path() / ("bravofinder_fs_util_" + tag);
  std::error_code ec;
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);
  return dir;
}

void Touch(const fs::path& p) { std::ofstream(p) << "x"; }

}  // namespace

TEST_CASE("ListFiles returns matching regular files in path order", "[unit][fs_util]") {
  const fs::path dir = TempDir("order");
  Touch(dir / "KLAX.dat");
  Touch(dir / "EGLL.dat");
  Touch(dir / "K1.dat");
  Touch(dir / "notes.txt");
  Touch(dir / "KJFK.dat.bak");
  fs::create_directory(dir / "SUB.dat");  // a directory, not a file

  bf::Result<std::vector<std::string>> got = bf::ListFiles(dir.string(), ".dat");
  REQUIRE(got);
  const std::vector<std::string> want = {(dir / "EGLL.dat").string(), (dir / "K1.dat").string(),
                                         (dir / "KLAX.dat").string()};
  CHECK(got.value() == want);
  fs::remove_all(dir);
}

TEST_CASE("ListFiles skips an entry whose type cannot be read", "[unit][fs_util]") {
  // A symlink to itself: stat fails with ELOOP. directory_entry's throwing
  // is_regular_file() would throw here and take the CLI down with it.
  const fs::path dir = TempDir("loop");
  Touch(dir / "good.s3db");
  std::error_code ec;
  fs::create_symlink(dir / "loop.s3db", dir / "loop.s3db", ec);
  if (ec) {
    fs::remove_all(dir);
    SKIP("cannot create a symlink here (" + ec.message() + ")");
  }

  bf::Result<std::vector<std::string>> got = bf::ListFiles(dir.string(), ".s3db");
  REQUIRE(got);
  CHECK(got.value() == std::vector<std::string>{(dir / "good.s3db").string()});
  fs::remove_all(dir);
}

TEST_CASE("ListFiles reports a missing directory as an error", "[unit][fs_util]") {
  CHECK_FALSE(
      bf::ListFiles((fs::temp_directory_path() / "bravofinder_fs_util_absent").string(), ".dat"));
}
