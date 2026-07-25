// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/loaders/sqlite_util.h"

#include <string>
#include <unordered_map>
#include <utility>

namespace bf {

void SqliteDeleter::operator()(sqlite3* db) const noexcept {
  if (db != nullptr) {
    sqlite3_close(db);
  }
}

void StmtDeleter::operator()(sqlite3_stmt* stmt) const noexcept {
  if (stmt != nullptr) {
    sqlite3_finalize(stmt);
  }
}

namespace {

// Per-thread connection cache keyed by "loader|db_path". Each thread keeps its
// own connections (never shared across threads -> SQLITE_OPEN_NOMUTEX is safe),
// and a connection to a given DB is reused across that thread's LoadProcedure /
// LoadNavData / LoadProcedures calls. Connections live until thread exit.
thread_local std::unordered_map<std::string, SqliteHandle> tls_conns;

}  // namespace

Result<sqlite3*> AcquireConn(std::string_view loader_name, const std::string& db_path) {
  const std::string key = std::string(loader_name) + "|" + db_path;
  if (auto it = tls_conns.find(key); it != tls_conns.end()) {
    return Result<sqlite3*>::Ok(it->second.get());
  }
  sqlite3* raw = nullptr;
  const int rc =
      sqlite3_open_v2(db_path.c_str(), &raw, SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX, nullptr);
  if (rc != SQLITE_OK) {
    const std::string msg = raw != nullptr ? sqlite3_errmsg(raw) : "cannot open database";
    if (raw != nullptr) {
      sqlite3_close(raw);
    }
    return Result<sqlite3*>::Err(Error(ErrorCode::kDataMissing, "SQLite: " + msg));
  }
  SqliteHandle handle(raw);
  sqlite3* ptr = handle.get();
  tls_conns.emplace(key, std::move(handle));
  return Result<sqlite3*>::Ok(ptr);
}

Result<SqliteStmt> Prepare(sqlite3* conn, std::string_view sql) {
  sqlite3_stmt* stmt = nullptr;
  const int rc = sqlite3_prepare_v2(conn, sql.data(), static_cast<int>(sql.size()), &stmt, nullptr);
  if (rc != SQLITE_OK) {
    return Result<SqliteStmt>::Err(Error(
        ErrorCode::kParseError, std::string("SQLite prepare failed: ") + sqlite3_errmsg(conn)));
  }
  return Result<SqliteStmt>::Ok(SqliteStmt(stmt));
}

Result<bool> Step(sqlite3_stmt* stmt) {
  const int rc = sqlite3_step(stmt);
  if (rc == SQLITE_ROW) {
    return Result<bool>::Ok(true);
  }
  if (rc == SQLITE_DONE) {
    return Result<bool>::Ok(false);
  }
  return Result<bool>::Err(Error(ErrorCode::kParseError, "SQLite step failed"));
}

std::string ColumnText(sqlite3_stmt* stmt, int col) {
  const unsigned char* t = sqlite3_column_text(stmt, col);
  if (t == nullptr) {
    return {};
  }
  std::string s(reinterpret_cast<const char*>(t));
  // DFD pads fixed-width TEXT columns with spaces; trim them.
  const auto b = s.find_first_not_of(' ');
  if (b == std::string::npos) {
    return {};
  }
  const auto e = s.find_last_not_of(' ');
  return s.substr(b, e - b + 1);
}

std::string ColumnTextRaw(sqlite3_stmt* stmt, int col) {
  const unsigned char* t = sqlite3_column_text(stmt, col);
  if (t == nullptr) {
    return {};
  }
  // No trimming: preserve leading/trailing spaces so fixed ARINC column offsets
  // stay aligned (see the header). Use the byte count so the full padded field
  // is returned verbatim.
  const int n = sqlite3_column_bytes(stmt, col);
  return std::string(reinterpret_cast<const char*>(t), static_cast<size_t>(n < 0 ? 0 : n));
}

int ColumnInt(sqlite3_stmt* stmt, int col) {
  if (sqlite3_column_type(stmt, col) == SQLITE_NULL) {
    return 0;
  }
  return sqlite3_column_int(stmt, col);
}

double ColumnDouble(sqlite3_stmt* stmt, int col) {
  if (sqlite3_column_type(stmt, col) == SQLITE_NULL) {
    return 0.0;
  }
  return sqlite3_column_double(stmt, col);
}

}  // namespace bf
