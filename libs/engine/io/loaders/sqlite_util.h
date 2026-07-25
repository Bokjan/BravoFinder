// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

// Generic SQLite plumbing shared by the DFD loaders (DFD v1.0 and DFD v2):
// RAII handles for connections and prepared statements, a per-thread
// connection cache (contract-B: a connection is never shared across threads),
// and null/blank-safe column accessors. The DFD loaders add only their own
// row-to-domain mapping on top of this; nothing here is DFD-version-specific.

#include <sqlite3.h>

#include <memory>
#include <string>
#include <string_view>

#include "core/result.h"

namespace bf {

// RAII handle for a sqlite3 connection (sqlite3_close on destruction).
struct SqliteDeleter {
  void operator()(sqlite3* db) const noexcept;
};
using SqliteHandle = std::unique_ptr<sqlite3, SqliteDeleter>;

// RAII handle for a prepared statement (sqlite3_finalize on destruction).
struct StmtDeleter {
  void operator()(sqlite3_stmt* stmt) const noexcept;
};
using SqliteStmt = std::unique_ptr<sqlite3_stmt, StmtDeleter>;

// Per-thread, per-(loader,db_path) connection cache. Opens read-only with
// SQLITE_OPEN_NOMUTEX (multi-thread mode): the connection lives in thread_local
// storage so it is never shared across threads, hence needs no per-connection
// mutex. Returns the borrowed live connection, opening it on first use; an Error
// (kDataMissing) if the database cannot be opened.
Result<sqlite3*> AcquireConn(std::string_view loader_name, const std::string& db_path);

// Prepare a SQL statement. Returns an Error (kParseError) on SQL failure.
Result<SqliteStmt> Prepare(sqlite3* conn, std::string_view sql);

// Step a statement once. true while a row is available, false when exhausted
// (SQLITE_DONE); any other SQLite result is an Error (kParseError).
Result<bool> Step(sqlite3_stmt* stmt);

// Repeatedly step `stmt`, invoking `on_row` for each row, until SQLITE_DONE.
// A step error (corruption, IO, schema mismatch) propagates as kParseError
// instead of being silently swallowed as "no more rows". This replaces the
// `while (Step(stmt).value_or(false))` pattern, which collapsed errors into
// normal row exhaustion and returned partial data as Ok -- violating the
// "parse error -> wrong route" invariant when a database page is corrupt.
template <class F>
Result<void> ForEachRow(sqlite3_stmt* stmt, F&& on_row) {
  for (;;) {
    Result<bool> r = Step(stmt);
    if (!r) {
      return Result<void>::Err(r.error());
    }
    if (!r.value()) {
      return Result<void>::Ok();
    }
    on_row();
  }
}

// Column accessors: null/blank-safe. Text is space-trimmed (DFD pads fixed-width
// text columns); Int/Double treat null as 0.
std::string ColumnText(sqlite3_stmt* stmt, int col);
int ColumnInt(sqlite3_stmt* stmt, int col);
double ColumnDouble(sqlite3_stmt* stmt, int col);

// Like ColumnText but WITHOUT trimming, so a caller indexing by fixed ARINC
// column position sees the original byte offsets. The waypoint description code
// (column 2 == 'E' marks End of Airway) must be read this way: trimming leading
// spaces would shift the columns left whenever column 1 is blank and silently
// break the End-of-Airway detection. Null -> empty string.
std::string ColumnTextRaw(sqlite3_stmt* stmt, int col);

}  // namespace bf
