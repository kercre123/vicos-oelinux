/*
* Copyright (c) 2016, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following
*       disclaimer in the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of The Linux Foundation nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
* ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <dlfcn.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <cutils/properties.h>
#include <mutex>
#include <chrono>
#include <sstream>
#include <fstream>
#include <inc/qmmf_database.h>

namespace qmmf {

namespace database {

typedef int   (*sqlite3_callback_t)(void *, int, char **, char **);
typedef int   (*sqlite3_open_t)(char *db_name, sqlite3 **handle);
typedef int   (*sqlite3_close_t)(sqlite3 *handle);
typedef char *(*sqlite3_errmsg_t)(sqlite3 *handle);
typedef void *(*sqlite3_malloc_t)(int bytes);
typedef void  (*sqlite3_free_t)(void *ptr);
typedef int   (*sqlite3_prepare_t)(sqlite3 *, const char *, int, sqlite3_stmt **, const char **);
typedef int   (*sqlite3_step_t)(sqlite3_stmt*);
typedef int   (*sqlite3_finalize_t)(sqlite3_stmt *);
typedef int   (*sqlite3_bind_blob_t)(sqlite3_stmt*, int, const void*, int n, void(*)(void*));
typedef int   (*sqlite3_bind_double_t)(sqlite3_stmt*, int, double);
typedef int   (*sqlite3_bind_int_t)(sqlite3_stmt*, int, int);
typedef int   (*sqlite3_bind_int64_t)(sqlite3_stmt*, int, sqlite3_int64);
typedef int   (*sqlite3_bind_text_t)(sqlite3_stmt*, int, const char*, int, void(*)(void*));
typedef int   (*sqlite3_column_bytes_t)(sqlite3_stmt*, int);
typedef int   (*sqlite3_column_int_t)(sqlite3_stmt*, int);
typedef sqlite3_int64 (*sqlite3_column_int64_t)(sqlite3_stmt*, int);
typedef double        (*sqlite3_column_double_t)(sqlite3_stmt*, int);
typedef const unsigned char *(*sqlite3_column_text_t)(sqlite3_stmt*, int);
typedef const void *  (*sqlite3_column_blob_t)(sqlite3_stmt*, int);
typedef int           (*sqlite3_table_column_metadata_t)(sqlite3 *db,
                                                 const char *db_name,
                                                 const char *table_name,
                                                 const char *column_name,
                                                 char const **data_type,
                                                 char const **collation_seq,
                                                 int *not_null,
                                                 int *primary_key,
                                                 int *autoincrement);

typedef struct {
  sqlite3_open_t open;
  sqlite3_table_column_metadata_t tc_metadata;
  sqlite3_close_t close;
  sqlite3_errmsg_t errmsg;
  sqlite3_malloc_t malloc;
  sqlite3_free_t free;
  sqlite3_prepare_t prepare;
  sqlite3_step_t step;
  sqlite3_finalize_t finalize;
  sqlite3_bind_int_t bind_int;
  sqlite3_bind_int64_t bind_int64;
  sqlite3_bind_double_t bind_double;
  sqlite3_bind_text_t bind_text;
  sqlite3_bind_blob_t bind_blob;
  sqlite3_column_bytes_t column_bytes;
  sqlite3_column_int_t column_int;
  sqlite3_column_int64_t column_int64;
  sqlite3_column_double_t column_double;
  sqlite3_column_text_t column_text;
  sqlite3_column_blob_t column_blob;
} sqlite3_db_api;

typedef struct {
  sqlite3 *db_;
  sqlite3_db_api *api_;
} sqlite3_db_info;

#define DB_ACCESS_INFO(x) sqlite3_db_info *(x) = (sqlite3_db_info *)dbi_
#define DB_ACCESS_API(x) sqlite3_db_api *(x) = ((sqlite3_db_info *)dbi_)->api_

static int isLoggingLevel = 0;
#define VAM_DB_LOGE(...) if (isLoggingLevel > 0) ALOGE("VAMDB: " __VA_ARGS__)
#define VAM_DB_LOGD(...) if (isLoggingLevel > 1) ALOGD("VAMDB: " __VA_ARGS__)
#define VAM_DB_LOGI(...) if (isLoggingLevel > 2) ALOGI("VAMDB: " __VA_ARGS__)


class SQLiteQueryCleanup {
private:
  SQLiteQuery *query_;
  SQLiteDB *db_;
public:
  SQLiteQueryCleanup(SQLiteQuery *q, SQLiteDB *db) : query_(q), db_(db) {}
  ~SQLiteQueryCleanup() {
    if (db_ && query_) {
      db_->FinalizeQuery(query_);
    }
    delete query_;
  }
};

bool loadSQLiteAPI(sqlite3_db_info *dbi) {

  {
    char property[PROPERTY_VALUE_MAX];
    if (property_get(VAM_DATABASE_LOGLEVEL_PROP, property, NULL) > 0) {
        isLoggingLevel = atoi(property);
    } else {
        isLoggingLevel = 0;
    }
  }

  sqlite3_db_api *sqlite3_api = new sqlite3_db_api;
  void *libptr;
  if (!sqlite3_api) {
    ALOGE("%s: not enough memory", __func__);
    goto sqlite_init_error;
  }

  libptr = dlopen("libsqlite3.so.0", RTLD_NOW);
  if (!libptr) {
    VAM_DB_LOGE("%s: can't load SQLite3 library", __func__);
    goto sqlite_init_error;
  }

  sqlite3_api->open           = (sqlite3_open_t)dlsym(libptr, "sqlite3_open");
  sqlite3_api->close          = (sqlite3_close_t)dlsym(libptr, "sqlite3_close");
  sqlite3_api->errmsg         = (sqlite3_errmsg_t)dlsym(libptr, "sqlite3_errmsg");
  sqlite3_api->malloc         = (sqlite3_malloc_t)dlsym(libptr, "sqlite3_malloc");
  sqlite3_api->free           = (sqlite3_free_t)dlsym(libptr, "sqlite3_free");
  sqlite3_api->tc_metadata    = (sqlite3_table_column_metadata_t)dlsym(libptr, "sqlite3_table_column_metadata");
  sqlite3_api->prepare        = (sqlite3_prepare_t)dlsym(libptr,  "sqlite3_prepare");
  sqlite3_api->step           = (sqlite3_step_t)dlsym(libptr, "sqlite3_step");
  sqlite3_api->finalize       = (sqlite3_finalize_t)dlsym(libptr, "sqlite3_finalize");
  sqlite3_api->bind_int       = (sqlite3_bind_int_t)dlsym(libptr, "sqlite3_bind_int");
  sqlite3_api->bind_int64     = (sqlite3_bind_int64_t)dlsym(libptr, "sqlite3_bind_int64");
  sqlite3_api->bind_double    = (sqlite3_bind_double_t)dlsym(libptr, "sqlite3_bind_double");
  sqlite3_api->bind_text      = (sqlite3_bind_text_t)dlsym(libptr, "sqlite3_bind_text");
  sqlite3_api->bind_blob      = (sqlite3_bind_blob_t)dlsym(libptr, "sqlite3_bind_blob");
  sqlite3_api->column_bytes   = (sqlite3_column_bytes_t)dlsym(libptr, "sqlite3_column_bytes");
  sqlite3_api->column_int     = (sqlite3_column_int_t)dlsym(libptr, "sqlite3_column_int");
  sqlite3_api->column_int64   = (sqlite3_column_int64_t)dlsym(libptr, "sqlite3_column_int64");
  sqlite3_api->column_double  = (sqlite3_column_double_t)dlsym(libptr, "sqlite3_column_double");
  sqlite3_api->column_text    = (sqlite3_column_text_t)dlsym(libptr, "sqlite3_column_text");
  sqlite3_api->column_blob    = (sqlite3_column_blob_t)dlsym(libptr, "sqlite3_column_blob");

  if (!sqlite3_api->open          ||
      !sqlite3_api->close         ||
      !sqlite3_api->errmsg        ||
      !sqlite3_api->malloc        ||
      !sqlite3_api->free          ||
      !sqlite3_api->tc_metadata   ||
      !sqlite3_api->prepare       ||
      !sqlite3_api->step          ||
      !sqlite3_api->finalize      ||
      !sqlite3_api->bind_int      ||
      !sqlite3_api->bind_int64    ||
      !sqlite3_api->bind_double   ||
      !sqlite3_api->bind_text     ||
      !sqlite3_api->bind_blob     ||
      !sqlite3_api->column_bytes  ||
      !sqlite3_api->column_int    ||
      !sqlite3_api->column_int64  ||
      !sqlite3_api->column_double ||
      !sqlite3_api->column_text   ||
      !sqlite3_api->column_blob) {
    VAM_DB_LOGE("%s: could not load one or more entry SQLite points", __func__);
    goto sqlite_init_error;
  }

  dbi->api_ = sqlite3_api;
  return true;

sqlite_init_error:
  delete sqlite3_api;
  dbi->api_ = 0;
  return false;
}

SQLiteDB::SQLiteDB() {
  dbi_ = new sqlite3_db_info;
  memset(dbi_, 0, sizeof(sqlite3_db_info));
  loadSQLiteAPI((sqlite3_db_info *)dbi_);
}

SQLiteDB::~SQLiteDB() {
  CloseDB();
}

bool SQLiteDB::checkIfExisting(const char *table_name, const char *column_name) {
  DB_ACCESS_INFO(dbi);
  int res = dbi->api_->tc_metadata(dbi->db_, 0, (char *)table_name,
          (char *)column_name, 0, 0, 0, 0, 0);
  return res == SQLITE_OK;
}

int32_t SQLiteDB::OpenDB(const char *db_name) {
  DB_ACCESS_INFO(dbi);

  if (!db_name || !strlen(db_name)) {
    VAM_DB_LOGE("%s: invalid database name argument", __func__);
    return SQLITE_ERROR;
  }

  if (dbi->db_) {
    dbi->api_->close(dbi->db_);
  }

  int32_t res = dbi->api_->open((char *)db_name, &dbi->db_);
  if (res != SQLITE_OK) {
    VAM_DB_LOGE("%s: opening of database '%s' failed: %s (%d)",
        __func__, db_name, dbi->api_->errmsg(dbi->db_), res);
  }

  return res;
}

int32_t SQLiteDB::CloseDB() {
  DB_ACCESS_INFO(dbi);
  if (dbi->db_) {
    dbi->api_->close(dbi->db_);
    dbi->db_ = 0;
  }
  return SQLITE_OK;
}

SQLiteQuery *SQLiteDB::PrepareQuery(const char *sql) {
  int res;
  sqlite3_stmt *stmt;
  DB_ACCESS_INFO(dbi);

  VAM_DB_LOGE("%s: query='%s'", __func__, sql);

  if (!dbi->db_) {
    VAM_DB_LOGE("%s: can't execute query without db connection", __func__);
    return NULL;
  }

  res = dbi->api_->prepare(dbi->db_, sql, -1, &stmt, 0);
  if (res != SQLITE_OK) {
    VAM_DB_LOGE("%s: could not prepare query.", __func__);
    return NULL;
  }

  SQLiteQuery *q = new SQLiteQuery;
  if (q) {
    q->sql = sql;
    q->stmt = stmt;
  } else {
    VAM_DB_LOGE("%s: not enough memory to prepare a sql query.", __func__);
  }

  return q;
}

int32_t SQLiteDB::GetNextResult(SQLiteQuery *query, vector<SQLiteDataEntry> *results) {
  int res = SQLITE_DONE;
  DB_ACCESS_INFO(dbi);

  if (!dbi->db_) {
    VAM_DB_LOGE("%s: can't execute query without db connection", __func__);
    return SQLITE_ERROR;
  }

  do {
    for (size_t i = 0; results && i < results->size(); i++) {
      SQLiteDataEntry &p = (*results)[i];
      if (!p.addr) {
          continue;
      }
      switch (p.type) {
        case SQLiteDataType::INTEGER64:
          dbi->api_->bind_int(query->stmt, i + 1, ((int64_t *)p.addr)[0]);
          break;
        case SQLiteDataType::INTEGER:
          dbi->api_->bind_int64(query->stmt, i + 1, ((int32_t *)p.addr)[0]);
          break;
        case SQLiteDataType::DOUBLE:
          dbi->api_->bind_double(query->stmt, i + 1, ((double *)p.addr)[0]);
          break;
        case SQLiteDataType::TEXT:
          dbi->api_->bind_text(query->stmt, i + 1, (const char *)p.addr, p.size, SQLITE_STATIC);
          break;
        case SQLiteDataType::BLOB:
          dbi->api_->bind_blob(query->stmt, i + 1, p.addr, p.size, SQLITE_STATIC);
          break;
        default:
          VAM_DB_LOGE("%s: unknown SQLite query parameter type: %d", __func__, p.type);
          break;
      }
    }

    res = dbi->api_->step(query->stmt);
    VAM_DB_LOGD("%s: execute query: '%s' status = %d", __func__, query->sql.c_str(), res);

    if (res == SQLITE_SCHEMA) {
      continue;
    }

    if (res != SQLITE_ROW) {
      break;
    }

    for (size_t i = 0; results && i < results->size(); i++) {
      SQLiteDataEntry &p = (*results)[i];
      if (p.addr) {
          continue;
      }
      switch (p.type) {
        case SQLiteDataType::INTEGER64: p.size = sizeof(uint64_t); break;
        case SQLiteDataType::INTEGER: p.size = sizeof(uint32_t); break;
        case SQLiteDataType::DOUBLE: p.size = sizeof(double); break;
        case SQLiteDataType::TEXT:
          p.size = dbi->api_->column_bytes(query->stmt, i) + 1;
          break;
        case SQLiteDataType::BLOB:
          p.size = dbi->api_->column_bytes(query->stmt, i);
          break;
        default:
          VAM_DB_LOGE("%s: unknown SQLite query parameter type: %d", __func__, p.type);
          break;
      }
      p.addr = new uint8_t[p.size];

      switch (p.type) {
        case SQLiteDataType::INTEGER64: {
          uint64_t v = dbi->api_->column_int64(query->stmt, i);
          memcpy(p.addr, &v, p.size);
          VAM_DB_LOGD("%s: query result: int64 val: %lld", __func__, v);
          break;
        }
        case SQLiteDataType::INTEGER: {
          uint32_t v = dbi->api_->column_int(query->stmt, i);
          memcpy(p.addr, &v, p.size);
          VAM_DB_LOGD("%s: query result: int32 val: %d", __func__, v);
          break;
        }
        case SQLiteDataType::DOUBLE: {
          double v = dbi->api_->column_double(query->stmt, i);
          memcpy(p.addr, &v, p.size);
          VAM_DB_LOGD("%s: query result: double val: %f", __func__, v);
          break;
        }
        case SQLiteDataType::TEXT: {
          memcpy(p.addr, dbi->api_->column_text(query->stmt, i), p.size - 1);
          ((char *)p.addr)[p.size - 1] = 0;
          VAM_DB_LOGD("%s: query result: string val: %s", __func__, (char *)p.addr);
          break;
        }
        case SQLiteDataType::BLOB:
          memcpy(p.addr, dbi->api_->column_blob(query->stmt, i), p.size);
          VAM_DB_LOGD("%s: query result: blob val: addr=%p datasize=%d", __func__, p.addr, p.size);
          break;
        default:
          VAM_DB_LOGE("%s: unknown SQLite query parameter type: %d", __func__, p.type);
          break;
      }
    }

    break;
  } while (1);

  return res;
}

int32_t SQLiteDB::FinalizeQuery(SQLiteQuery *query) {
  DB_ACCESS_INFO(dbi);

  if (!query) {
      VAM_DB_LOGE("%s: can't have NULL query pointer", __func__);
      return SQLITE_ERROR;
  }

  if (!dbi->db_) {
    VAM_DB_LOGE("%s: can't execute query without db connection", __func__);
    return SQLITE_ERROR;
  }

  int res = dbi->api_->finalize(query->stmt);
  query->stmt = 0;
  return res;
}

EventDB::EventDB() {
    current_event_page_size_ = VAM_EVENT_PAGE_SIZE;
}

EventDB::~EventDB() {
  db_.CloseDB();
}

int32_t EventDB::OpenDB(const char *session_id) {
  if (!session_id || !strlen(session_id)) {
    char timefmt[20];
    time_t t = time(NULL);
    struct tm *lt = localtime(&t);

    strftime(timefmt, sizeof(timefmt), "%4Y%2m%2d-%2k%2M%2S", lt);
    for (size_t i = 0; i < strlen(timefmt); i++) {
        if (timefmt[i] <= ' ') timefmt[i] = '0';
    }
    session_id_ = timefmt;
  } else {
    session_id_ = session_id;
  }

  mkdir(VAM_DIR, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  std::stringstream db_dir;
  db_dir << VAM_DIR << "/vamsession_" << session_id_;
  mkdir(db_dir.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  images_dir_ = db_dir.str() + "/images";
  mkdir(images_dir_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  videos_dir_ = db_dir.str() + "/videos";
  mkdir(videos_dir_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  events_dir_ = db_dir.str() + "/events";
  mkdir(events_dir_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  std::stringstream db_name;
  db_name << db_dir.str() << "/events.db";

  VAM_DB_LOGE("%s: session: %s", __func__, session_id_.c_str());

  int res = db_.OpenDB(db_name.str().c_str());
  if (res != SQLITE_OK) {
    VAM_DB_LOGE("%s: could not open database '%s'", __func__, db_name.str().c_str());
    return res;
  }

  if (!db_.checkIfExisting("Events", NULL)) {
    std::stringstream sql;
    sql << "CREATE TABLE Events( ";
    sql << "Id TEXT PRIMARY KEY NOT NULL, ";
    sql << "Type INT, ";
    sql << "Pts BIGINT, ";
    sql << "FrameId TEXT, ";
    sql << "RuleId TEXT, ";
    sql << "HasImage INT, ";
    sql << "HasVideo INT,";
    sql << "HasThumbnail INT);";
    SQLiteQuery *q = db_.PrepareQuery(sql.str().c_str());
    SQLiteQueryCleanup qc(q, &db_);
    if (!q) {
      VAM_DB_LOGE("%s: could not create query for event data table", __func__);
      return SQLITE_ERROR;
    }

    res = db_.GetNextResult(q, NULL);
    if (res != SQLITE_DONE && res != SQLITE_ROW) {
      VAM_DB_LOGE("%s: could not create event data table", __func__);
      return res;
    }
  }

  if (!db_.checkIfExisting("Rules", NULL)) {
    std::stringstream sql;
    sql << "CREATE TABLE Rules( ";
    sql << "Id TEXT PRIMARY KEY NOT NULL, ";
    sql << "Data BLOB);";
    SQLiteQuery *q = db_.PrepareQuery(sql.str().c_str());
    SQLiteQueryCleanup qc(q, &db_);
    if (!q) {
      VAM_DB_LOGE("%s: could not create query for rules data table", __func__);
      return SQLITE_ERROR;
    }

    res = db_.GetNextResult(q, NULL);
    if (res != SQLITE_DONE && res != SQLITE_ROW) {
      VAM_DB_LOGE("%s: could not create rules data table", __func__);
      return res;
    }
  }

  return SQLITE_OK;
}

int32_t EventDB::CloseDB() {
  return db_.CloseDB();
}

int32_t EventDB::GetCurrentSession(std::string *session) {
  if (!session) {
    return SQLITE_ERROR;
  }
  *session = session_id_;
  return SQLITE_OK;
}

int32_t EventDB::GetAllSessions(std::vector<std::string> *sessions) {
  if (!sessions) {
    return SQLITE_ERROR;
  }
  DIR *pDir = opendir(VAM_DIR);
  if (pDir != nullptr) {
    struct dirent *pDirent = nullptr;
    while ((pDirent = readdir(pDir)) != nullptr) {
      char *pos = strstr(pDirent->d_name, "vamsession_");
      if (pos) {
        pos += strlen("vamsession_");
        if (strlen(pos)) {
          sessions->push_back(pos);
        }
      }
    }
  }
  closedir(pDir);

  return SQLITE_OK;
}

bool EventDB::IsVAAPIIdValid(const char *vaapi_id) {
    if (!vaapi_id) return false;
    if (strlen(vaapi_id) != VAAPI_UUID_LEN - 1) return false;
    for (int j = 0; j < 8; j++) {
      for (int i = 0; i < 4; i++, vaapi_id++) {
        if (!isxdigit(*vaapi_id)) return false;
      }
      if (j > 0 && j < 5) {
        if(*vaapi_id != '-') return false;
        vaapi_id++;
      }
    }

    return true;
}

int32_t EventDB::WriteToFile(const char *file_name,
                             const uint8_t *data,
                             size_t size) {
  if (!file_name || !strlen(file_name) || !data || !size) {
    return SQLITE_ERROR;
  }

  std::ofstream fs(file_name, std::ofstream::binary | std::ofstream::app);
  fs.write((const char *)data, size);
  fs.close();

  return SQLITE_OK;
}

int32_t EventDB::WriteJpeg(const char *frame_id,
                           uint8_t *data,
                           size_t size) {
  std::stringstream path;
  path << images_dir_ << "/" << frame_id << ".jpg";

  return WriteToFile(path.str().c_str(), data, size);
}

int32_t EventDB::WriteVideo(const char *frame_id,
                            uint8_t *data,
                            size_t size) {
  std::stringstream path;
  path << videos_dir_ << "/" << frame_id << ".vid";

  return WriteToFile(path.str().c_str(), data, size);
}

int32_t EventDB::WriteMeta(vaapi_object *objects,
                           uint32_t cnt) {
  std::stringstream path;
  path << VAM_DIR << "/vamsession_" << session_id_ << "/metadata.raw";
  uint32_t size = sizeof(vaapi_object) * cnt;

  return WriteToFile(path.str().c_str(), (uint8_t *)objects, size);
}

int32_t EventDB::WriteEvents(const char *frame_id,
                             vaapi_event *events,
                             uint32_t cnt) {
  std::stringstream path;
  path << events_dir_ << "/" << frame_id << ".raw";
  uint32_t size = sizeof(vaapi_event) * cnt;

  return WriteToFile(path.str().c_str(), (uint8_t *)events, size);
}

int32_t EventDB::ReadFromFile(std::ifstream &fs,
                              uint8_t *data,
                              size_t *size) {
  if (!fs.is_open() || !data || !size || fs.eof()) {
    if (size) {
      *size = 0;
    }
    return SQLITE_ERROR;
  }

  fs.read((char *)data, *size);
  *size = fs.gcount();

  return SQLITE_OK;
}

int32_t EventDB::ReadEntireFile(std::ifstream &fs,
                                uint8_t **data,
                                size_t *size) {
  if (!fs.is_open() || !data || !size) {
    return SQLITE_ERROR;
  }

  fs.seekg(0, std::ifstream::beg);
  int32_t file_size = (int32_t)fs.tellg();
  fs.seekg(0, std::ifstream::end);
  file_size = (int32_t)fs.tellg() - file_size;
  fs.seekg(0, std::ifstream::beg);

  if (!file_size) {
    return SQLITE_ERROR;
  }

  uint8_t *ptr = new uint8_t[file_size];
  if (!ptr) {
    return SQLITE_ERROR;
  }

  int32_t bytesRead = 0;
  do {
    fs.read((char *)&ptr[bytesRead], file_size - bytesRead);
    bytesRead += fs.gcount();
  } while(!fs.eof() && bytesRead < file_size);

  *size = bytesRead;
  *data = ptr;

  return SQLITE_OK;
}

int32_t EventDB::ReadJpeg(const char *frame_id,
                          uint8_t **data,
                          size_t *size) {
  std::lock_guard<std::mutex> lg(delete_lock_);
  std::stringstream path;
  path << images_dir_ << "/" << frame_id << ".jpg";
  std::ifstream fs(path.str().c_str(), std::ifstream::binary);

  int32_t res = ReadEntireFile(fs, data, size);
  fs.close();

  return res;
}

int32_t EventDB::ReadVideo(const char *frame_id,
                           uint8_t **data,
                           size_t *size) {
  std::lock_guard<std::mutex> lg(delete_lock_);
  std::stringstream path;
  path << videos_dir_ << "/" << frame_id << ".vid";
  std::ifstream fs(path.str().c_str(), std::ifstream::binary);

  int32_t res = ReadEntireFile(fs, data, size);
  fs.close();

  return res;
}

int32_t EventDB::ReadMeta(uint32_t obj_id,
                          std::vector<vaapi_object> *objects) {
  std::lock_guard<std::mutex> lg(delete_lock_);
  std::stringstream path;
  path << VAM_DIR << "/vamsession_" << session_id_ << "/metadata.raw";
  std::ifstream fs(path.str().c_str(), std::ifstream::binary);

  if (!fs.is_open()) {
    VAM_DB_LOGE("%s: could not open metadata file '%s'",
            __func__, path.str().c_str());
    return SQLITE_ERROR;
  }

  size_t bytesRead;
  vaapi_object obj;
  int32_t res = SQLITE_ERROR;
  do {
    bytesRead = sizeof(vaapi_object);
    memset(&obj, 0, sizeof(vaapi_object));
    ReadFromFile(fs, (uint8_t *)&obj, &bytesRead);
    if (bytesRead == sizeof(vaapi_object) && obj.id == obj_id) {
      objects->push_back(obj);
      res = SQLITE_OK;
    }
  } while (!fs.eof());
  fs.close();

  return res;
}

int32_t EventDB::ReadEvent(const char *frame_id,
                           const char *event_id,
                           vaapi_event *event) {
  std::lock_guard<std::mutex> lg(delete_lock_);
  std::stringstream path;
  path << events_dir_ << "/" << frame_id << ".raw";
  std::ifstream fs(path.str().c_str(), std::ifstream::binary);

  if (!fs.is_open()) {
    VAM_DB_LOGE("%s: could not open event file '%s'",
            __func__, path.str().c_str());
    return SQLITE_ERROR;
  }

  size_t bytesRead;
  vaapi_event e;
  int32_t res = SQLITE_ERROR;
  do {
    bytesRead = sizeof(vaapi_event);
    memset(&e, 0, sizeof(vaapi_event));
    ReadFromFile(fs, (uint8_t *)&e, &bytesRead);
    if (bytesRead == sizeof(vaapi_event) && !strcmp(event_id, e.id)) {
      *event = e;
      res = SQLITE_OK;
      break;
    }
  } while (!fs.eof());
  fs.close();

  return res;
}

int32_t EventDB::DeleteJpeg(const char *frame_id) {
  std::lock_guard<std::mutex> lg(delete_lock_);
  std::stringstream path;
  path << images_dir_ << "/" << frame_id << ".jpg";
  remove(path.str().c_str());
  return SQLITE_OK;
}

int32_t EventDB::DeleteVideo(const char *frame_id) {
  std::lock_guard<std::mutex> lg(delete_lock_);
  std::stringstream path;
  path << videos_dir_ << "/" << frame_id << ".vid";
  remove(path.str().c_str());
  return SQLITE_OK;
}

int32_t EventDB::DeleteEvent(const char *frame_id, const char *event_id) {
  std::lock_guard<std::mutex> lg(delete_lock_);
  std::stringstream path;
  path << events_dir_ << "/" << frame_id << ".raw";

  std::ifstream fs(path.str().c_str(), std::ifstream::binary);

  size_t bytesRead;
  vaapi_event e;
  std::vector<vaapi_event> events;
  do {
    bytesRead = sizeof(vaapi_event);
    memset(&e, 0, sizeof(vaapi_event));
    ReadFromFile(fs, (uint8_t *)&e, &bytesRead);
    if (bytesRead == sizeof(vaapi_event) && strcmp(event_id, e.id)) {
      events.push_back(e);
    }
  } while (bytesRead == sizeof(vaapi_event));
  fs.close();

  remove(path.str().c_str());

  if (events.size()) {
    size_t size = sizeof(vaapi_event) * events.size();
    WriteToFile(path.str().c_str(), (uint8_t *)events.data(), size);
  }

  return SQLITE_OK;
}

int32_t EventDB::GetMultipleEvents(const char *sql_condition,
                                   std::vector<vaapi_event> *events) {
    vector<SQLiteDataEntry> dbRes;
    SQLiteDataEntry e;

    // event id
    e.type = SQLiteDataType::TEXT;
    e.addr = NULL;
    dbRes.push_back(e);

    // event's frame id
    dbRes.push_back(e);

    std::stringstream sql;
    sql << "SELECT Id, FrameId FROM Events ";
    if (sql_condition) {
      sql << sql_condition;
    }
    sql << ";";

    SQLiteQuery *q = db_.PrepareQuery(sql.str().c_str());
    SQLiteQueryCleanup qc(q, &db_);
    int32_t res;
    do {
      res = db_.GetNextResult(q, &dbRes);
      if (res == SQLITE_ROW) {
        vaapi_event ev;
        ReadEvent((char *)dbRes[1].addr, (char *)dbRes[0].addr, &ev);
        for (size_t j = 0; j < dbRes.size(); j++) {
          delete [] (uint8_t *)dbRes[j].addr;
          dbRes[j].addr = NULL;
        }

        events->push_back(ev);
      } else if (res != SQLITE_DONE) {
        return res;
      }
    } while (res == SQLITE_ROW);

    return SQLITE_OK;
}

int32_t EventDB::GetFrameIdForEvent(const char *event_id,
                                    std::string *frame_id) {
  std::vector<vaapi_event> events;
  std::stringstream sql;
  sql << "SELECT FrameId FROM Events WHERE Id='" << event_id << "';";

  vector<SQLiteDataEntry> dbRes;
  SQLiteDataEntry e;
  e.type = SQLiteDataType::TEXT;
  e.addr = NULL;
  dbRes.push_back(e);

  SQLiteQuery *q = db_.PrepareQuery(sql.str().c_str());
  SQLiteQueryCleanup qc(q, &db_);
  int32_t res = db_.GetNextResult(q, &dbRes);
  if (res != SQLITE_DONE && res != SQLITE_ROW && res != SQLITE_OK) {
    VAM_DB_LOGE("%s: could not find event %s.", __func__, event_id);
    return res;
  }
  if (!dbRes[0].addr) {
    VAM_DB_LOGE("%s: could not find or read event %s.", __func__, event_id);
    return SQLITE_ERROR;
  }

  *frame_id = (char *)dbRes[0].addr;
  delete [] (uint8_t *)dbRes[0].addr;

  return SQLITE_OK;
}

int32_t EventDB::GetEventsCountForFrame(const char *frame_id) {
  if (!frame_id || !strlen(frame_id)) {
    return -1;
  }

  std::stringstream sql;
  sql << "SELECT COUNT(*) FROM Events WHERE FrameId='" << frame_id << "';";

  vector<SQLiteDataEntry> dbRes;
  SQLiteDataEntry e;
  e.type = SQLiteDataType::INTEGER;
  e.addr = NULL;
  dbRes.push_back(e);

  SQLiteQuery *q = db_.PrepareQuery(sql.str().c_str());
  SQLiteQueryCleanup qc(q, &db_);
  int32_t res = db_.GetNextResult(q, &dbRes);
  if (res != SQLITE_DONE && res != SQLITE_ROW && res != SQLITE_OK) {
    return -1;
  }

  int32_t count = *((int32_t *)dbRes[0].addr);
  delete [] (uint8_t *)dbRes[0].addr;
  return count;
}

/* ############################
 * ###### Public methods ######
 * ############################
 */

int32_t EventDB::Insert(const vaapi_snapshot_info *snapshot) {
  if (!snapshot ||
      !snapshot->events || !snapshot->objects ||
      !snapshot->events_count || !snapshot->objects_count) {
    VAM_DB_LOGE("%s: invalid input arguments.", __func__);
    return SQLITE_ERROR;
  }

  int32_t res = SQLITE_OK;
  int32_t hasVideo = 0;
  int32_t hasImage = 0;
  int32_t hasThumbnail = 0;

  if (snapshot->img_data[0] && snapshot->data_size[0]) {
    WriteJpeg(snapshot->id, snapshot->img_data[0], snapshot->data_size[0]);
    hasImage = 1;
  }
  if (snapshot->img_data[1] && snapshot->data_size[1]) {
    WriteVideo(snapshot->id, snapshot->img_data[1], snapshot->data_size[1]);
    hasVideo = 1;
  }
  if (snapshot->img_data[2] && snapshot->data_size[2]) {
    std::stringstream tn;
    tn << snapshot->id << "-thumbnail";
    WriteJpeg(tn.str().c_str(), snapshot->img_data[2], snapshot->data_size[2]);
    hasThumbnail = 1;
  }

  WriteMeta(snapshot->objects, snapshot->objects_count);
  WriteEvents(snapshot->id, snapshot->events, snapshot->events_count);

  for (uint32_t i = 0; i < snapshot->events_count; i++) {
    std::stringstream sql;

    sql << "INSERT INTO Events(Id, Type, Pts, FrameId, RuleId, ";
    sql << "HasImage, HasVideo, HasThumbnail) ";
    sql << "VALUES (";
    sql << "'" << snapshot->events[i].id << "', ";
    sql <<        snapshot->events[i].type << ", ";
    sql <<        snapshot->events[i].pts << ", ";
    sql << "'" << snapshot->id << "', ";
    sql << "'" << snapshot->events[i].rule_id << "', ";
    sql <<        hasImage << ", ";
    sql <<        hasVideo << ", ";
    sql <<        hasThumbnail << ");";

    vector<SQLiteDataEntry> arguments;
    SQLiteQuery *q = db_.PrepareQuery(sql.str().c_str());
    SQLiteQueryCleanup qc(q, &db_);
    res = db_.GetNextResult(q, &arguments);

    if (res != SQLITE_DONE && res != SQLITE_ROW && res != SQLITE_OK) {
      VAM_DB_LOGE("%s: could not insert event to database.", __func__);
      return res;
    }
  }

  return SQLITE_OK;
}

int32_t EventDB::InsertMeta(const vaapi_metadata_frame *meta) {
  if (!meta || !meta->objects || !meta->object_num) {
    VAM_DB_LOGE("%s: invalid input arguments.", __func__);
    return SQLITE_ERROR;
  }

  return WriteMeta(meta->objects, meta->object_num);
}

int32_t EventDB::InsertRule(const vaapi_rule *rule) {
  if (!rule || !IsVAAPIIdValid(rule->id)) {
    VAM_DB_LOGE("%s: invalid input arguments.", __func__);
    return SQLITE_ERROR;
  }

  std::stringstream sql;

  sql << "INSERT INTO Rules(Id, Data) ";
  sql << "VALUES ('" << rule->id << "', ?)";

  vector<SQLiteDataEntry> arguments;
  SQLiteDataEntry e;
  e.type = SQLiteDataType::BLOB;
  e.addr = (void *)rule;
  e.size = sizeof(*rule);
  arguments.push_back(e);

  SQLiteQuery *q = db_.PrepareQuery(sql.str().c_str());
  SQLiteQueryCleanup qc(q, &db_);
  int32_t res = db_.GetNextResult(q, &arguments);

  if (res != SQLITE_DONE && res != SQLITE_ROW && res != SQLITE_OK) {
    VAM_DB_LOGE("%s: could not insert rule to database.", __func__);
    return res;
  }

  return res;
}

int32_t EventDB::RemoveEvent(const char *event_id) {
  if (!IsVAAPIIdValid(event_id)) {
    VAM_DB_LOGE("%s: invalid input arguments.", __func__);
    return SQLITE_ERROR;
  }

  std::string frameId;
  GetFrameIdForEvent(event_id, &frameId);

  {
    std::stringstream sql;
    sql << "DELETE FROM Events WHERE Id='" << event_id << "';";

    SQLiteQuery *q = db_.PrepareQuery(sql.str().c_str());
    SQLiteQueryCleanup qc(q, &db_);
    int32_t res = db_.GetNextResult(q, NULL);
    if (res != SQLITE_DONE && res != SQLITE_ROW && res != SQLITE_OK) {
      VAM_DB_LOGE("%s: could not remove event %s.", __func__, event_id);
      return res;
    }
  }

  bool clearSnapshotData = (GetEventsCountForFrame(frameId.c_str()) == 0);
  if (clearSnapshotData) {
    VAM_DB_LOGE("%s: no more events for frame %s.", __func__, frameId.c_str());
    DeleteJpeg(frameId.c_str());
    DeleteVideo(frameId.c_str());
  }
  DeleteEvent(frameId.c_str(), event_id);

  return SQLITE_OK;
}

int32_t EventDB::RemoveRule(const char *rule_id) {
  if (!IsVAAPIIdValid(rule_id)) {
    VAM_DB_LOGE("%s: invalid input arguments.", __func__);
    return SQLITE_ERROR;
  }

  stringstream sql;
  sql << "DELETE FROM Rules WHERE Id='" << rule_id << "';";

  SQLiteQuery *q = db_.PrepareQuery(sql.str().c_str());
  SQLiteQueryCleanup qc(q, &db_);
  int32_t res = db_.GetNextResult(q, NULL);
  if (res != SQLITE_DONE && res != SQLITE_ROW && res != SQLITE_OK) {
    VAM_DB_LOGE("%s: could not remove rule with id=%s.", __func__, rule_id);
    return res;
  }

  return SQLITE_OK;
}

int32_t EventDB::GetEventsForFrame(const char *frame_id,
                                   uint32_t max_event_count,
                                   std::vector<vaapi_event> *events) {
  if (!events || !frame_id || !strlen(frame_id)) {
    return SQLITE_ERROR;
  }

  std::stringstream sql_condition;
  sql_condition << "WHERE FrameId='" << frame_id << "'";
  if (max_event_count) {
      sql_condition << " LIMIT " << max_event_count;
  }

  return GetMultipleEvents(sql_condition.str().c_str(), events);
}

int32_t EventDB::GetEventsForInterval(uint32_t max_event_count,
                                      uint64_t pts_begin,
                                      uint64_t pts_end,
                                      std::vector<vaapi_event> *events,
                                      uint32_t *type,
                                      uint32_t num_types) {
  if (!events) {
    return SQLITE_ERROR;
  }

  std::stringstream sql_condition;
  sql_condition << "WHERE Pts >= " << pts_begin;
  sql_condition << " AND Pts <= " << pts_end;
  if (type && type[0]) {
      sql_condition << " AND (Type=" << type[0];
      for (uint32_t i = 1; i < num_types; i++) {
          sql_condition << " OR Type=" << type[i];
      }
      sql_condition << ")";
  }
  if (max_event_count) {
      sql_condition << " LIMIT " << max_event_count;
  }

  return GetMultipleEvents(sql_condition.str().c_str(), events);
}

int32_t EventDB::GetLastEvents(uint32_t max_event_count,
                               std::vector<vaapi_event> *events,
                               uint32_t *type,
                               uint32_t num_types) {
  if (!events) {
    return SQLITE_ERROR;
  }

  std::stringstream sql_condition;
  if (type && type[0]) {
      sql_condition << "WHERE (Type=" << type[0];
      for (uint32_t i = 1; i < num_types; i++) {
        sql_condition << " OR Type=" << type[i];
      }
      sql_condition << ") ";
  }
  sql_condition << "ORDER BY Pts DESC";
  if (max_event_count) {
      sql_condition << " LIMIT " << max_event_count;
  }

  return GetMultipleEvents(sql_condition.str().c_str(), events);
}

int32_t EventDB::GetEventsTimestamps(std::vector<std::string> *timestamps) {
  if (!timestamps) {
    ALOGE("%s: invalid input param", __func__);
    return SQLITE_ERROR;
  }

  vector<SQLiteDataEntry> dbRes;
  SQLiteDataEntry e;

  // date timestamp
  e.type = SQLiteDataType::INTEGER64;
  e.addr = NULL;
  dbRes.push_back(e);

  std::stringstream sql;
  sql << "SELECT Pts FROM Events;";

  SQLiteQuery *q = db_.PrepareQuery(sql.str().c_str());
  SQLiteQueryCleanup qc(q, &db_);
  int32_t res;
  do {
    res = db_.GetNextResult(q, &dbRes);
    if (res == SQLITE_ROW) {
      std::stringstream ss;
      ss << *((uint64_t *)dbRes[0].addr);
      timestamps->push_back(ss.str().c_str());
      delete [] (uint8_t *)dbRes[0].addr;
      dbRes[0].addr = NULL;
    } else if (res != SQLITE_DONE) {
      return res;
    }
  } while (res == SQLITE_ROW);

  return SQLITE_OK;

}

int32_t EventDB::GetEventCount(uint32_t *event_count) {
  if (!event_count) {
    ALOGE("%s: invalid input param", __func__);
    return SQLITE_ERROR;
  }
  std::stringstream sql;
  sql << "SELECT COUNT(*) FROM Events;";

  vector<SQLiteDataEntry> dbRes;
  SQLiteDataEntry e;
  e.type = SQLiteDataType::INTEGER;
  e.addr = NULL;
  dbRes.push_back(e);

  SQLiteQuery *q = db_.PrepareQuery(sql.str().c_str());
  SQLiteQueryCleanup qc(q, &db_);
  int32_t res = db_.GetNextResult(q, &dbRes);
  if (res != SQLITE_DONE && res != SQLITE_ROW && res != SQLITE_OK) {
    return SQLITE_ERROR;
  }

  *event_count = *((int32_t *)dbRes[0].addr);
  delete [] (uint8_t *)dbRes[0].addr;

  return SQLITE_OK;
}

int32_t EventDB::SetEventPageSize(uint32_t page_size) {
  if (!page_size) {
    ALOGE("%s: page size can't be 0", __func__);
    return SQLITE_ERROR;
  }
  current_event_page_size_ = page_size;
  return SQLITE_OK;
}

int32_t EventDB::GetEventPageSize(uint32_t *page_size) {
  if (!page_size) {
    ALOGE("%s: invalid input param", __func__);
    return SQLITE_ERROR;
  }
  *page_size = current_event_page_size_;
  return SQLITE_OK;
}

int32_t EventDB::GetEventPage(uint32_t page_index,
                              std::vector<vaapi_event> *events,
                              uint32_t *type,
                              uint32_t num_types) {
  if (!events) {
    ALOGE("%s: invalid imput param", __func__);
    return SQLITE_ERROR;
  }

  std::stringstream sql_condition;
  if (type && type[0]) {
      sql_condition << "WHERE (Type=" << type[0];
      for (uint32_t i = 1; i < num_types; i++) {
        sql_condition << " OR Type=" << type[i];
      }
      sql_condition << ") ";
  }
  sql_condition << "LIMIT "<< current_event_page_size_;
  if (page_index) {
      sql_condition << " OFFSET " << page_index * current_event_page_size_;
  }

  return GetMultipleEvents(sql_condition.str().c_str(), events);

}

int32_t EventDB::GetEventsByIndexRange(std::vector<vaapi_event> *events,
                                       uint32_t *type,
                                       uint32_t num_types,
                                       uint32_t *idx_range,
                                       uint32_t num_ranges) {
  if (!events || !idx_range || (num_ranges % 2)) {
    ALOGE("%s: invalid input param", __func__);
    return SQLITE_ERROR;
  }

  std::stringstream baseSql;
  if (type && type[0]) {
    baseSql << "WHERE (Type=" << type[0];
    for (uint32_t i = 1; i < num_types; i++) baseSql << " OR Type=" << type[i];
    baseSql << ") ";
  }

  for (uint32_t i = 0; i < num_ranges; i+=2) {
    std::stringstream sql_condition;
    sql_condition << baseSql.str() << "LIMIT "<< (idx_range[i+1] - idx_range[i] + 1);
    if (idx_range[i]) sql_condition << " OFFSET " << idx_range[i];

    std::vector<vaapi_event> eList;
    GetMultipleEvents(sql_condition.str().c_str(), &eList);
    events->insert(events->end(), eList.begin(), eList.end());
  }
  return SQLITE_OK;
}

int32_t EventDB::GetEventsByIndexRangeBeforeTS(std::vector<vaapi_event> *events,
                                               uint32_t *type,
                                               uint32_t num_types,
                                               uint32_t *idx_range,
                                               uint32_t num_ranges,
                                               uint64_t pts) {
  if (!events || !idx_range || (num_ranges % 2)) {
    ALOGE("%s: invalid input param", __func__);
    return SQLITE_ERROR;
  }

  std::stringstream baseSql;
  baseSql << "WHERE Pts <= " << pts;
  if (type && type[0]) {
    baseSql << "AND (Type=" << type[0];
    for (uint32_t i = 1; i < num_types; i++) baseSql << " OR Type=" << type[i];
    baseSql << ") ";
  }

  for (uint32_t i = 0; i < num_ranges; i+=2) {
    std::stringstream sql_condition;
    sql_condition << baseSql.str() << "LIMIT "<< (idx_range[i+1] - idx_range[i] + 1);
    if (idx_range[i]) sql_condition << " OFFSET " << idx_range[i];

    std::vector<vaapi_event> eList;
    GetMultipleEvents(sql_condition.str().c_str(), &eList);
    events->insert(events->end(), eList.begin(), eList.end());
  }

  return SQLITE_OK;
}

int32_t EventDB::GetEventsByIndexRangeAfterTS(std::vector<vaapi_event> *events,
                                              uint32_t *type,
                                              uint32_t num_types,
                                              uint32_t *idx_range,
                                              uint32_t num_ranges,
                                              uint64_t pts) {
  if (!events || !idx_range || (num_ranges % 2)) {
    ALOGE("%s: invalid input param", __func__);
    return SQLITE_ERROR;
  }

  std::stringstream baseSql;
  baseSql << "WHERE Pts >= " << pts;
  if (type && type[0]) {
    baseSql << "AND (Type=" << type[0];
    for (uint32_t i = 1; i < num_types; i++) baseSql << " OR Type=" << type[i];
    baseSql << ") ";
  }

  for (uint32_t i = 0; i < num_ranges; i+=2) {
    std::stringstream sql_condition;
    sql_condition << baseSql.str() << "LIMIT "<< (idx_range[i+1] - idx_range[i] + 1);
    if (idx_range[i]) sql_condition << " OFFSET " << idx_range[i];

    std::vector<vaapi_event> eList;
    GetMultipleEvents(sql_condition.str().c_str(), &eList);
    events->insert(events->end(), eList.begin(), eList.end());
  }
  return SQLITE_OK;
}

int32_t EventDB::GetEventsForRule(const char *rule_id,
                                  uint32_t max_event_count,
                                  uint64_t pts_begin,
                                  uint64_t pts_end,
                                  std::vector<vaapi_event> *events) {
  if (!events || !IsVAAPIIdValid(rule_id)) {
    return SQLITE_ERROR;
  }

  std::stringstream sql_condition;
  sql_condition << "WHERE RuleId='" << rule_id << "'";
  sql_condition << " AND Pts >= " << pts_begin;
  sql_condition << " AND Pts <= " << pts_end;
  if (max_event_count) {
      sql_condition << " LIMIT " << max_event_count;
  }

  return GetMultipleEvents(sql_condition.str().c_str(), events);
}

int32_t EventDB::GetEventsForRule(const char *rule_id,
                                  uint32_t max_event_count,
                                  std::vector<vaapi_event> *events) {
  if (!events || !IsVAAPIIdValid(rule_id)) {
    return SQLITE_ERROR;
  }

  std::stringstream sql_condition;
  sql_condition << "WHERE RuleId='" << rule_id << "'";
  if (max_event_count) {
      sql_condition << " LIMIT " << max_event_count;
  }

  return GetMultipleEvents(sql_condition.str().c_str(), events);
}

int32_t EventDB::GetEvent(const char *id, vaapi_event *event) {
  if (!IsVAAPIIdValid(id)) {
      return SQLITE_ERROR;
  }

  std::stringstream sql_condition;
  sql_condition << "WHERE Id='" << id << "'";

  std::vector<vaapi_event> events;
  int32_t res = GetMultipleEvents(sql_condition.str().c_str(), &events);
  if (!events.size()) {
    res = SQLITE_ERROR;
  }
  if (res == SQLITE_OK) {
    *event = events[0];
  }

  return res;
}

int32_t EventDB::GetFrameForEvent(const char *event_id,
                                  uint8_t **addr,
                                  size_t *size) {
  if (!IsVAAPIIdValid(event_id)) {
    VAM_DB_LOGE("%s: invalid input parameter", __func__);
    return SQLITE_ERROR;
  }

  std::string frameId;
  GetFrameIdForEvent(event_id, &frameId);
  if (!IsVAAPIIdValid(frameId.c_str())) {
      VAM_DB_LOGE("%s: no frame id found for event %s", __func__, event_id);
    return SQLITE_ERROR;
  }

  return GetImage(frameId.c_str(), addr, size);
}

int32_t EventDB::GetVideoForEvent(const char *event_id,
                                  uint8_t **addr,
                                  size_t *size) {
  if (!IsVAAPIIdValid(event_id)) {
    VAM_DB_LOGE("%s: invalid input parameter", __func__);
    return SQLITE_ERROR;
  }

  std::string frameId;
  GetFrameIdForEvent(event_id, &frameId);
  if (!IsVAAPIIdValid(frameId.c_str())) {
      VAM_DB_LOGE("%s: no frame id found for event %s", __func__, event_id);
    return SQLITE_ERROR;
  }

  return GetVideo(frameId.c_str(), addr, size);
}

int32_t EventDB::GetThumbnailForEvent(const char *event_id,
                                      uint8_t **addr,
                                      size_t *size) {
  if (!IsVAAPIIdValid(event_id)) {
    VAM_DB_LOGE("%s: invalid input parameter", __func__);
    return SQLITE_ERROR;
  }

  std::string frameId;
  GetFrameIdForEvent(event_id, &frameId);
  if (!IsVAAPIIdValid(frameId.c_str())) {
      VAM_DB_LOGE("%s: no frame id found for event %s", __func__, event_id);
    return SQLITE_ERROR;
  }

  std::stringstream tn;
  tn << frameId << "-thumbnail";

  return GetImage(tn.str().c_str(), addr, size);
}

int32_t EventDB::GetMetadataForFrame(const char *frame_id,
                                     vaapi_metadata_frame *meta) {
  if (!frame_id || !strlen(frame_id) || !meta) {
      return SQLITE_ERROR;
  }

  memset(meta, 0, sizeof(vaapi_metadata_frame));

  std::vector<vaapi_event> events;
  GetEventsForFrame(frame_id, 0, &events);
  if (!events.size()) {
    VAM_DB_LOGE("%s: could not find events for frame %s", __func__, frame_id);
    return SQLITE_ERROR;
  }

  uint32_t total_count = 0;
  std::vector<vaapi_metadata_frame> events_meta;
  for (size_t i = 0; i < events.size(); i++) {
      vaapi_metadata_frame tmp_meta;
      GetMetadataForEvent(events[i].id, &tmp_meta);
      events_meta.push_back(tmp_meta);
      total_count+= tmp_meta.object_num;
  }

  if (!total_count) {
    VAM_DB_LOGE("%s: no objects for any of the events for frame %s",
            __func__, frame_id);
    return SQLITE_ERROR;
  }

  meta->objects = new vaapi_object[total_count];
  for (size_t i = 0; i < events_meta.size(); i++) {
    memcpy(&meta->objects[meta->object_num],
           events_meta[i].objects,
           events_meta[i].object_num);
    meta->object_num += events_meta[i].object_num;
    delete [] events_meta[i].objects;
  }

  return SQLITE_OK;
}

int32_t EventDB::GetMetadataForEvent(const char *event_id,
                                     vaapi_metadata_frame *meta) {
  if (!IsVAAPIIdValid(event_id) || !meta) {
      return SQLITE_ERROR;
  }

  memset(meta, 0, sizeof(vaapi_metadata_frame));

  vaapi_event e;
  std::string frameId;
  GetFrameIdForEvent(event_id, &frameId);
  memset(&e, 0, sizeof(vaapi_event));
  ReadEvent(frameId.c_str(), event_id, &e);
  if (strcmp(event_id, e.id)) {
    VAM_DB_LOGE("%s: no such event with id %s", __func__, event_id);
    return SQLITE_ERROR;
  }

  std::vector<vaapi_object> objects;
  int32_t res = ReadMeta(e.obj.id, &objects);
  if (res == SQLITE_OK && objects.size()) {
    meta->object_num = objects.size();
    meta->objects = new vaapi_object[meta->object_num];
    memcpy(meta->objects, objects.data(), sizeof(vaapi_object)*objects.size());
  }

  return res;
}

int32_t EventDB::GetImage(const char *frame_id, uint8_t **addr, size_t *size) {
  if (!frame_id || !strlen(frame_id) || !addr) {
    return SQLITE_ERROR;
  }

  size_t bytesRead = 0;
  int32_t res = ReadJpeg(frame_id, addr, &bytesRead);
  if (size) {
    *size = bytesRead;
  }

  return res;
}

int32_t EventDB::GetVideo(const char *frame_id, uint8_t **addr, size_t *size) {
  if (!frame_id || !strlen(frame_id) || !addr) {
    VAM_DB_LOGE("%s: invalid input arguments.", __func__);
    return SQLITE_ERROR;
  }

  size_t bytesRead = 0;
  int32_t res = ReadVideo(frame_id, addr, &bytesRead);
  if (size) {
    *size = bytesRead;
  }

  return res;
}

int32_t EventDB::GetThumbnail(const char *frame_id,
                              uint8_t **addr,
                              size_t *size) {
  if (!frame_id || !strlen(frame_id) || !addr) {
    VAM_DB_LOGE("%s: invalid input arguments.", __func__);
    return SQLITE_ERROR;
  }

  std::stringstream tn;
  tn << frame_id << "-thumbnail";

  size_t bytesRead = 0;
  int32_t res = ReadJpeg(tn.str().c_str(), addr, &bytesRead);
  if (size) {
    *size = bytesRead;
  }

  return res;
}

int32_t EventDB::GetRule(const char *rule_id, vaapi_rule *rule) {
  if (!IsVAAPIIdValid(rule_id) || !rule) {
    return SQLITE_ERROR;
  }

  vector<SQLiteDataEntry> dbRes;
  SQLiteDataEntry e;

  // event id
  e.type = SQLiteDataType::BLOB;
  e.addr = NULL;
  dbRes.push_back(e);

  std::stringstream sql;
  sql << "SELECT Data FROM Rules WHERE Id='" << rule_id << "';";

  SQLiteQuery *q = db_.PrepareQuery(sql.str().c_str());
  SQLiteQueryCleanup qc(q, &db_);
  int32_t res = db_.GetNextResult(q, &dbRes);
  if (res != SQLITE_DONE && res != SQLITE_ROW) {
    VAM_DB_LOGE("%s: could not find rule with id=%s", __func__, rule_id);
    return res;
  }

  memcpy(rule, dbRes[0].addr, sizeof(vaapi_rule));
  delete [] (uint8_t *)dbRes[0].addr;

  return SQLITE_OK;
}

int32_t EventDB::GetMultipleIds(const char *table_name,
                                const char *table_column,
                                std::vector<std::string> *ids) {
  if (!table_name || !table_column ||
      !strlen(table_name) || !strlen(table_column)) {
    return SQLITE_ERROR;
  }

  vector<SQLiteDataEntry> dbRes;
  SQLiteDataEntry e;

  // event id
  e.type = SQLiteDataType::TEXT;
  e.addr = NULL;
  dbRes.push_back(e);

  std::stringstream sql;
  sql << "SELECT DISTINCT " << table_column << " FROM " << table_name << ";";

  SQLiteQuery *q = db_.PrepareQuery(sql.str().c_str());
  SQLiteQueryCleanup qc(q, &db_);
  int32_t res;
  do {
    res = db_.GetNextResult(q, &dbRes);
    if (res == SQLITE_ROW) {
      ids->push_back((char *)dbRes[0].addr);
      delete [] (uint8_t *)dbRes[0].addr;
      dbRes[0].addr = NULL;
    } else if (res != SQLITE_DONE) {
      return res;
    }
  } while (res == SQLITE_ROW);

  return SQLITE_OK;
}

int32_t EventDB::GetAllEventIds(std::vector<std::string> *ids) {
  return GetMultipleIds("Events", "Id", ids);
}

int32_t EventDB::GetAllFrameIds(std::vector<std::string> *ids) {
  return GetMultipleIds("Events", "FrameId", ids);
}

int32_t EventDB::GetAllRulesIds(std::vector<std::string> *ids) {
  return GetMultipleIds("Rules", "Id", ids);
}

/* ##########################################################################
 * ###### Class for storing enrolled images for FaceRecognition engine ######
 * ##########################################################################
 */
EnrolledFacesDB::EnrolledFacesDB() {
}

EnrolledFacesDB::~EnrolledFacesDB() {
  db_.CloseDB();
}

int32_t EnrolledFacesDB::OpenDB() {
  std::stringstream db_dir;
  db_dir << VAM_ENROLLDB_DIR;
  mkdir(db_dir.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  images_dir_ = db_dir.str() + "/images";
  mkdir(images_dir_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  std::stringstream db_name;
  db_name << db_dir.str() << "/enrolledfaces.db";

  int res = db_.OpenDB(db_name.str().c_str());
  if (res != SQLITE_OK) {
    VAM_DB_LOGE("%s: could not open database '%s'", __func__, db_name.str().c_str());
    return res;
  }

  if (!db_.checkIfExisting("EnrollInfo", NULL)) {
    std::stringstream sql;
    sql << "CREATE TABLE EnrollInfo( ";
    sql << "Id TEXT PRIMARY KEY NOT NULL, ";
    sql << "DisplayName TEXT, ";
    sql << "ImageId TEXT NOT NULL, ";
    sql << "Type INT, ";
    sql << "ImageFormat INT, ";
    sql << "ImageWidth1 INT, ImageHeight1 INT, ImageStride1 INT, ";
    sql << "ImageWidth2 INT, ImageHeight2 INT, ImageStride2 INT, ";
    sql << "ImageWidth3 INT, ImageHeight3 INT, ImageStride3 INT);";
    SQLiteQuery *q = db_.PrepareQuery(sql.str().c_str());
    SQLiteQueryCleanup qc(q, &db_);
    if (!q) {
      VAM_DB_LOGE("%s: could not create query for enrollment table", __func__);
      return SQLITE_ERROR;
    }

    res = db_.GetNextResult(q, NULL);
    if (res != SQLITE_DONE && res != SQLITE_ROW) {
      VAM_DB_LOGE("%s: could not create enrollment table", __func__);
      return res;
    }
  }

  return SQLITE_OK;
}

int32_t EnrolledFacesDB::CloseDB() {
  return db_.CloseDB();
}

bool EnrolledFacesDB::IsVAAPIIdValid(const char *vaapi_id) {
    if (!vaapi_id) return false;
    if (strlen(vaapi_id) != VAAPI_UUID_LEN - 1) return false;
    for (int j = 0; j < 8; j++) {
      for (int i = 0; i < 4; i++, vaapi_id++) {
        if (!isxdigit(*vaapi_id)) return false;
      }
      if (j > 0 && j < 5) {
        if(*vaapi_id != '-') return false;
        vaapi_id++;
      }
    }

    return true;
}

int32_t EnrolledFacesDB::WriteImage(const vaapi_enrollment_info *enroll_info) {
  if (!IsVAAPIIdValid(enroll_info->img_id) || !enroll_info->img_data[0]) {
    return SQLITE_ERROR;
  }

  std::stringstream path;
  path << images_dir_ << "/" << enroll_info->img_id;

  std::ofstream fs(path.str().c_str(), std::ofstream::binary | std::ofstream::app);
  for (int i = 0; i < 3; i++) {
    fs.write((const char *)enroll_info->img_data[i],
            enroll_info->img_pitch[i] * enroll_info->img_height[i]);
  }
  fs.close();


  return SQLITE_OK;
}

int32_t EnrolledFacesDB::ReadImage(vaapi_enrollment_info *enroll_info) {
  std::lock_guard<std::mutex> lg(delete_lock_);
  if (!IsVAAPIIdValid(enroll_info->img_id)) {
    return SQLITE_ERROR;
  }

  std::stringstream path;
  path << images_dir_ << "/" << enroll_info->img_id;

  std::ifstream fs(path.str().c_str(), std::ifstream::binary);
  if (!fs.is_open() || fs.eof()) {
    return SQLITE_ERROR;
  }

  for (int i = 0; i < 3; i++) {
    uint32_t p_size = enroll_info->img_pitch[i] * enroll_info->img_height[i];
    if (p_size) {
      enroll_info->img_data[i] = new uint8_t[p_size];
    }
  }

  for (int i = 0; i < 3; i++) {
    uint32_t p_size = enroll_info->img_pitch[i] * enroll_info->img_height[i];
    if (!enroll_info->img_data[i] || !p_size) {
      continue;
    }
    fs.read((char *)enroll_info->img_data[i], p_size);
  }
  fs.close();

  return SQLITE_OK;
}

int32_t EnrolledFacesDB::DeleteImage(const char *img_id) {
  if (!IsVAAPIIdValid(img_id)) {
    return SQLITE_ERROR;
  }

  std::lock_guard<std::mutex> lg(delete_lock_);
  std::stringstream path;
  path << images_dir_ << "/" << img_id;
  remove(path.str().c_str());
  return SQLITE_OK;
}

int32_t EnrolledFacesDB::GetMultipleIDs(const char *sql_condition,
                                        const char *column,
                                        std::vector<std::string> *ids) {
    vector<SQLiteDataEntry> dbRes;
    SQLiteDataEntry e;

    // enrolled image id
    e.type = SQLiteDataType::TEXT;
    e.addr = NULL;
    dbRes.push_back(e);

    std::stringstream sql;
    sql << "SELECT " << column << " FROM EnrollInfo ";
    if (sql_condition) {
      sql << sql_condition;
    }
    sql << ";";

    SQLiteQuery *q = db_.PrepareQuery(sql.str().c_str());
    SQLiteQueryCleanup qc(q, &db_);
    int32_t res;
    do {
      res = db_.GetNextResult(q, &dbRes);
      if (res == SQLITE_ROW) {
        ids->push_back((char *)dbRes[0].addr);
        delete [] (uint8_t *)dbRes[0].addr;
        dbRes[0].addr = NULL;
      } else if (res != SQLITE_DONE) {
        return res;
      }
    } while (res == SQLITE_ROW);

    return SQLITE_OK;
}

/* ############################
 * ###### Public methods ######
 * ############################
 */

int32_t EnrolledFacesDB::Insert(const vaapi_enrollment_info *enroll_info) {
  if (!enroll_info ||
      !IsVAAPIIdValid(enroll_info->id) ||
      !IsVAAPIIdValid(enroll_info->img_id)) {
    VAM_DB_LOGE("%s: invalid input arguments.", __func__);
    return SQLITE_ERROR;
  }

  int32_t res = WriteImage(enroll_info);
  if (res != SQLITE_OK) {
    VAM_DB_LOGE("%s: could not save the enrolled image.", __func__);
    return res;
  }

  std::stringstream sql;
  sql << "INSERT INTO EnrollInfo(Id, DisplayName, ImageId, Type, ImageFormat, ";
  sql << "ImageWidth1, ImageHeight1, ImageStride1, ";
  sql << "ImageWidth2, ImageHeight2, ImageStride2, ";
  sql << "ImageWidth3, ImageHeight3, ImageStride3) ";
  sql << "VALUES (";
  sql << "'" << enroll_info->id << "', ";
  sql << "'" << enroll_info->display_name << "', ";
  sql << "'" << enroll_info->img_id << "', ";
  sql << (int32_t)enroll_info->type << ", ";
  sql << (int32_t)enroll_info->img_format << ", ";
  for (int i = 0; i < 3; i++) {
      sql << enroll_info->img_width[i] << ", ";
      sql << enroll_info->img_height[i] << ", ";
      sql << enroll_info->img_pitch[i];
      if (i < 2) sql << ", "; else sql << ");";
  }

  vector<SQLiteDataEntry> arguments;
  SQLiteQuery *q = db_.PrepareQuery(sql.str().c_str());
  SQLiteQueryCleanup qc(q, &db_);
  res = db_.GetNextResult(q, &arguments);

  if (res != SQLITE_DONE && res != SQLITE_ROW && res != SQLITE_OK) {
    VAM_DB_LOGE("%s: could not insert enroll info to database.", __func__);
    return res;
  }

  return SQLITE_OK;
}

int32_t EnrolledFacesDB::Remove(const char *id) {
  if (!IsVAAPIIdValid(id)) {
    VAM_DB_LOGE("%s: invalid input arguments.", __func__);
    return SQLITE_ERROR;
  }

  {
    std::vector<std::string> imageIds;
    std::stringstream sql_condition;
    sql_condition << "WHERE Id='" << id << "' OR ImageId='" << id << "'";
    GetMultipleIDs(sql_condition.str().c_str(), "ImageId", &imageIds);

    for (size_t i = 0; i < imageIds.size(); i++) {
      DeleteImage(imageIds[i].c_str());
    }
  }

  {
    std::stringstream sql;
    sql << "DELETE FROM EnrollInfo ";
    sql << "WHERE Id='" << id << "' OR ImageId='" << id << "';";

    SQLiteQuery *q = db_.PrepareQuery(sql.str().c_str());
    SQLiteQueryCleanup qc(q, &db_);
    int32_t res = db_.GetNextResult(q, NULL);
    if (res != SQLITE_DONE && res != SQLITE_ROW && res != SQLITE_OK) {
      VAM_DB_LOGE("%s: could not remove enroll %s.", __func__, id);
      return res;
    }
  }

  return SQLITE_OK;
}

int32_t EnrolledFacesDB::GetFeatureIds(std::vector<std::string> *ids) {
  if (!ids) {
    VAM_DB_LOGE("%s: invalid input parameter", __func__);
    return SQLITE_ERROR;
  }

  return GetMultipleIDs(nullptr, "Id", ids);
}

int32_t EnrolledFacesDB::GetImageIds(std::vector<std::string> *ids) {
  if (!ids) {
    VAM_DB_LOGE("%s: invalid input parameter", __func__);
    return SQLITE_ERROR;
  }

  return GetMultipleIDs(nullptr, "ImageId", ids);
}

int32_t EnrolledFacesDB::GetImageIdsForFeature(const char *feature_id,
                                               std::vector<std::string> *ids) {
  if (!IsVAAPIIdValid(feature_id) || !ids) {
    VAM_DB_LOGE("%s: invalid input parameter", __func__);
    return SQLITE_ERROR;
  }

  std::stringstream sql_condition;
  sql_condition << "WHERE Id='" << feature_id << "'";

  return GetMultipleIDs(sql_condition.str().c_str(), "ImageId", ids);
}

int32_t EnrolledFacesDB::GetImageIdsForDisplayName(const char *display_name,
                                               std::vector<std::string> *ids) {
  if (!display_name || !ids) {
    VAM_DB_LOGE("%s: invalid input parameter", __func__);
    return SQLITE_ERROR;
  }

  std::stringstream sql_condition;
  sql_condition << "WHERE DisplayName='" << display_name << "'";

  return GetMultipleIDs(sql_condition.str().c_str(), "ImageId", ids);
}

int32_t EnrolledFacesDB::GetFeatureIdForImage(const char *image_id,
                             std::vector<std::string> *ids) {
  if (!IsVAAPIIdValid(image_id) || !ids) {
    VAM_DB_LOGE("%s: invalid input parameter", __func__);
    return SQLITE_ERROR;
  }

  std::stringstream sql_condition;
  sql_condition << "WHERE ImageId='" << image_id << "'";

  return GetMultipleIDs(sql_condition.str().c_str(), "Id", ids);
}

int32_t EnrolledFacesDB::GetFeatureIdsForDisplayName(const char *display_name,
                                             std::vector<std::string> *ids) {
  if (!display_name || !ids) {
    VAM_DB_LOGE("%s: invalid input parameter", __func__);
    return SQLITE_ERROR;
  }

  std::stringstream sql_condition;
  sql_condition << "WHERE DisplayName='" << display_name << "'";

  return GetMultipleIDs(sql_condition.str().c_str(), "Id", ids);
}

int32_t EnrolledFacesDB::GetEnrollInfo(const char *img_id,
                                       vaapi_enrollment_info *enroll_info,
                                       bool read_image_data) {
  if (!IsVAAPIIdValid(img_id) || !enroll_info) {
    VAM_DB_LOGE("%s: invalid input parameter", __func__);
    return SQLITE_ERROR;
  }

  memset(enroll_info, 0, sizeof(*enroll_info));

  int32_t res = SQLITE_OK;
  {
    vector<SQLiteDataEntry> dbRes;
    SQLiteDataEntry e;

    // enrolled id
    e.type = SQLiteDataType::TEXT;
    e.addr = NULL;
    dbRes.push_back(e);

    // enrolled display_name
    e.type = SQLiteDataType::TEXT;
    e.addr = NULL;
    dbRes.push_back(e);

    // enrolled image id
    e.type = SQLiteDataType::TEXT;
    e.addr = NULL;
    dbRes.push_back(e);

    // enrolled type
    e.type = SQLiteDataType::INTEGER;
    e.addr = NULL;
    dbRes.push_back(e);

    // enrolled image format
    e.type = SQLiteDataType::INTEGER;
    e.addr = NULL;
    dbRes.push_back(e);

    // enrolled image width, height and stride
    e.type = SQLiteDataType::INTEGER;
    e.addr = NULL;
    for (int i = 0; i < 9; i++) {
        dbRes.push_back(e);
    }

    std::stringstream sql;
    sql << "SELECT * FROM EnrollInfo ";
    sql << "WHERE ImageId='" << img_id << "';";

    SQLiteQuery *q = db_.PrepareQuery(sql.str().c_str());
    SQLiteQueryCleanup qc(q, &db_);
    res = db_.GetNextResult(q, &dbRes);
    if (res != SQLITE_DONE && res != SQLITE_ROW && res != SQLITE_OK) {
      VAM_DB_LOGE("%s: could not read enroll info for %s.", __func__, img_id);
      return res;
    }

    if (!dbRes[0].addr) {
      VAM_DB_LOGE("%s: no enroll info entry for %s.", __func__, img_id);
      return res;
    }

    int iArg = 0;
    strncpy(enroll_info->id, (char *)dbRes[iArg++].addr, VAAPI_UUID_LEN);
    strncpy(enroll_info->display_name, (char *)dbRes[iArg++].addr, VAAPI_NAME_LEN);
    strncpy(enroll_info->img_id, (char *)dbRes[iArg++].addr, VAAPI_UUID_LEN);
    enroll_info->type = *((vaapi_object_type *)dbRes[iArg++].addr);
    enroll_info->img_format = *((vaapi_img_format *)dbRes[iArg++].addr);
    for (int i = 0; i < 3; i++) {
        enroll_info->img_width[i]  = *((uint32_t *)dbRes[iArg++].addr);
        enroll_info->img_height[i] = *((uint32_t *)dbRes[iArg++].addr);
        enroll_info->img_pitch[i]  = *((uint32_t *)dbRes[iArg++].addr);
    }

    for (size_t i = 0; i < dbRes.size(); i++) {
      delete [] (uint8_t *)dbRes[i].addr;
    }

    res = SQLITE_OK;
  }

  if (read_image_data) {
    res = ReadImage(enroll_info);
  }

  return res;
}

} //namespace database ends here
} //namespace qmmf ends here
