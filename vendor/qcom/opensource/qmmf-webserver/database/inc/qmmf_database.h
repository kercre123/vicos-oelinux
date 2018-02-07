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
#ifndef QMMF_DATABASE_H_
#define QMMF_DATABASE_H_

#include <VAM/vaapi.h>
#include <VAM/VAMUtilities.h>
#include <string.h>
#include <sqlite3.h>
#include <utils/Log.h>
#include <string>
#include <vector>

/* log levels:
 *   0 - disabled
 *   1 - errors
 *   2 - errors, debug info
 *   3 - all logs
 */
#define VAM_DATABASE_LOGLEVEL_PROP "vam.database.log_level"
#define VAM_DIR "/data/misc/vam"
#define VAM_ENROLLDB_DIR "/data/misc/vam/facereco"
#define VAM_EVENT_PAGE_SIZE 100

namespace qmmf {

namespace database {

using namespace std;

typedef enum {
  INTEGER = 0,
  INTEGER64,
  DOUBLE,
  TEXT,
  BLOB,
  NULL_ENTRY,
} SQLiteDataType;

typedef struct {
    SQLiteDataType type;
    void *addr;
    uint32_t size;
} SQLiteDataEntry;

typedef struct {
    string sql;
    sqlite3_stmt *stmt;
} SQLiteQuery;

class SQLiteDB {
private:
  void *dbi_;

public:
  SQLiteDB();
  ~SQLiteDB();

  int32_t OpenDB(const char *db_name);
  int32_t CloseDB();
  SQLiteQuery *PrepareQuery(const char *sql);
  int32_t GetNextResult(SQLiteQuery *query, vector<SQLiteDataEntry> *results);
  int32_t FinalizeQuery(SQLiteQuery *query);

  bool checkIfExisting(const char *table_name, const char *column_name);
};

class EventDB {
private:
  SQLiteDB db_;
  std::string session_id_;
  std::string images_dir_;
  std::string videos_dir_;
  std::string events_dir_;

  std::mutex delete_lock_;
  uint32_t current_event_page_size_;

  bool IsVAAPIIdValid(const char *vaapi_id);
  int32_t WriteToFile(const char *file_name, const uint8_t *data, size_t size);
  int32_t WriteJpeg(const char *frame_id, uint8_t *data, size_t size);
  int32_t WriteVideo(const char *frame_id, uint8_t *data, size_t size);
  int32_t WriteMeta(vaapi_object *objects, uint32_t cnt);
  int32_t WriteEvents(const char *frame_id, vaapi_event *events, uint32_t cnt);
  int32_t ReadFromFile(std::ifstream &fs, uint8_t *data, size_t *size);
  int32_t ReadEntireFile(std::ifstream &fs, uint8_t **data, size_t *size);
  int32_t ReadJpeg(const char *frame_id, uint8_t **data, size_t *size);
  int32_t ReadVideo(const char *frame_id, uint8_t **data, size_t *size);
  int32_t ReadMeta(uint32_t obj_id, std::vector<vaapi_object> *objects);
  int32_t ReadEvent(const char *frame_id,
                    const char *event_id,
                    vaapi_event *event);
  int32_t DeleteJpeg(const char *frame_id);
  int32_t DeleteVideo(const char *frame_id);
  int32_t DeleteEvent(const char *frame_id, const char *event_id);
  int32_t GetMultipleEvents(const char *sql_condition,
                            std::vector<vaapi_event> *events);
  int32_t GetMultipleIds(const char *table_name,
                         const char *table_column,
                         std::vector<std::string> *ids);
  int32_t GetEventsCountForFrame(const char *frame_id);

public:
  EventDB();
  ~EventDB();

  int32_t OpenDB(const char *session = nullptr);
  int32_t CloseDB();

  int32_t Insert(const vaapi_snapshot_info *snapshot);
  int32_t InsertMeta(const vaapi_metadata_frame *meta);
  int32_t InsertRule(const vaapi_rule *rule);

  int32_t RemoveEvent(const char *event_id);
  int32_t RemoveRule(const char *rule_id);

  int32_t GetEventsForFrame(const char *frame_id,
                            uint32_t max_event_count,
                            std::vector<vaapi_event> *events);
  int32_t GetEventsForInterval(uint32_t max_event_count,
                               uint64_t pts_begin,
                               uint64_t pts_end,
                               std::vector<vaapi_event> *events,
                               uint32_t *type,
                               uint32_t num_types);
  int32_t GetLastEvents(uint32_t max_event_count,
                        std::vector<vaapi_event> *events,
                        uint32_t *type,
                        uint32_t num_types);
  int32_t GetEventsTimestamps(std::vector<std::string> *timestamps);
  int32_t GetEventCount(uint32_t *event_count);
  int32_t SetEventPageSize(uint32_t page_size);
  int32_t GetEventPageSize(uint32_t *page_size);
  int32_t GetEventPage(uint32_t page_index,
                       std::vector<vaapi_event> *events,
                       uint32_t *type,
                       uint32_t num_types);
  int32_t GetEventsByIndexRange(std::vector<vaapi_event> *events,
                                uint32_t *type,
                                uint32_t num_types,
                                uint32_t *idx_range,
                                uint32_t num_ranges);
  int32_t GetEventsByIndexRangeBeforeTS(std::vector<vaapi_event> *events,
                                        uint32_t *type,
                                        uint32_t num_types,
                                        uint32_t *idx_range,
                                        uint32_t num_ranges,
                                        uint64_t pts);
  int32_t GetEventsByIndexRangeAfterTS(std::vector<vaapi_event> *events,
                                       uint32_t *type,
                                       uint32_t num_types,
                                       uint32_t *idx_range,
                                       uint32_t num_ranges,
                                       uint64_t pts);
  int32_t GetEventsForRule(const char *rule_id,
                           uint32_t max_event_count,
                           uint64_t pts_begin,
                           uint64_t pts_end,
                           std::vector<vaapi_event> *events);
  int32_t GetEventsForRule(const char *rule_id,
                           uint32_t max_event_count,
                           std::vector<vaapi_event> *events);
  int32_t GetEvent(const char *event_id, vaapi_event *event);
  int32_t GetFrameIdForEvent(const char *event_id, std::string *frame_id);
  int32_t GetFrameForEvent(const char *event_id, uint8_t **addr, size_t *size);
  int32_t GetThumbnailForEvent(const char *event_id, uint8_t **addr, size_t *size);
  int32_t GetVideoForEvent(const char *event_id, uint8_t **addr, size_t *size);
  int32_t GetMetadataForFrame(const char *frame_id,
                              vaapi_metadata_frame *meta);
  int32_t GetMetadataForEvent(const char *event_id,
                              vaapi_metadata_frame *meta);
  int32_t GetImage(const char *frame_id, uint8_t **addr, size_t *size);
  int32_t GetVideo(const char *frame_id, uint8_t **addr, size_t *size);
  int32_t GetThumbnail(const char *frame_id, uint8_t **addr, size_t *size);
  int32_t GetRule(const char *rule_id, vaapi_rule *rule);
  int32_t GetAllEventIds(std::vector<std::string> *ids);
  int32_t GetAllFrameIds(std::vector<std::string> *ids);
  int32_t GetAllRulesIds(std::vector<std::string> *ids);

  int32_t GetCurrentSession(std::string *session);
  static int32_t GetAllSessions(std::vector<std::string> *sessions);
};

class EnrolledFacesDB {
private:
  SQLiteDB db_;
  std::string images_dir_;

  std::mutex delete_lock_;

  bool IsVAAPIIdValid(const char *vaapi_id);
  int32_t WriteImage(const vaapi_enrollment_info *enroll_info);
  int32_t ReadImage(vaapi_enrollment_info *enroll_info);
  int32_t DeleteImage(const char *img_id);
  int32_t GetMultipleIDs(const char *sql_condition,
                         const char *column,
                         std::vector<std::string> *ids);
public:
  EnrolledFacesDB();
  ~EnrolledFacesDB();

  int32_t OpenDB();
  int32_t CloseDB();

  int32_t Insert(const vaapi_enrollment_info *enroll_info);

  int32_t Remove(const char *id);

  int32_t GetFeatureIds(std::vector<std::string> *ids);
  int32_t GetImageIds(std::vector<std::string> *ids);
  int32_t GetImageIdsForFeature(const char *feature_id,
                                std::vector<std::string> *ids);
  int32_t GetImageIdsForDisplayName(const char *display_name,
                                    std::vector<std::string> *ids);
  int32_t GetFeatureIdForImage(const char *image_id,
                                 std::vector<std::string> *ids);
  int32_t GetFeatureIdsForDisplayName(const char *display_name,
                                      std::vector<std::string> *ids);
  int32_t GetEnrollInfo(const char *img_id,
                        vaapi_enrollment_info *enroll_info,
                        bool read_image_data = false);
};

} //namespace database ends here
} //namespace qmmf ends here

#endif /* QMMF_DATABASE_H_ */
