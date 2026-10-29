#include "event_logger.h"
#include <cstdio>
#include <cstring>

EventLogger::EventLogger(int count_every)
: count_every_(count_every), frames_since_counts_(0) {}

static void emit_appear(int id, const std::string &cls)
{
  printf("{\"event\":\"appear\",\"id\":%d,\"cls\":\"%s\"}\n",
         id, cls.c_str());
}

void EventLogger::log_frame(int frame_index,
                            int width, int height,
                            float fps,
                            const std::vector<TrackBox> &tracks)
{
  /* frame line */
  printf("{\"event\":\"frame\",\"frame\":%d,\"w\":%d,\"h\":%d,\"fps\":%.1f,\"tracks\":[",
         frame_index, width, height, fps);
  bool first = true;
  for (const auto &t : tracks) {
    if (!first) printf(",");
    first = false;

    int x = (int)t.left, y = (int)t.top;
    int w = (int)(t.right - t.left);
    int h = (int)(t.bottom - t.top);
    printf("{\"id\":%d,\"cls\":\"%s\",\"score\":%.2f,\"x\":%d,\"y\":%d,\"w\":%d,\"h\":%d}",
           t.track_id, t.name.c_str(), t.score, x, y, w, h);

    if (t.first_seen_this_frame) {
      char buf[128];
      snprintf(buf, sizeof(buf), "{\"event\":\"appear\",\"id\":%d,\"cls\":\"%s\"}",
               t.track_id, t.name.c_str());
      /* emit appear after we close the frame line below — but stdout is line
       * buffered when tty; we'll emit a separate appear line right after. */
      /* note: we can't write appear inline without breaking JSON.  Emit
       * the appear events AFTER the frame closes — see the loop below. */
    }
  }
  printf("]}\n");

  for (const auto &t : tracks) {
    if (t.first_seen_this_frame) {
      emit_appear(t.track_id, t.name);
    }
  }

  if (++frames_since_counts_ >= count_every_) {
    frames_since_counts_ = 0;
    printf("{\"event\":\"counts\",\"counts\":{");
    bool first_c = true;
    std::map<std::string,int> agg;
    for (const auto &t : tracks) {
      agg[t.name]++;  /* live count — not cumulative; cumulative needs tracker */
    }
    /* The "live in frame" counts; for cumulative see IoUTracker::print_counts_json */
    for (const auto &kv : agg) {
      if (!first_c) printf(",");
      printf("\"%s_live\":%d", kv.first.c_str(), kv.second);
      first_c = false;
    }
    printf("}}\n");
  }
  fflush(stdout);
}
