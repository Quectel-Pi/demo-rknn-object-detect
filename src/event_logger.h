#ifndef _EVENT_LOGGER_H_
#define _EVENT_LOGGER_H_

#include <vector>
#include <string>
#include "tracker.h"

// Emit one JSON line per frame on stdout. Lines are designed to be
// consumed line-by-line by upstream tools (jq, grep, journal, etc.).
//
// Example lines:
//   {"event":"frame","frame":1234,"fps":27.3,"tracks":[{"id":5,"cls":"person","score":0.82,"x":120,"y":80,"w":45,"h":180}, ...]}
//   {"event":"appear","id":5,"cls":"person"}
//   {"event":"counts","counts":{"person":3,"car":5}}
//
// "appear" lines are emitted only when a new track_id appears.
// "counts" line is emitted every N frames (default 30).
// "frame"  line is emitted every frame with current track list.

class EventLogger
{
public:
  EventLogger(int count_every = 30);
  void log_frame(int frame_index,
                 int width, int height,
                 float fps,
                 const std::vector<TrackBox> &tracks);
private:
  int count_every_;
  int frames_since_counts_;
};

#endif
