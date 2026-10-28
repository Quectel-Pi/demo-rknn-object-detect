#ifndef _TRACKER_H_
#define _TRACKER_H_

#include <vector>
#include <string>
#include <map>

// Single detection (same shape as detect_result_t but decoupled).
struct TrackBox
{
  int track_id;
  std::string name;
  float left, top, right, bottom;
  float score;
  int   age;     // frames since last match
  int   hits;    // total frames matched
  bool  first_seen_this_frame;
};

struct Detection
{
  std::string name;
  float left, top, right, bottom;
  float score;
};

class IoUTracker
{
public:
  IoUTracker(float iou_thresh = 0.3f, int max_age = 30);
  ~IoUTracker();

  // Update tracks with current detections. Returns the per-frame tracked list
  // (track_id assigned; first_seen_this_frame=true for newly appeared).
  std::vector<TrackBox> update(const std::vector<Detection> &dets);

  // Cumulative counter: number of unique track_ids ever created (per class).
  int total_count(const std::string &class_name) const;

  // Print all class->count to stdout as JSON.
  void print_counts_json() const;

private:
  float iou(float ax1, float ay1, float ax2, float ay2,
            float bx1, float by1, float bx2, float by2) const;
  int next_id();
  int max_age_;
  float iou_thresh_;
  std::vector<TrackBox> tracks_;
  int next_id_val_;
  // class -> total unique objects spawned
  std::map<std::string,int> total_counts_;
};

#endif
