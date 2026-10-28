// IoU multi-object tracker (SORT-like, no Kalman) + per-class cumulative counter.
// Lightweight on purpose — runs alongside RKNN inference per frame on A72.

#include "tracker.h"
#include "postprocess.h"   // OBJ_CLASS_NUM

#include <cstdio>
#include <cstring>
#include <algorithm>
#include <map>

IoUTracker::IoUTracker(float iou_thresh, int max_age)
: iou_thresh_(iou_thresh), max_age_(max_age), next_id_val_(1) {}

IoUTracker::~IoUTracker() {}

float IoUTracker::iou(float ax1, float ay1, float ax2, float ay2,
                      float bx1, float by1, float bx2, float by2) const
{
  float xx1 = std::max(ax1, bx1);
  float yy1 = std::max(ay1, by1);
  float xx2 = std::min(ax2, bx2);
  float yy2 = std::min(ay2, by2);
  float w = std::max(0.0f, xx2 - xx1);
  float h = std::max(0.0f, yy2 - yy1);
  float inter = w * h;
  float a_area = (ax2 - ax1) * (ay2 - ay1);
  float b_area = (bx2 - bx1) * (by2 - by1);
  float u = a_area + b_area - inter;
  return u <= 0 ? 0 : inter / u;
}

int IoUTracker::next_id() { return next_id_val_++; }

std::vector<TrackBox> IoUTracker::update(const std::vector<Detection> &dets)
{
  std::vector<int> matched_to_det(dets.size(), -1);   /* idx->track */
  std::vector<int> matched_to_trk(tracks_.size(), -1);/* trk->det */

  if (!tracks_.empty() && !dets.empty()) {
    /* greedy: for each det, pick best IoU track; greedy is fine for < ~64 dets */
    for (size_t di = 0; di < dets.size(); di++) {
      float best = iou_thresh_;
      int best_trk = -1;
      const Detection &d = dets[di];
      for (size_t ti = 0; ti < tracks_.size(); ti++) {
        if (matched_to_trk[ti] >= 0) continue;
        const TrackBox &t = tracks_[ti];
        if (t.name != d.name) continue;  /* per-class matching */
        float i = iou(t.left, t.top, t.right, t.bottom, d.left, d.top, d.right, d.bottom);
        if (i > best) { best = i; best_trk = (int)ti; }
      }
      if (best_trk >= 0) {
        matched_to_det[di] = best_trk;
        matched_to_trk[best_trk] = (int)di;
      }
    }
  }

  std::vector<TrackBox> output;
  output.reserve(dets.size());

  /* update matched + un-matched-detection (new tracks) */
  for (size_t di = 0; di < dets.size(); di++) {
    const Detection &d = dets[di];
    if (matched_to_det[di] >= 0) {
      TrackBox &trk = tracks_[matched_to_det[di]];
      trk.left = d.left; trk.top = d.top; trk.right = d.right; trk.bottom = d.bottom;
      trk.score = d.score;
      trk.age = 0;
      trk.hits++;
      trk.first_seen_this_frame = false;
      output.push_back(trk);
    } else {
      /* new track */
      TrackBox t;
      t.track_id = next_id();
      t.name = d.name;
      t.left = d.left; t.top = d.top; t.right = d.right; t.bottom = d.bottom;
      t.score = d.score;
      t.age = 0;
      t.hits = 1;
      t.first_seen_this_frame = true;
      tracks_.push_back(t);
      total_counts_[d.name]++;
      output.push_back(t);
    }
  }

  /* Resize matched_to_trk to match tracks_ (new tracks were added above) */
  matched_to_trk.resize(tracks_.size(), -1);

  /* aged-out pruning: increment age for unmatched tracks, drop if too old */
  for (size_t ti = 0; ti < tracks_.size(); ) {
    if (matched_to_trk[ti] < 0) {
      tracks_[ti].age++;
      if (tracks_[ti].age > max_age_) {
        tracks_.erase(tracks_.begin() + ti);
        matched_to_trk.erase(matched_to_trk.begin() + ti);
        continue;
      }
    }
    ti++;
  }
  return output;
}

int IoUTracker::total_count(const std::string &class_name) const
{
  auto it = total_counts_.find(class_name);
  return it == total_counts_.end() ? 0 : it->second;
}

void IoUTracker::print_counts_json() const
{
  /* {"counts":{"person":3,"car":5,"bus":1}} */
  printf("{\"event\":\"counts\",\"counts\":{");
  bool first = true;
  for (const auto &kv : total_counts_) {
    if (!first) printf(",");
    printf("\"%s\":%d", kv.first.c_str(), kv.second);
    first = false;
  }
  printf("}}\n");
}
