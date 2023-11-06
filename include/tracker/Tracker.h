#ifndef TRACKER_H_
#define TRACKER_H_

#include "tracker/Tracklet.h"
#include <limits>

class Tracker
{
public:
  Tracker();
  ~Tracker();


  int getPersonInside(){ return this->tracklet_inside.size(); };
  Eigen::VectorXd getCircleCenter(){ return this->circle_center; };
  float getRadius(){ return this->radius; };

  // handle tracklets
  void removeTracks();
  void addTracks(const std::vector<bool> &associated_detections,
                 const std::vector<double> &centroids_x,
                 const std::vector<double> &centroids_y);

  // associate tracklets and detections
  void dataAssociation(std::vector<bool> &associated_detections,
                       const std::vector<double> &centroids_x,
                       const std::vector<double> &centroids_y);

  // track objects
  void track(const std::vector<double> &centroids_x,
             const std::vector<double> &centroids_y,
             bool lidarStatus);

  // getters
  const std::vector<Tracklet> &getTracks() { return tracks_; }

private:
  // tracklets
  std::vector<Tracklet> tracks_;
  int cur_id_;

  // association
  std::vector<std::pair<int, int>> associated_track_det_ids_;

  // thresholds
  double distance_threshold_;
  double covariance_threshold;
  int loss_threshold;

  /*
    Vado a definire un centro e un raggio in questo modo rappresento l'area in cui entrare con un cerchio
  */

  Eigen::VectorXd circle_center;
  float radius;
  std::vector<int> tracklet_inside; //indico gli id dei tralket che sono entrati nell'area



};

#endif // TRACKER_H_
