#ifndef TRACKLET_H_
#define TRACKLET_H_

#include <vector>
#include <cmath>
#include <iostream>

#include "KalmanFilter.h"

class Tracklet
{
public:
  Tracklet(int idTrack, double x, double y);
  ~Tracklet();

  void predict();
  void update(double x, double y, bool lidarStatus);
  void updateMeters(double x, double y);

  // getters
  double getX() { return kf_.getX(); }
  double getY() { return kf_.getY(); }
  double getXCovariance() { return kf_.getXCovariance(); }
  double getYCovariance() { return kf_.getYCovariance(); }
  double getMeters() { return meters_covered; }

  int getLossCount() { return loss_count_; }
  int getId() { return id_; }


private:
  // tracklet id
  int id_;

  // number of loss since last update
  int loss_count_;

  // number of meters covered by the traklet
  double meters_covered;

  // filter
  KalmanFilter kf_;
  
};

#endif // TRACKLET_H_
