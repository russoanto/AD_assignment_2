#include "tracker/Tracklet.h"

Tracklet::Tracklet(int idTrack, double x, double y)
{
  // set id
  id_ = idTrack;

  // initialize filter
  kf_.init(0.1);
  kf_.setState(x, y);

  // set loss count to 0
  loss_count_ = 0;

  meters_covered = 0.0;
}

Tracklet::~Tracklet()
{
}

// Predict a single measurement
void Tracklet::predict()
{
  kf_.predict();
  loss_count_++;
}

void Tracklet::updateMeters(double x, double y){
  this->meters_covered += sqrt(pow(getX() - x, 2) + pow(getY() - y, 2));
} 

// Update with a real measurement
void Tracklet::update(double x, double y, bool lidarStatus)
{
  Eigen::VectorXd raw_measurements_ = Eigen::VectorXd(2);
  // measurement update
  if (lidarStatus){
    raw_measurements_ << x, y;
    kf_.update(raw_measurements_);
    loss_count_ = 0;
    updateMeters(x,y);
  }

}
