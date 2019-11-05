/*
 * ukf.h
 *
 *  Created on: Nov 3, 2019
 *      Author: sarah
 */

#ifndef SRC_IMAGE_PROCESS_SRC_UKF_H_
#define SRC_IMAGE_PROCESS_SRC_UKF_H_

// Includes
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

#include "track_type.h"

// Namespaces
namespace tracking{

using namespace Eigen;

struct Parameter{

  float da_ped_dist_pos;
  float da_ped_dist_form;
  float da_car_dist_pos;
  float da_car_dist_form;

  int tra_dim_z;
  int tra_dim_x;
  int tra_dim_x_aug;

  float tra_std_lidar_x;
  float tra_std_lidar_y;
  float tra_std_acc;
  float tra_std_yaw_rate;
  float tra_lambda;

  int tra_n_init;// How many match times will think be confirmed
  int tra_max_age;// At most how many missed times(means no matches) will be allowed

  float tra_occ_factor;

  float p_init_x;
  float p_init_y;
  float p_init_v;
  float p_init_yaw;
  float p_init_yaw_rate;
};

struct History{

  int age; // Aready tracked how many times, no matter wheter missed or not
  int hits;// Count update (match) times
  int time_since_update; // How many times since last time update
  visualization_msgs::Marker trajectory;
};

struct Geometry{

  float width;
  float length;
  float height;
  float orientation;
};

struct Semantic{

  int id;
  std::string name;
  float confidence;
};

struct State{

  VectorXd x;
  // Original world posisition from detection module
  geometry_msgs::PointStamped world_origin_pos;
  float z;
  MatrixXd P;
  VectorXd x_aug;
  VectorXd P_aug;
  MatrixXd Xsig_pred;
};

struct Track{

  // Attributes
  int id;
  State sta;
  Geometry geo;
  Semantic sem;
  History hist;
  int r;
  int g;
  int b;
};


class UnscentedKF;
typedef std::shared_ptr<UnscentedKF> UnscentedKFTracker;

enum class TrackState
{
  TS_NONE = 0,
  Tentative,
  Confirmed,
  Deleted
};

class UnscentedKF
{
public:
  // Default constructor
  UnscentedKF();

  // Virtual destructor
  virtual ~UnscentedKF();


  void mark_missed(){
    if(state_ == TrackState::Tentative){
      state_ = TrackState::Deleted;
    }
    else if(track_.hist.time_since_update >= params_.tra_max_age)
      state_ = TrackState::Deleted;
  }

  void markDeleted(){
    state_ = TrackState::Deleted;
  }

  void markTentative(){
    state_ = TrackState::Tentative;
  }

  void markConfirmed(){
    state_ = TrackState::Confirmed;
  }

  bool is_deleted(){
    return state_==TrackState::Deleted;
  }

  bool is_tentative(){
    return state_ == TrackState::Tentative;
  }

  bool is_confirmed()const {
    return state_ == TrackState::Confirmed;
  }

  // Class functions
  void initialize(long long id, const sensors_fusion::ObjectTrack& obj, double timestamp);
  void Prediction(const double delta_t);
  void predict(const double& time_stamp);
  /**!
   * @brief Update UKF state by using measurement
   * @param detected_objects
   */
  void update(const sensors_fusion::ObjectTrack& detected_objects);

  void reset();

  // Object track
  Track track_;

  // Whether a valid 3D track object flag,
  // if is from image tracking object, is_valid is false, false by default
  bool is_valid_;

protected:

  // Class member
  Parameter params_;
private:

  // Visualization
  cv::RNG rng_;

  // UKF
  MatrixXd R_laser_;
  VectorXd weights_;
  //    std::vector<Track> tracks_;

  // Prediction
  double last_time_stamp_;

  TrackState state_;
};

} // namespace tracking



#endif /* SRC_IMAGE_PROCESS_SRC_UKF_H_ */
