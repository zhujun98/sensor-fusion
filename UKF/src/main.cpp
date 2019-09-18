/*
 * Author: Jun Zhu, zhujun981661@gmail.com
 */

#include <fstream>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "ukf.h"
#include "ground_truth_package.h"
#include "measurement_package.h"


//
// Calculate root mean square error.
//
Eigen::VectorXd calculateRMSE(
    const std::vector<Eigen::VectorXd> &estimations,
    const std::vector<Eigen::VectorXd> &ground_truth) {

  Eigen::VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  for (std::size_t i=0; i != estimations.size(); ++i) {
    Eigen::VectorXd residual = estimations[i] - ground_truth[i];

    residual = residual.array()*residual.array();

    rmse += residual;
  }

  rmse /= estimations.size();

  rmse = rmse.array().sqrt();

  return rmse;
}

int main(int argc, char* argv[]) {

  std::string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " input output";

  if (argc != 3) {
    std::cerr << usage_instructions << std::endl;
    exit(EXIT_FAILURE);
  }

  std::ifstream ifs(std::string(argv[1]).c_str(), std::ifstream::in);
  std::ofstream ofs(std::string(argv[2]).c_str(), std::ofstream::out);

  if (!ifs.is_open()) {
    std::cerr << "Failed to open input file!" << std::endl;
    exit(EXIT_FAILURE);
  }

  if (!ofs.is_open()) {
    std::cerr << "Failed to open output file!" << std::endl;
    exit(EXIT_FAILURE);
  }

  //
  // Set Measurements
  //
  std::vector<MeasurementPackage> m_hist;
  std::vector<GroundTruthPackage> gt_hist;

  std::string line;

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (getline(ifs, line)) {
    std::string sensor_type;
    MeasurementPackage m_pkg;
    GroundTruthPackage gt_pkg;
    std::istringstream iss(line);
    long long timestamp;

    // reads first element from the current line
    iss >> sensor_type;

    if (sensor_type == "L") {
      // laser measurement

      // read measurements at this timestamp
      m_pkg.sensor_type = MeasurementPackage::LIDAR;
      m_pkg.values = Eigen::VectorXd(2);
      double px;
      double py;
      iss >> px;
      iss >> py;
      m_pkg.values << px, py;
      iss >> timestamp;
      m_pkg.timestamp = timestamp;
      m_hist.push_back(m_pkg);
    } else if (sensor_type == "R") {
      // radar measurement

      // read measurements at this timestamp
      m_pkg.sensor_type = MeasurementPackage::RADAR;
      m_pkg.values = Eigen::VectorXd(3);
      double ro;
      double phi;
      double ro_dot;
      iss >> ro;
      iss >> phi;
      iss >> ro_dot;
      m_pkg.values << ro, phi, ro_dot;
      iss >> timestamp;
      m_pkg.timestamp = timestamp;
      m_hist.push_back(m_pkg);
    } else {
      std::cerr << "Unknown sensor type: " << m_pkg.sensor_type << std::endl;
      exit(EXIT_FAILURE);
    }

    // read ground truth data to compare later
    double x_gt;
    double y_gt;
    double vx_gt;
    double vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;
    gt_pkg.values = Eigen::VectorXd(4);
    gt_pkg.values << x_gt, y_gt, vx_gt, vy_gt;
    gt_hist.push_back(gt_pkg);
  }

  // Create a UKF instance
  UKF ukf;

  // used to compute the RMSE later
  std::vector<Eigen::VectorXd> estimations;
  std::vector<Eigen::VectorXd> ground_truth;

  // start filtering from the second frame (the speed is unknown in the first frame)

  size_t number_of_measurements = m_hist.size();

  // column names for output file
  ofs << "px" << "\t";
  ofs << "py" << "\t";
  ofs << "v" << "\t";
  ofs << "yaw" << "\t";
  ofs << "yaw_rate" << "\t";
  ofs << "NIS" << "\n";
  ofs << "px_m" << "\t";
  ofs << "py_m" << "\t";
  ofs << "px_gt" << "\t";
  ofs << "py_gt" << "\t";
  ofs << "vx_gt" << "\t";
  ofs << "vy_gt" << "\t";

  for (size_t k = 0; k < number_of_measurements; ++k) {
    // Call the UKF-based fusion
    ukf.processMeasurement(m_hist[k]);

    // output the estimation
    ofs << ukf.x_(0) << "\t"; // estimated x
    ofs << ukf.x_(1) << "\t"; // estimated y
    ofs << ukf.x_(2) << "\t"; // estimated v
    ofs << ukf.x_(3) << "\t"; // estimated yaw
    ofs << ukf.x_(4) << "\t"; // estimated yaw rate

    // output the measurements
    if (m_hist[k].sensor_type == MeasurementPackage::LIDAR) {
      // output the estimation

      ofs << m_hist[k].values(0) << "\t";
      ofs << m_hist[k].values(1) << "\t";
    } else if (m_hist[k].sensor_type == MeasurementPackage::RADAR) {
      // output the estimation in the cartesian coordinates
      double ro = m_hist[k].values(0);
      double phi = m_hist[k].values(1);
      ofs << ro * cos(phi) << "\t";
      ofs << ro * sin(phi) << "\t";
    }

    // output the ground truth packages
    ofs << gt_hist[k].values(0) << "\t";
    ofs << gt_hist[k].values(1) << "\t";
    ofs << gt_hist[k].values(2) << "\t";
    ofs << gt_hist[k].values(3) << "\t";

    // output the NIS values
    if (m_hist[k].sensor_type == MeasurementPackage::LIDAR) {
      ofs << ukf.nis_lidar_ << "\t" << "L" << "\n";
    } else if (m_hist[k].sensor_type == MeasurementPackage::RADAR) {
      ofs << ukf.nis_radar_ << "\t" << "R" << "\n";
    }

    // convert ukf x vector to cartesian to compare to ground truth
    Eigen::VectorXd ukf_x_cartesian_ = Eigen::VectorXd(4);

    double x_estimate_ = ukf.x_(0);
    double y_estimate_ = ukf.x_(1);
    double vx_estimate_ = ukf.x_(2) * std::cos(ukf.x_(3));
    double vy_estimate_ = ukf.x_(2) * std::sin(ukf.x_(3));
    
    ukf_x_cartesian_ << x_estimate_, y_estimate_, vx_estimate_, vy_estimate_;
    
    estimations.push_back(ukf_x_cartesian_);
    ground_truth.push_back(gt_hist[k].values);
  }

  // compute the accuracy (RMSE)
  std::cout << "Accuracy - RMSE:\n" << calculateRMSE(estimations, ground_truth) << std::endl;

  // close files
  if (ofs.is_open()) ofs.close();
  if (ifs.is_open()) ifs.close();

  return 0;
}
