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


void checkArguments(int argc, char* argv[]) {
  std::string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " input output";

  bool has_valid_args = false;

  // make sure the user has provided input and output files
  if (argc == 1) {
    std::cerr << usage_instructions << std::endl;
  } else if (argc == 2) {
    std::cerr << "Please include an output file.\n" << usage_instructions
              << std::endl;
  } else if (argc == 3) {
    has_valid_args = true;
  } else if (argc > 3) {
    std::cerr << "Too many arguments.\n" << usage_instructions << std::endl;
  }

  if (!has_valid_args) exit(EXIT_FAILURE);
}

void checkFiles(std::ifstream& in_file, std::string& in_name,
                std::ofstream& out_file, std::string& out_name) {
  if (!in_file.is_open()) {
    std::cerr << "Cannot open input file: " << in_name << std::endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    std::cerr << "Cannot open output file: " << out_name << std::endl;
    exit(EXIT_FAILURE);
  }
}

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

  checkArguments(argc, argv);

  std::string in_file_name_ = argv[1];
  std::ifstream in_file_(in_file_name_.c_str(), std::ifstream::in);

  std::string out_file_name_ = argv[2];
  std::ofstream out_file_(out_file_name_.c_str(), std::ofstream::out);

  checkFiles(in_file_, in_file_name_, out_file_, out_file_name_);

  //
  // Set Measurements
  //
  std::vector<MeasurementPackage> m_hist;
  std::vector<GroundTruthPackage> gt_hist;

  std::string line;

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (getline(in_file_, line)) {
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
  out_file_ << "px" << "\t";
  out_file_ << "py" << "\t";
  out_file_ << "v" << "\t";
  out_file_ << "yaw" << "\t";
  out_file_ << "yaw_rate" << "\t";
  out_file_ << "NIS" << "\n";
  out_file_ << "px_m" << "\t";
  out_file_ << "py_m" << "\t";
  out_file_ << "px_gt" << "\t";
  out_file_ << "py_gt" << "\t";
  out_file_ << "vx_gt" << "\t";
  out_file_ << "vy_gt" << "\t";

  for (size_t k = 0; k < number_of_measurements; ++k) {
    // Call the UKF-based fusion
    ukf.processMeasurement(m_hist[k]);

    // output the estimation
    out_file_ << ukf.x_(0) << "\t"; // estimated x
    out_file_ << ukf.x_(1) << "\t"; // estimated y
    out_file_ << ukf.x_(2) << "\t"; // estimated v
    out_file_ << ukf.x_(3) << "\t"; // estimated yaw
    out_file_ << ukf.x_(4) << "\t"; // estimated yaw rate

    // output the measurements
    if (m_hist[k].sensor_type == MeasurementPackage::LIDAR) {
      // output the estimation

      out_file_ << m_hist[k].values(0) << "\t";
      out_file_ << m_hist[k].values(1) << "\t";
    } else if (m_hist[k].sensor_type == MeasurementPackage::RADAR) {
      // output the estimation in the cartesian coordinates
      double ro = m_hist[k].values(0);
      double phi = m_hist[k].values(1);
      out_file_ << ro * cos(phi) << "\t";
      out_file_ << ro * sin(phi) << "\t";
    }

    // output the ground truth packages
    out_file_ << gt_hist[k].values(0) << "\t";
    out_file_ << gt_hist[k].values(1) << "\t";
    out_file_ << gt_hist[k].values(2) << "\t";
    out_file_ << gt_hist[k].values(3) << "\t";

    // output the NIS values
    if (m_hist[k].sensor_type == MeasurementPackage::LIDAR) {
      out_file_ << ukf.nis_lidar_ << "\t" << "L" << "\n";
    } else if (m_hist[k].sensor_type == MeasurementPackage::RADAR) {
      out_file_ << ukf.nis_radar_ << "\t" << "R" << "\n";
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
  if (out_file_.is_open()) out_file_.close();

  if (in_file_.is_open()) in_file_.close();

  std::cout << "Done!" <<  std::endl;
  return 0;
}
