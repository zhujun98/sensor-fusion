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
#include "utilities.h"


void checkArguments(int argc, char* argv[]) {
  std::string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

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
  std::vector<MeasurementPackage> measurement_pack_list;
  std::vector<GroundTruthPackage> gt_pack_list;

  std::string line;

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (getline(in_file_, line)) {
    std::string sensor_type;
    MeasurementPackage meas_package;
    GroundTruthPackage gt_package;
    std::istringstream iss(line);
    long long timestamp;

    // reads first element from the current line
    iss >> sensor_type;

    if (sensor_type == "L") {
      // laser measurement

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::LIDAR;
      meas_package.raw_measurements_ = Eigen::VectorXd(2);
      double px;
      double py;
      iss >> px;
      iss >> py;
      meas_package.raw_measurements_ << px, py;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    } else if (sensor_type == "R") {
      // radar measurement

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = Eigen::VectorXd(3);
      double ro;
      double phi;
      double ro_dot;
      iss >> ro;
      iss >> phi;
      iss >> ro_dot;
      meas_package.raw_measurements_ << ro, phi, ro_dot;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    } else {
      std::cerr << "Unknown sensor type: " << meas_package.sensor_type_ << std::endl;
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
    gt_package.gt_values_ = Eigen::VectorXd(4);
    gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
    gt_pack_list.push_back(gt_package);
  }

  // Create a UKF instance
  UKF ukf;

  // used to compute the RMSE later
  std::vector<Eigen::VectorXd> estimations;
  std::vector<Eigen::VectorXd> ground_truth;

  // start filtering from the second frame (the speed is unknown in the first
  // frame)

  size_t number_of_measurements = measurement_pack_list.size();

  // column names for output file
  out_file_ << "px" << "\t";
  out_file_ << "py" << "\t";
  out_file_ << "v" << "\t";
  out_file_ << "yaw_angle" << "\t";
  out_file_ << "yaw_rate" << "\t";
  out_file_ << "px_measured" << "\t";
  out_file_ << "py_measured" << "\t";
  out_file_ << "px_true" << "\t";
  out_file_ << "py_true" << "\t";
  out_file_ << "vx_true" << "\t";
  out_file_ << "vy_true" << "\t";
  out_file_ << "NIS" << "\n";

  for (size_t k = 0; k < number_of_measurements; ++k) {
    // Call the UKF-based fusion
    ukf.processMeasurement(measurement_pack_list[k]);

    // output the estimation
    out_file_ << ukf.x_(0) << "\t"; // pos1 - est
    out_file_ << ukf.x_(1) << "\t"; // pos2 - est
    out_file_ << ukf.x_(2) << "\t"; // vel_abs -est
    out_file_ << ukf.x_(3) << "\t"; // yaw_angle -est
    out_file_ << ukf.x_(4) << "\t"; // yaw_rate -est

    // output the measurements
    if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LIDAR) {
      // output the estimation

      out_file_ << measurement_pack_list[k].raw_measurements_(0) << "\t";
      out_file_ << measurement_pack_list[k].raw_measurements_(1) << "\t";
    } else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
      // output the estimation in the cartesian coordinates
      double ro = measurement_pack_list[k].raw_measurements_(0);
      double phi = measurement_pack_list[k].raw_measurements_(1);
      out_file_ << ro * cos(phi) << "\t";
      out_file_ << ro * sin(phi) << "\t";
    }

    // output the ground truth packages
    out_file_ << gt_pack_list[k].gt_values_(0) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(1) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(2) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(3) << "\t";

    // output the NIS values
    if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LIDAR) {
      out_file_ << ukf.NIS_lidar_ << "\t" << "L" << "\n";
    } else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
      out_file_ << ukf.NIS_radar_ << "\t" << "R" << "\n";
    }

    // convert ukf x vector to cartesian to compare to ground truth
    Eigen::VectorXd ukf_x_cartesian_ = Eigen::VectorXd(4);

    double x_estimate_ = ukf.x_(0);
    double y_estimate_ = ukf.x_(1);
    double vx_estimate_ = ukf.x_(2) * std::cos(ukf.x_(3));
    double vy_estimate_ = ukf.x_(2) * std::sin(ukf.x_(3));
    
    ukf_x_cartesian_ << x_estimate_, y_estimate_, vx_estimate_, vy_estimate_;
    
    estimations.push_back(ukf_x_cartesian_);
    ground_truth.push_back(gt_pack_list[k].gt_values_);
  }

  // compute the accuracy (RMSE)
  std::cout << "Accuracy - RMSE:" << std::endl
            << calculateRMSE(estimations, ground_truth) << std::endl;

  // close files
  if (out_file_.is_open()) out_file_.close();

  if (in_file_.is_open()) in_file_.close();

  std::cout << "Done!" <<  std::endl;
  return 0;
}
