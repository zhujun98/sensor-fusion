#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <Eigen/Dense>

#include "fusion_ekf.h"
#include "ground_truth_package.h"
#include "measurement_package.h"
#include "utilities.h"


void check_arguments(int argc, char* argv[]) {
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

  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}

void check_files(std::ifstream& in_file, std::string& in_name,
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

int main(int argc, char* argv[]) {

  check_arguments(argc, argv);

  std::string in_file_name_ = argv[1];
  std::ifstream in_file_(in_file_name_.c_str(), std::ifstream::in);

  std::string out_file_name_ = argv[2];
  std::ofstream out_file_(out_file_name_.c_str(), std::ofstream::out);

  check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  std::vector<MeasurementPackage> measurement_pack_list;
  std::vector<GroundTruthPackage> gt_pack_list;

  std::string line;

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  Utilities utilities;
  while (std::getline(in_file_, line)) {

    std::string sensor_type;
    MeasurementPackage meas_package;
    GroundTruthPackage gt_package;
    std::istringstream iss(line);
    long long timestamp;

    // reads first element from the current line
    iss >> sensor_type;

    if (sensor_type.compare("L") == 0) {
      // LIDAR MEASUREMENT
      meas_package.sensor_type_ = MeasurementPackage::LIDAR;

      double x;
      double y;
      iss >> x;
      iss >> y;
      meas_package.raw_measurements_ = Eigen::VectorXd(2);
      meas_package.raw_measurements_ << x, y;
    } else if (sensor_type.compare("R") == 0) {
      // RADAR MEASUREMENT
      meas_package.sensor_type_ = MeasurementPackage::RADAR;

      double rho;
      double phi;
      double v_rho;
      iss >> rho;
      iss >> phi;
      iss >> v_rho;

      // Normalize the angle to (-pi, pi]
      phi = utilities.normalize_angle(phi);
      meas_package.raw_measurements_ = Eigen::VectorXd(3);
      meas_package.raw_measurements_ << rho, phi, v_rho;
    } else {
      std::cerr << "Unknown sensor type: " << meas_package.sensor_type_
                << std::endl;
      exit(EXIT_FAILURE);
    }

    // read timestamp for both LIDAR and RADAR
    iss >> timestamp;
    meas_package.timestamp_ = timestamp;
    measurement_pack_list.push_back(meas_package);

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

  // Create a Fusion EKF instance
  FusionEKF fusion_ekf;

  // used to compute the RMSE later
  std::vector<Eigen::VectorXd> estimations;
  std::vector<Eigen::VectorXd> ground_truth;

  //Call the EKF-based fusion
  std::size_t N = measurement_pack_list.size();
  for (std::size_t k = 0; k < N; ++k) {
    // start filtering from the second frame (the speed is unknown in the first
    // frame)
    fusion_ekf.ProcessMeasurement(measurement_pack_list[k]);

    // output the estimation
    out_file_ << fusion_ekf.ekf_.x_(0) << "\t";
    out_file_ << fusion_ekf.ekf_.x_(1) << "\t";
    out_file_ << fusion_ekf.ekf_.x_(2) << "\t";
    out_file_ << fusion_ekf.ekf_.x_(3) << "\t";

    // output the measurements
    if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LIDAR) {
      // output the estimation
      out_file_ << measurement_pack_list[k].raw_measurements_(0) << "\t";
      out_file_ << measurement_pack_list[k].raw_measurements_(1) << "\t";
    } else if (measurement_pack_list[k].sensor_type_
               == MeasurementPackage::RADAR) {
      // output the estimation in the cartesian coordinates
      double rho = measurement_pack_list[k].raw_measurements_(0);
      double phi = measurement_pack_list[k].raw_measurements_(1);
      out_file_ << rho * std::cos(phi) << "\t"; // p1_meas
      out_file_ << rho * std::sin(phi) << "\t"; // ps_meas
    }

    // output the ground truth packages
    out_file_ << gt_pack_list[k].gt_values_(0) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(1) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(2) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(3) << "\n";

    estimations.push_back(fusion_ekf.ekf_.x_);
    ground_truth.push_back(gt_pack_list[k].gt_values_);
  }

  // compute the accuracy (RMSE)
  std::cout << "Estimation accuracy - RMSE:" << std::endl
            << utilities.CalculateRMSE(estimations, ground_truth) << std::endl;

  // close files
  if (out_file_.is_open()) {
    out_file_.close();
  }

  if (in_file_.is_open()) {
    in_file_.close();
  }

  return 0;
}
