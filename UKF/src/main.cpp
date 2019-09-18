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

  Eigen::RowVector4d rmse {0, 0, 0, 0};

  for (std::size_t i=0; i != estimations.size(); ++i) {
    Eigen::VectorXd residual = estimations[i] - ground_truth[i];

    residual = residual.array() * residual.array();

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

  std::vector<MeasurementPackage> m_hist;
  std::vector<GroundTruthPackage> gt_hist;

  std::string line;

  // prepare the measurement packages (each line represents a measurement
  // at a timestamp)

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

  // process data

  UKF ukf;

  std::vector<Eigen::VectorXd> ret; // estimated values
  std::vector<Eigen::VectorXd> gt; // ground truth values

  // column names in the output file
  ofs << "sensor_type\tpx\tpy\tv\tyaw\tyaw_rate\tNIS\tpx_m\tpy_m\tpx_gt\tpy_gt\tvx_gt\tvy_gt\ttimestamp\n";

  for (size_t k = 0; k < m_hist.size(); ++k) {

    ukf.processMeasurement(m_hist[k]);

    // dump the sensor type
    if (m_hist[k].sensor_type == MeasurementPackage::LIDAR) {
      ofs << "L\t";
    } else if (m_hist[k].sensor_type == MeasurementPackage::RADAR) {
      ofs << "R\t";
    }

    // dump the estimations
    ofs << ukf.x_(0) << "\t"; // estimated x
    ofs << ukf.x_(1) << "\t"; // estimated y
    ofs << ukf.x_(2) << "\t"; // estimated v
    ofs << ukf.x_(3) << "\t"; // estimated yaw
    ofs << ukf.x_(4) << "\t"; // estimated yaw rate

    // dump the NIS values
    if (m_hist[k].sensor_type == MeasurementPackage::LIDAR) {
      ofs << ukf.nis_lidar_ << "\t";
    } else if (m_hist[k].sensor_type == MeasurementPackage::RADAR) {
      ofs << ukf.nis_radar_ << "\t";
    }

    // dump the measurements
    if (m_hist[k].sensor_type == MeasurementPackage::LIDAR) {
      ofs << m_hist[k].values(0) << "\t";
      ofs << m_hist[k].values(1) << "\t";
    } else if (m_hist[k].sensor_type == MeasurementPackage::RADAR) {
      double ro = m_hist[k].values(0);
      double phi = m_hist[k].values(1);
      ofs << ro * cos(phi) << "\t";
      ofs << ro * sin(phi) << "\t";
    }

    // dump the ground truth values
    ofs << gt_hist[k].values(0) << "\t";
    ofs << gt_hist[k].values(1) << "\t";
    ofs << gt_hist[k].values(2) << "\t";
    ofs << gt_hist[k].values(3) << "\t";

    ofs << m_hist[k].timestamp << "\n";

    // convert ukf x vector to cartesian so as to compare with ground truth
    Eigen::VectorXd x_cartesian = Eigen::VectorXd(4);
    x_cartesian << ukf.x_(0),
                   ukf.x_(1),
                   ukf.x_(2) * std::cos(ukf.x_(3)),
                   ukf.x_(2) * std::sin(ukf.x_(3));
    
    ret.push_back(x_cartesian);
    gt.push_back(gt_hist[k].values);
  }

  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols,
                               ", ", ", ", "", "", " << ", "");
  std::cout << "RMSE" << calculateRMSE(ret, gt).format(CommaInitFmt) << std::endl;

  // close files
  if (ofs.is_open()) ofs.close();
  if (ifs.is_open()) ifs.close();

  return 0;
}
