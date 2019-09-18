/*
 * Author: Jun Zhu, zhujun981661@gmail.com
 */

#include <fstream>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "fusion_ekf.h"
#include "ground_truth_package.h"
#include "measurement_package.h"
#include "utilities.h"


//
// Calculate root mean square error.
//
inline Eigen::VectorXd calculateRMSE(const std::vector<Eigen::VectorXd>& estimations,
                                     const std::vector<Eigen::VectorXd>& ground_truth) {
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

  std::vector<MeasurementPackage> m_hist;
  std::vector<GroundTruthPackage> gt_hist;

  std::string line;

  // prepare the measurement packages (each line represents a measurement
  // at a timestamp)

  while (std::getline(ifs, line)) {

    std::string sensor_type;
    MeasurementPackage m_pkg;
    GroundTruthPackage gt_pkg;
    std::istringstream iss(line);
    long long timestamp;

    // reads first element from the current line
    iss >> sensor_type;

    if (sensor_type == "L") {
      // LIDAR measurement
      m_pkg.sensor_type = MeasurementPackage::LIDAR;

      double x;
      double y;
      iss >> x;
      iss >> y;
      m_pkg.values = Eigen::VectorXd(2);
      m_pkg.values << x, y;
    } else if (sensor_type == "R") {
      // RADAR measurement
      m_pkg.sensor_type = MeasurementPackage::RADAR;

      double rho;
      double phi;
      double v_rho;
      iss >> rho;
      iss >> phi;
      iss >> v_rho;

      // Normalize the angle to (-pi, pi]
      phi = utilities::normalizeAngle(phi);
      m_pkg.values = Eigen::VectorXd(3);
      m_pkg.values << rho, phi, v_rho;
    } else {
      std::cerr << "Unknown sensor type: " << m_pkg.sensor_type << std::endl;
      exit(EXIT_FAILURE);
    }

    // read the timestamp for both LIDAR and RADAR
    iss >> timestamp;
    m_pkg.timestamp = timestamp;
    m_hist.push_back(m_pkg);

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

  FusionEKF fusion_ekf;

  std::vector<Eigen::VectorXd> ret; // estimated values
  std::vector<Eigen::VectorXd> gt; // ground truth values

  // column names in the output file
  ofs <<"sensor_type\tpx\tpy\tvx\tvy\tpx_m\tpy_m\tpx_gt\tpy_gt\tvx_gt\tvy_gt\ttimestamp\n";

  for (std::size_t k = 0; k < m_hist.size(); ++k) {

    fusion_ekf.processMeasurement(m_hist[k]);

    // dump the sensor type
    if (m_hist[k].sensor_type == MeasurementPackage::LIDAR) {
      ofs << "L\t";
    } else if (m_hist[k].sensor_type == MeasurementPackage::RADAR) {
      ofs << "R\t";
    }

    // dump the estimations
    ofs << fusion_ekf.ekf_.x_(0) << "\t";
    ofs << fusion_ekf.ekf_.x_(1) << "\t";
    ofs << fusion_ekf.ekf_.x_(2) << "\t";
    ofs << fusion_ekf.ekf_.x_(3) << "\t";

    // dump the measurements
    if (m_hist[k].sensor_type == MeasurementPackage::LIDAR) {
      // output the estimation
      ofs << m_hist[k].values(0) << "\t";
      ofs << m_hist[k].values(1) << "\t";
    } else if (m_hist[k].sensor_type == MeasurementPackage::RADAR) {
      // output the estimation in the cartesian coordinates
      double rho = m_hist[k].values(0);
      double phi = m_hist[k].values(1);
      ofs << rho * std::cos(phi) << "\t"; // p1_meas
      ofs << rho * std::sin(phi) << "\t"; // ps_meas
    }

    // dump the ground truth values
    ofs << gt_hist[k].values(0) << "\t";
    ofs << gt_hist[k].values(1) << "\t";
    ofs << gt_hist[k].values(2) << "\t";
    ofs << gt_hist[k].values(3) << "\t";

    // dump the timestamp
    ofs << m_hist[k].timestamp << "\n";

    ret.push_back(fusion_ekf.ekf_.x_);
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
