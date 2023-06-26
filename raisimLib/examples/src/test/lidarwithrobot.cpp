// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

int main(int argc, char **argv) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  const int loopN = 200000000;
  raisim::RaiSimMsg::setFatalCallback([](){throw;});

  raisim::World world;
  raisim::RaisimServer server(&world);

  auto checkerBoard = world.addGround(0.0, "glass");

  Eigen::VectorXd jointConfig(19), jointVelocityTarget(18);
  Eigen::VectorXd jointState(18), jointVel(18), jointPgain(18), jointDgain(18);

  jointPgain.setZero();
  jointPgain.tail(12).setConstant(200.0);

  jointDgain.setZero();
  jointDgain.tail(12).setConstant(10.0);

  jointVelocityTarget.setZero();

  jointConfig << 0, 0, 0.54, 1, 0, 0, 0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8,
      0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  jointVel << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  auto anymal = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\anymal_c\\urdf\\anymal_sensored.urdf");
  anymal->setState(jointConfig, jointVel);
  anymal->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  anymal->setPdGains(jointPgain, jointDgain);
  anymal->setPdTarget(jointConfig, jointVelocityTarget);
  anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));
  anymal->setName("Anymal");

  auto depthSensor1 = anymal->getSensor<raisim::DepthCamera>("depth_camera_front_camera_parent:depth");
  depthSensor1->setMeasurementSource(raisim::Sensor::MeasurementSource::VISUALIZER);
  auto rgbCamera1 = anymal->getSensor<raisim::RGBCamera>("depth_camera_front_camera_parent:color");
  rgbCamera1->setMeasurementSource(raisim::Sensor::MeasurementSource::VISUALIZER);

  auto depthSensor2 = anymal->getSensor<raisim::DepthCamera>("depth_camera_rear_camera_parent:depth");
  depthSensor2->setMeasurementSource(raisim::Sensor::MeasurementSource::VISUALIZER);
  auto rgbCamera2 = anymal->getSensor<raisim::RGBCamera>("depth_camera_rear_camera_parent:color");
  rgbCamera2->setMeasurementSource(raisim::Sensor::MeasurementSource::VISUALIZER);
  auto imu = anymal->getSensor<raisim::InertialMeasurementUnit>("depth_camera_front_camera_parent:imu");
	

  /* start added part for using lidar */

  //for make lidar's laser box
	auto scans = server.addInstancedVisuals("scan points",
                                          raisim::Shape::Box,
                                          {0.05, 0.05, 0.05},
                                          {1,0,0,1},
                                          {0,1,0,1});
	int scanSize1 = 1; //2d lidar (one line)
  int scanSize2 = 50; //make 50 detective box
	scans->resize(scanSize1*scanSize2);
	Eigen::Vector3d direction;

	

  server.launchServer();
	server.focusOn(anymal);
  for (int k = 0; k < loopN; k++) {
    RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
    server.integrateWorldThreadSafe();
		raisim::Vec<3> lidarPos; raisim::Mat<3,3> lidarOri; 
		anymal->getFramePosition("lidar_joint", lidarPos); // data getting from example/rsc/anymal_c/urdf/anymal_sensored.urdf
		anymal->getFrameOrientation("lidar_joint", lidarOri); // data getting from example/rsc/anymal_c/urdf/anymal_sensored.urdf
		std::vector<std::vector<double>> lidarpoints; // for lidardata output
		
    for (int j = 0; j < scanSize2; j++) {
			const double yaw = j * M_PI / scanSize2 * 0.6 - 0.175 * M_PI; //j_th angle(- 0.175 * M_PI : for align yaw axis)
			double pitch = 0;
			const double normInv = 1. / sqrt(pitch * pitch + 1); // 1
			direction = {cos(yaw) * normInv, sin(yaw) * normInv, -pitch * normInv}; // lidar's lazer's unit direction vector ( lidar frame of reference )
			Eigen::Vector3d rayDirection;
			//lidarOri.e() : rotation matrix of lidar, reference frame is spatial frame (maybe)
			//lidarPos.e() : position vector of lidar, reference frame is spatial frame
			rayDirection = lidarOri.e() * direction; // change reference frame body frame to world frame
			
			auto &col = world.rayTest(lidarPos.e(), rayDirection, 30); // detect collision between lazerbox and world

			if (col.size() > 0) { // if laser detects an object ( get collision )
				scans->setPosition(j, col[0].getPosition()); // make laser box's position to collision point
				float length = (col[0].getPosition() - lidarPos.e()).norm(); // length of lidar's laser, use for determine lazer's color
			
				auto lidarpoint = col[0].getPosition() - lidarPos.e(); // Relative position of collided laser with lidar
				double pointx = lidarpoint[0]; // relative x position
				double pointy = lidarpoint[1]; // relative y position
				double pointz = lidarpoint[2]; // relative z position
				std::vector<double> lidarpointvec = {pointx, pointy, pointz};
				lidarpoints.push_back(lidarpointvec);
				
				scans->setColorWeight(j, std::min(length/15.f, 1.0f)); // color
			}
			else
				scans->setPosition(j, {0, 0, 100});
		}
		for (int i = 0; i < lidarpoints.size(); i++)
		{
			for(auto loop : lidarpoints[i])
				std::cout << loop << ", ";
			std::cout << "\n";
		}
		std::cout << "num of points:"<< lidarpoints.size() << "\n" << std::endl;
  }

  server.killServer();
  return 0;
}