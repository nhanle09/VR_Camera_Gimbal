#pragma once

#include <sl/Camera.hpp>

class CameraTracker
{
	private:
		sl::Camera				camera;
		sl::InitParameters		initial_parameters;
		sl::TrackingParameters	tracking_parameters;
		sl::IMUData				imu_data;
		sl::Pose				pose;


	public:
		CameraTracker()
		{
			initial_parameters.camera_resolution = sl::RESOLUTION_HD720;
			initial_parameters.coordinate_units  = sl::UNIT_METER;
			initial_parameters.coordinate_system = sl::COORDINATE_SYSTEM_LEFT_HANDED_Y_UP;
			initial_parameters.depth_mode = sl::DEPTH_MODE_NONE;
		
			sl::ERROR_CODE err = camera.open(initial_parameters);
			if (err != sl::SUCCESS)
			{
				camera.close();
				std::cout << err << std::endl;
				exit(-1);
			}

			//tracking_parameters.enable_imu_fusion = true;
			//tracking_parameters.enable_pose_smoothing = true;
			////tracking_parameters.enable_spatial_memory = true;
			////tracking_parameters.set_floor_as_origin = true;
			
			err = camera.enableTracking();
			//err = camera.enableTracking(tracking_parameters);
			//if (err != sl::SUCCESS)
			//{
			//	camera.close();
			//	std::cout << err << std::endl;
			//	exit(-1);
			//}
		}
		
		bool Home()
		{
			bool homed = false;
			while (!homed)
			{
				if (camera.grab() == sl::SUCCESS)
				{
					//sl::ERROR_CODE imu = camera.getIMUData(imu_data, sl::TIME_REFERENCE_CURRENT);
					sl::TRACKING_STATE tracking = camera.getPosition(pose, sl::REFERENCE_FRAME_WORLD);
					
					if (tracking == sl::TRACKING_STATE_OK)
					{
						sl::float3 rotation = pose.getEulerAngles(false);
						printf("Rotation: X = %5.2f | Y = %5.2f | Z = %5.2f\r", rotation.x, rotation.y, rotation.z);
					}
				}
				sl::sleep_ms(1);
			}

			camera.close();
			return true;
		}
};