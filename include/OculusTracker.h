#pragma once

#include <boost/asio.hpp>

#include <iostream>
#include <OVR_CAPI.h>
#include <Extras/OVR_Math.h>
#include <sl/Camera.hpp>

#define PORT "COM3"

boost::asio::serial_port_base::baud_rate BAUD(115200);
boost::asio::serial_port_base::flow_control FLOW(boost::asio::serial_port_base::flow_control::none);
boost::asio::serial_port_base::parity PARITY(boost::asio::serial_port_base::parity::none);
boost::asio::serial_port_base::stop_bits STOP(boost::asio::serial_port_base::stop_bits::one);

class OculusTracker
{
	private:
		ovrSession& session;

	public:
		bool ended = false;
		
		OculusTracker(ovrSession& _session) : session(_session) {}
		
		void Execute()
		{
			boost::asio::io_service io;
			boost::asio::serial_port port(io, PORT);

			port.set_option(BAUD);
			port.set_option(FLOW);
			port.set_option(PARITY);
			port.set_option(STOP);

			printf("Waiting for MCU Boot\n");
			
			sl::sleep_ms(3000);

			printf("Streaming Oculus orientation data\n");

			while (!ended)
			{
				ovrTrackingState tracking_state = ovr_GetTrackingState(session, ovr_GetTimeInSeconds(), ovrTrue);
				if (tracking_state.StatusFlags & (ovrStatus_OrientationTracked | ovrStatus_PositionTracked))
				{
					//ovrPosef pose = tracking_state.HeadPose.ThePose;
					float x = tracking_state.HeadPose.ThePose.Orientation.x * 180.0;
					float y = tracking_state.HeadPose.ThePose.Orientation.y * 180.0;
					float z = tracking_state.HeadPose.ThePose.Orientation.z * 180.0;

					std::string toESP = std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) + "\r";
					boost::asio::write(port, boost::asio::buffer(toESP.c_str(),toESP.size()));

					printf("X = %5.2f | Y = %5.2f | Z = %5.2f\r", x, y, z);
				}
				sl::sleep_ms(50);
			}

			//port.close();
			exit(0);
		}
};