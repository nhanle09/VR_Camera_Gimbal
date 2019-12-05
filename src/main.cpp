#include <thread>

#include <boost/asio.hpp>

#include "VideoStream.h"
#include "OculusTracker.h"

int main(int argc, char** argv)
{
	ovrSession session;

	OculusTracker oculusTracker(session);
	VideoStream videoStream(session, oculusTracker);

	std::thread OCULUS_TRACKING(&OculusTracker::Execute, &oculusTracker);
	std::thread VIDEO_STREAM(&VideoStream::Execute, &videoStream);

	VIDEO_STREAM.join();
	OCULUS_TRACKING.join();

	return 0;
}