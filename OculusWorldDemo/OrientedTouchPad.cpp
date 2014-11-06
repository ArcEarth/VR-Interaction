#pragma comment(lib,"libboost_system-vc120-mt-gd-1_56.lib")

#include "OrientedTouchPad.h"
#include <ctime>
#include <iostream>
#include <string>
#include <thread>
#include "Util\Filter.h"
#include "Common.hpp"
#ifndef _WIN32_WINNT
#define _WIN32_WINNT _WIN32_WINNT_WINBLUE 
#endif

#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/signals2.hpp>

using boost::asio::ip::udp;

using namespace Platform::Input;

std::string make_daytime_string()
{
	using namespace std; // For time_t, time and ctime;
	time_t now = time(0);
	return ctime(&now);
}

struct OrientedTouchPad::OrientatedTablet
{
public:
	static const unsigned short DefaultPort = 13000;

	struct OrientationEventPackage
	{
		// Ticks in UTC
		int64_t TimeStamp;
		union
		{
			float	Quat[4];
			struct
			{
				float X, Y, Z, W;
			};
		};
	};

	OrientatedTablet(unsigned short port = DefaultPort)
		: socket_(io_service_, udp::endpoint(udp::v4(), port))
	{
		writable_package_no = 0;
		have_new_data = false;
		start_receive();
	}

	void Start()
	{
		running_thread_ = std::thread([this]{
			io_service_.run();
		});
	}

	void Stop()
	{
		if (!io_service_.stopped())
			io_service_.stop();
		running_thread_.join();
	}

	bool HaveNewData() const
	{
		return have_new_data;
	}

	void SwapBuffer()
	{
		writable_package_no = 1 - writable_package_no; // Flip the double buffer
		have_new_data = false;
	}

	Platform::Event<OrientationEventPackage*> OrientationChanged;


	OrientationEventPackage CurrentOrientation() const
	{
		return packages[1 - writable_package_no];
	}
private:
	void start_receive()
	{
		socket_.async_receive(
			boost::asio::buffer(recv_buffer_),
			boost::bind(&OrientatedTablet::handle_receive, this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));
	}

	void handle_receive(const boost::system::error_code& error,
		std::size_t bytes_transferred)
	{
		if (!error || error == boost::asio::error::message_size)
		{
			const OrientationEventPackage &package = *(OrientationEventPackage*)&recv_buffer_[0];
			if (package.TimeStamp > LatestTimestep)
			{
				have_new_data = true;
				packages[writable_package_no] = package;
				OrientationChanged(&packages[writable_package_no]);
				//std::cout << "Ticks : " << package.TimeStamp << " , Quat : (" << package.X << ',' << package.Y << ',' << package.Z << ',' << package.W << ')' << std::endl;
			}
			start_receive();
		}
	}

	void handle_send(boost::shared_ptr<std::string> /*message*/,
		const boost::system::error_code& /*error*/,
		std::size_t /*bytes_transferred*/)
	{
	}

	// Double buffer of the package
	bool						have_new_data;
	mutable size_t				writable_package_no;
	OrientationEventPackage		packages[2];
	boost::asio::io_service		io_service_;
	udp::socket					socket_;
	udp::endpoint				remote_endpoint_;
	boost::array<char, 25U>		recv_buffer_;
	std::thread					running_thread_;
	int64_t						LatestTimestep;
};


OrientedTouchPad::OrientedTouchPad()
{
	using namespace Platform;
	p_Impl = std::make_unique<OrientatedTablet>();
	p_Impl->OrientationChanged += [this](OrientatedTablet::OrientationEventPackage* package)
	{
		OVR::Quatf q(package->X, package->Y, package->Z, package->W);
		OrientationChanged(q);
	};
}


OrientedTouchPad::~OrientedTouchPad()
{
	p_Impl->Stop();
}

void Platform::Input::OrientedTouchPad::Start()
{
	p_Impl->Start();
}

void Platform::Input::OrientedTouchPad::Stop()
{
	p_Impl->Stop();
}


void Platform::Input::OrientedTouchPad::Update()
{
	OVR::Quatf q;
	if (p_Impl->HaveNewData())
	{
		auto pack = p_Impl->CurrentOrientation();
		p_Impl->SwapBuffer();

		q = OVR::Quatf(pack.X, pack.Y, pack.Z, pack.W);
		q = DefaultOrientation.Inverted() * q;
		CurrentOrientation.Invert();
		CurrentOrientationDelta = CurrentOrientation * q;
		CurrentOrientation = q;
	}
	else
	{
		CurrentOrientationDelta = OVR::Quatf(0, 0, 0, 0);
	}
}

OVR::Quatf OrientedTouchPad::GetCurrentOrientation() const
{
	return CurrentOrientation;
}

void OrientedTouchPad::ResetOrientation()
{
	auto pack = p_Impl->CurrentOrientation();
	DefaultOrientation = OVR::Quatf(pack.X, pack.Y, pack.Z, pack.W);
}

OVR::Quatf Platform::Input::OrientedTouchPad::GetOrientationDelta() const
{
	return CurrentOrientationDelta;
}
