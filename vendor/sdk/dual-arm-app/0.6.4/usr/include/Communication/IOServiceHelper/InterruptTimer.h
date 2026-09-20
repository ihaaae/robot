#pragma once
#include <boost/asio.hpp>

namespace Common
{
	class InterruptTimer
	{
	public:
		InterruptTimer(boost::asio::io_service& ios);
	protected:
		void handleInterrupt(const boost::system::error_code&ec);
	private:
		boost::asio::deadline_timer interruptTimer_;
	};
}