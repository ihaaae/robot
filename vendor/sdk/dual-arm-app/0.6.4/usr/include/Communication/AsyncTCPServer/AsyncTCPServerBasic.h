#pragma once
#include <boost/asio.hpp>

class AsyncTCPServerBasic
{
public:
	AsyncTCPServerBasic(int port,std::string ip = boost::asio::ip::address_v4::any().to_string());

	virtual ~AsyncTCPServerBasic() = 0;

	boost::asio::io_service& getIoService(){ return ios_; }

protected:
	boost::asio::io_service ios_;
	
	boost::shared_ptr<boost::asio::ip::tcp::socket> socketPtr_;

	boost::asio::ip::tcp::acceptor acceptor_;

	boost::asio::streambuf streambuf_;

};
