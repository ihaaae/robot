#pragma once
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/regex.hpp>
#include <boost/asio.hpp>

typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> Iterator;
typedef std::function<std::pair<Iterator, bool>(Iterator, Iterator)> MatchFunction;

namespace Communication
{
	class ITCPClientManager
	{
	public:
		/**
		@brief 用于管理Client类接口，根据业务逻辑不同自行实现
		@param int port 端口号
		@param std::string  address IP地址
		@param boost::shared_ptr<boost::asio::ip::tcp::socket> socket指针
		@param boost::regex & expr 正则表达式
		*/
		virtual void addClient(int port, std::string address, boost::shared_ptr<boost::asio::ip::tcp::socket>& socketPtr,const boost::regex& expr) = 0;

		//用于MatchClass匹配
		virtual void addClient(int port, std::string address, boost::shared_ptr<boost::asio::ip::tcp::socket>& socketPtr, const MatchFunction& matchFunction) = 0;
		virtual ~ITCPClientManager() = default;
	};
}
