#pragma once
#include "TCPServer.h"

namespace Communication
{
	class TCPServerClient :public AsyncWriteHandle<boost::asio::ip::tcp::socket>
	{
	public:
		/**
		@brief 代表一个TCP客户端
		@param boost::shared_ptr<boost::asio::ip::tcp::socket> socket指针
		@param boost::regex 正则表达式
		*/
		TCPServerClient(boost::shared_ptr<boost::asio::ip::tcp::socket> socketPtr, boost::regex expr);
		TCPServerClient(boost::shared_ptr<boost::asio::ip::tcp::socket> socketPtr, const MatchFunction & matchFunction);
		~TCPServerClient() {}
		/**
		@brief 设置每个Client读取到指定信息的回调函数
		@param std::function<void(std::vector<char>&)>
		*/
		void setReadFunction(std::function<void(std::vector<char>&)> f);
	private:
		bool disposeData(const boost::system::error_code& ec, size_t readbytesTransferred);
		void doRead();
		void doReadMatchClass();
		std::function<void(std::vector<char>&)>  readFunction_;
		boost::asio::streambuf streambuf_;
		boost::regex expr_;
		const MatchFunction matchFunction_;
	};
}
