#pragma once
#include "AsyncTCPServerBasic.h"
#include "../IOServiceHelper/InterruptTimer.h"
#include "../AsyncWriteHandle/AsyncWriteHandle.h"
#include "PackageHelper/PackageHelper.h"
#include "ITCPClientManager.h"
#include "MatchClass/MatchClass.h"
#include <boost/noncopyable.hpp>

namespace Communication
{
	class TCPServer :public AsyncTCPServerBasic, private boost::noncopyable, public Common::InterruptTimer
	{
	public:

		/**
		@brief 有头有尾的包构造函数
		@param int  端口号
		@param const std::string  分隔符(即数据包的尾)
		*/
		TCPServer(int port, const std::string delim,std::string ip = boost::asio::ip::address_v4::any().to_string());

		/**
		@brief 设置自定义manager指针,用于在client连接时触发回调函数ITCPClientManager->addClient
		@return void
		@param ITCPClientManager * 设置自定义manager指针
		*/
		void setClientManager(ITCPClientManager* m) { clientManager_ = m; }
		//AsyncWriteHandle<boost::asio::ip::tcp::socket> writeHandle_;
		void run();
		~TCPServer();

	private:
		void doAccept();
		ITCPClientManager* clientManager_ = nullptr;
		boost::shared_ptr<boost::asio::ip::tcp::socket> newConnection_;
		std::function<void()> acceptFunction_;
		boost::shared_ptr<PackageHelper> packageHelper_;
		boost::shared_ptr<MatchClass> matchClass_;

		enum class MatchMode
		{
			UNKNOWN,
			REGEX,
			CUSTOM,
		};
		MatchMode matchMode_;
	};
}
