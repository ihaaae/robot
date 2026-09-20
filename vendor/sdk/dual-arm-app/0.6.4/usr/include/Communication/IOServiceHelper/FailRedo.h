#pragma once
#include <boost/asio.hpp>
#include <functional>
#include <iostream>
#include<boost/bind.hpp>
#include <boost/asio/spawn.hpp>
#include <type_traits>
#include <boost/asio/async_result.hpp>
#include <boost/asio.hpp>
namespace Common
{

	/**
	@section 类说明
	* 此类用于实现超时重传或者心跳功能(非阻塞,异步)
	* 建议配合回调函数使用
	* 对于需要自定义其他异步操作来配合asio工作，请参考
	* asyncCheckIsDone和check函数的定义
	* 和 http://open-std.org/jtc1/sc22/wg21/docs/papers/2014/n4045.pdf
	*/
	class FailRedoTimer
	{
	public:
		FailRedoTimer(boost::asio::io_service& ios) :ios_(ios), dt_(ios){}


		/**
		@brief 在未达到最大失败次数时每次失败执行函数（如：超时重传执行“重传”函数）
		@return FailRedoTimer&
		@param FailFuncType && 每次失败后的函数
		*/
		template<typename FailFuncType>
		FailRedoTimer& setFailHandle(FailFuncType&& f)
		{
			static_assert(std::is_same<decltype(f()),void>::value, "FailHandle type requirements not met");
			this->failFunc_ = f; return *this;
		}

		/**
		@brief 用于设置多长时间(毫秒)检测一下失败次数
		@return FailRedoTimer&
		@param long milliseconds
		*/
		FailRedoTimer& setTimeoutThreshold(long milliseconds){ millSeconds_ = milliseconds; return *this; }

		/**
		@brief 用于设置多长时间(毫秒)检测一下失败次数
		@return FailRedoTimer&
		@param long milliseconds
		*/
		FailRedoTimer& setTimePeriod(long milliseconds){ millSeconds_ = milliseconds; return *this; }


		/**
		@brief 注册回调函数 或者 协程上下文 （如：超时重传失败N次后执行断链函数）
		@return 依模板特化决定
		@param CompletionToken && token 回掉函数(这里可以设为失败N次后执行逻辑) 或者 协程上下文
		*/
		template<typename CompletionToken>
		BOOST_ASIO_INITFN_RESULT_TYPE(CompletionToken, void(boost::system::error_code))
		ifFailMaxTimesHandle(CompletionToken&&  token)
		{
			using namespace boost::asio;
			using namespace boost;
			asio::detail::async_result_init<CompletionToken, void(system::error_code)> asyncResInit{ std::forward<decltype(token)>(token) };
			ios_.post([this](){failFunc_(); });
			failConunt_ = 1;
			dt_.expires_from_now(posix_time::milliseconds{ millSeconds_ });
			dt_.async_wait(boost::bind(&FailRedoTimer::check<decltype(asyncResInit.handler)>, this,boost::asio::placeholders::error, asyncResInit.handler));
			return asyncResInit.result.get();
		}

		/**
		@brief 将失败次数置为0,如果setIsRecursive(false),则异步操作需要用ifFailMaxTimesHandle重新注册
		@return FailRedoTimer& 
		*/
		FailRedoTimer& resetFailCount()
		{
			failConunt_ = 0; 
			return *this;
		}

		/**
		@brief 设置最大尝试次数
		@return FailRedoTimer&
		@param unsigned chance 最大尝试次数
		*/
		FailRedoTimer& setMaxAttemptNum(unsigned chance){ maxFailChance_ = chance+1; return *this; }

		/**
		@brief 为true时，在达到最大尝试次数后自动重新使用ifFailMaxTimesHandle注册,或者在resetFailCount后自动注册handle
		@return FailRedoTimer&
		@param bool isRecursive 设置是否递归
		*/
		FailRedoTimer& setIsRecursive(bool isRecursive){ isRecursive_ = isRecursive; return *this; }

	private:
		template<typename HandlerType>
		void check(const boost::system::error_code& ec, HandlerType h)
		{
			using namespace boost::asio;
			if (ec!=error::operation_aborted)
			{
				
				if (failConunt_==maxFailChance_)
				{
					ios_.post([h]() mutable {h(boost::system::error_code{}); });
					resetFailCount();
					if (isRecursive_)
					ifFailMaxTimesHandle(h);
				}
				else if (failConunt_ == 0)
				{
					if (isRecursive_)
						ifFailMaxTimesHandle(h);
					return;
				}
				else
				{
					failFunc_();
					failConunt_ += 1;
					dt_.expires_from_now(boost::posix_time::milliseconds{ millSeconds_ });
					dt_.async_wait(boost::bind(&FailRedoTimer::check<HandlerType>, this, placeholders::error, h));
				}
				
			}
		}

	private:
		boost::asio::io_service& ios_;
		boost::asio::deadline_timer dt_;
		std::function<void()> failFunc_;
		bool isRecursive_ = false;
		long millSeconds_ = 1000;
		unsigned failConunt_ = 0;
		unsigned maxFailChance_ = 5;
	};
}