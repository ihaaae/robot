#pragma once
#include <boost/asio.hpp>
#include <functional>
#include<boost/bind.hpp>
#include <boost/asio/spawn.hpp>
#include <type_traits>


namespace Common
{
	
	/**
	@section 类说明
	* 此类用于检查某个布尔值是否为真(非阻塞,异步)
	* 建议配合协程使用
	* 也可配合std::future和回调函数
	* 需要更改判断条件或者时间需要在yiled之前，
	* 否则将在下一次yield生效，
	* 对于需要自定义其他异步操作来配合asio工作，请参考
	* asyncCheckIsDone和check函数的定义
	* 和 http://open-std.org/jtc1/sc22/wg21/docs/papers/2014/n4045.pdf
	*/
	class CheckIsDoneTimer
	{
	public:
		CheckIsDoneTimer(boost::asio::io_service& ios) :ios_(ios), dt_(ios){}

		
		/**
		@brief 设置需要检查bool值函数,为true继续执行
		@return CheckIsDoneTimer& 
		@param CheckFuncType && 一个返回bool值的函数对象
		*/
		template<typename CheckFuncType>
		CheckIsDoneTimer& setCheckFunction(CheckFuncType&& f)
		{
			static_assert(std::is_same<decltype(f()), bool>::value, "CheckFunction type requirements not met");
			this->f_ = f; return *this;
		}

		/**
		@brief 设置需要检查bool值函数,为true继续执行
		@return CheckIsDoneTimer&
		@param bool & 用于检查bool引用
		*/
		CheckIsDoneTimer& setBoolRef(bool & b){ f_ = [&b](){return b; }; return *this; }
		
		/**
		@brief 用于设置多长时间(毫秒)检测一回bool值
		@return CheckIsDoneTimer&
		@param long milliseconds 
		*/
		CheckIsDoneTimer& setCheckTime(long milliseconds){ millSeconds_ = milliseconds; return *this; }

		/**
		@brief 等待一段时间,不做任何事(但期间不阻塞线程,也不sleep)
		@return 
		@param CompletionToken && token 回掉函数 或者 协程上下文
		*/
		template<typename CompletionToken>
		BOOST_ASIO_INITFN_RESULT_TYPE(CompletionToken, void(boost::system::error_code))
		justWait(CompletionToken&&  token)
		{
			setCheckFunction([]{return true; }).asyncCheckIsDone(token);
		}


		/**
		@brief 注册回调函数 或者 协程上下文
		@return
		@param CompletionToken && token 回掉函数 或者 协程上下文
		*/
		template<typename CompletionToken>
		BOOST_ASIO_INITFN_RESULT_TYPE(CompletionToken, void(boost::system::error_code))
		asyncCheckIsDone(CompletionToken&&  token)
		{
				using namespace boost::asio;
				using namespace boost;
				asio::detail::async_result_init<CompletionToken, void(system::error_code)> asyncResInit{ std::forward<decltype(token)>(token) };
				dt_.expires_from_now(posix_time::milliseconds{ millSeconds_ });
				dt_.async_wait(boost::bind(&CheckIsDoneTimer::check<decltype(asyncResInit.handler)>, this, placeholders::error, asyncResInit.handler));
				return asyncResInit.result.get();
		}

	private:
		template<typename HandlerType>
		void check(const boost::system::error_code& ec, HandlerType h)
		{
			if (!ec)
			{
				if (f_())
				{
					ios_.post([h]() mutable {h(boost::system::error_code{}); });
				}
				else
				{
					dt_.expires_from_now(boost::posix_time::milliseconds{ millSeconds_ });
					dt_.async_wait(boost::bind(&CheckIsDoneTimer::check<HandlerType>, this, boost::asio::placeholders::error, h));
				}
			}
		}

	private:
		boost::asio::io_service& ios_;
		boost::asio::deadline_timer dt_;
		std::function<bool()> f_;
		long millSeconds_ = 50;
	};
}