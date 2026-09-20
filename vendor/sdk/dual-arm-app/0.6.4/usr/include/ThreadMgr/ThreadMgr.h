#pragma once
#include<boost/thread.hpp>

namespace ThreadPool {

	class ThreadMgr: boost::thread_group
	{
	public:
		ThreadMgr();
		~ThreadMgr() throw();
		
		/**
		@brief 接收void(void)函数,并创建线程
		@return boost::thread*
		*/
		using boost::thread_group::create_thread;

		static boost::shared_ptr<ThreadMgr> create();

	};

	typedef boost::shared_ptr<ThreadMgr> ThreadMgrPtr;
}
