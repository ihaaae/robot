#pragma once
#define LOCK_QUEUE_AND_HANDLE_OP_PTR  do{std::unique_lock<std::mutex> lock{mutex_};\
if (asyncOpQueue_.empty() && !continuationHandler_)\
{\
	continuation();\
	boost::asio::async_write(*commChannel_, boost::asio::buffer(*pacPtr),\
		[this, pacPtr](const boost::system::error_code& ec, size_t s)\
		{\
		this->continuationHandler_(ec, s);\
		}\
	);\
}\
else\
{\
	asyncOpQueue_.emplace([this, pacPtr]{\
		boost::asio::async_write(*commChannel_, boost::asio::buffer(*pacPtr),\
			[this, pacPtr](const boost::system::error_code& ec, size_t s)\
				{\
			this->continuationHandler_(ec, s);\
				}\
		);\
		});\
}}while(0)

#define LOCK_QUEUE_AND_HANDLE_OP(...) do{std::unique_lock<std::mutex> lock{mutex_};\
if (asyncOpQueue_.empty() && !continuationHandler_)\
{\
	continuation();\
	boost::asio::async_write(*commChannel_, boost::asio::buffer(__VA_ARGS__),\
		this->continuationHandler_\
	);\
}\
else\
{\
	asyncOpQueue_.emplace([this,& __VA_ARGS__]{\
		boost::asio::async_write(*commChannel_, boost::asio::buffer(__VA_ARGS__),\
			this->continuationHandler_\
		);\
				});\
}}while(0)

template<typename ChannelType>
void AsyncWriteHandle<ChannelType>::doWrite(const std::vector<char>& package)
{
	LOCK_QUEUE_AND_HANDLE_OP(package);
}

template<typename ChannelType>
void AsyncWriteHandle<ChannelType>::doWrite(const std::string& package)
{
	LOCK_QUEUE_AND_HANDLE_OP(package);
}

template<typename ChannelType>
void AsyncWriteHandle<ChannelType>::doWrite(const char * package, size_t s)
{
	LOCK_QUEUE_AND_HANDLE_OP(package, s);
}

template<typename ChannelType>
void AsyncWriteHandle<ChannelType>::doWrite(const unsigned char * package, size_t s)
{
	LOCK_QUEUE_AND_HANDLE_OP(package, s);
}

template<typename ChannelType>
void AsyncWriteHandle<ChannelType> ::doWriteCopy(const std::vector<char>& package)
{
	boost::shared_ptr<std::vector<char>> pacPtr = boost::make_shared<std::vector<char>>(package);
	LOCK_QUEUE_AND_HANDLE_OP_PTR;
}

template<typename ChannelType>
void AsyncWriteHandle<ChannelType>::doWriteCopy(const std::string& package)
{
	boost::shared_ptr<std::string> pacPtr = boost::make_shared<std::string>(package);
	LOCK_QUEUE_AND_HANDLE_OP_PTR;
}

template<typename ChannelType>
void AsyncWriteHandle<ChannelType>::doWriteCopy(const char * package, size_t s)
{
	auto  pacPtr = boost::make_shared<std::vector<char>>(package, package + s);
	LOCK_QUEUE_AND_HANDLE_OP_PTR;
}

template<typename ChannelType>
void AsyncWriteHandle<ChannelType>::doWriteCopy(const unsigned char * package, size_t s)
{
	auto  pacPtr = boost::make_shared< std::vector<char>>(package, package + s);
	LOCK_QUEUE_AND_HANDLE_OP_PTR;
}

template<typename ChannelType>
void AsyncWriteHandle<ChannelType> ::continuation()
{
	continue_ = true;
}

template<typename ChannelType>
void AsyncWriteHandle<ChannelType>::resetChannel(boost::shared_ptr<ChannelType>& channel)
{
	commChannel_ = channel;
}

template<typename ChannelType>
void AsyncWriteHandle<ChannelType>::assignHandler(WriteHandler whandler)
{
	whandler_ = move(whandler);
}
template<typename ChannelType>
AsyncWriteHandle<ChannelType>::AsyncWriteHandle(boost::asio::io_service& ios) :commChannel_(new ChannelType{ ios }), continuationHandler_()
{
	continuationHandler_.parent_ = this;
}


template<typename ChannelType>
AsyncWriteHandle<ChannelType>::AsyncWriteHandle(boost::shared_ptr<ChannelType>& channel) :continuationHandler_()
{
	commChannel_ = channel;
	continuationHandler_.parent_ = this;
}

template<typename ChannelType>
struct AsyncWriteHandle<ChannelType>::Continuation
{
	AsyncWriteHandle<ChannelType>* parent_;
	explicit operator bool() const { return parent_->continue_; }
	void operator()(const boost::system::error_code& ec, size_t s)
	{
		if (parent_->whandler_)
			parent_->whandler_(ec, s);
		parent_->continue_ = false;
		std::unique_lock<std::mutex> lock{ parent_->mutex_ };
		if (!parent_->asyncOpQueue_.empty())
		{
			auto op = std::move(parent_->asyncOpQueue_.front());
			parent_->asyncOpQueue_.pop();
			op();
			parent_->continue_ = true;
		}
	}
};
template<typename ChannelType>
template<size_t N>
void AsyncWriteHandle<ChannelType>::doWrite(const char(&myArray)[N])
{
	LOCK_QUEUE_AND_HANDLE_OP(myArray);
}

template<typename ChannelType>
template<size_t N>
void AsyncWriteHandle<ChannelType>::doWrite(const unsigned char(&myArray)[N])
{
	LOCK_QUEUE_AND_HANDLE_OP(myArray);
}

template<typename ChannelType>
template<size_t N>
void AsyncWriteHandle<ChannelType>::doWrite(const char* m)
{
	const char(*ptrToArray)[N];
	ptrToArray = reinterpret_cast<decltype(ptrToArray)>(m);
	doWrite(*ptrToArray);
}

template<typename ChannelType>
template<size_t N>
void AsyncWriteHandle<ChannelType>::doWrite(const unsigned char* m)
{
	const unsigned char(*ptrToArray)[N];
	ptrToArray = reinterpret_cast<decltype(ptrToArray)>(m);
	doWrite(*ptrToArray);
}

template<typename ChannelType>
template<template<typename, size_t> class theArray, typename T, size_t size>
void AsyncWriteHandle<ChannelType>::doWrite(const theArray<T, size>& myArray)
{
	LOCK_QUEUE_AND_HANDLE_OP(myArray);
}

//copy
template<typename ChannelType>
template<size_t N>
void AsyncWriteHandle<ChannelType>::doWriteCopy(const char(&myArray)[N])
{
	auto  pacPtr = boost::make_shared< boost::array<char, N> >();
	std::copy_n(myArray, N, pacPtr->begin());
	LOCK_QUEUE_AND_HANDLE_OP_PTR;
}

template<typename ChannelType>
template<size_t N>
void AsyncWriteHandle<ChannelType>::doWriteCopy(const unsigned char(&myArray)[N])
{
	auto  pacPtr = boost::make_shared< boost::array<unsigned char, N> >();
	std::copy_n(myArray, N, pacPtr->begin());
	LOCK_QUEUE_AND_HANDLE_OP_PTR;
}

template<typename ChannelType>
template<size_t N>
void AsyncWriteHandle<ChannelType>::doWriteCopy(const char* m)
{
	const char(*ptrToArray)[N];
	ptrToArray = reinterpret_cast<decltype(ptrToArray)>(m);
	doWriteCopy(*ptrToArray);
}

template<typename ChannelType>
template<size_t N>
void AsyncWriteHandle<ChannelType>::doWriteCopy(const unsigned char* m)
{
	const unsigned char(*ptrToArray)[N];
	ptrToArray = reinterpret_cast<decltype(ptrToArray)>(m);
	doWriteCopy(*ptrToArray);
}

template<typename ChannelType>
template<template<typename, size_t> class theArray, typename T, size_t size>
void AsyncWriteHandle<ChannelType>::doWriteCopy(const theArray<T, size>& myArray)
{
	auto  pacPtr = boost::make_shared<theArray<T, size>>(myArray);
	LOCK_QUEUE_AND_HANDLE_OP_PTR;
}
#undef LOCK_QUEUE_AND_HANDLE_OP_PTR
#undef LOCK_QUEUE_AND_HANDLE_OP