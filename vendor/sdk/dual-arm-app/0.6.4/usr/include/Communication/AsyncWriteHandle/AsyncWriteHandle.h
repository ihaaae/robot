#pragma once
#include <boost/asio.hpp>
#include <functional>
#include <boost/make_shared.hpp>
#include <boost/array.hpp>
#include <queue>
#include <mutex>
#include <boost/noncopyable.hpp>

template<typename ChannelType>
class AsyncWriteHandle :private boost::noncopyable
{
public:
	using WriteHandler = std::function<void(const boost::system::error_code&, size_t)>;
public:
	AsyncWriteHandle(boost::shared_ptr<ChannelType>& channel);
	AsyncWriteHandle(boost::asio::io_service& ios);
	void assignHandler(WriteHandler whandler);
	void resetChannel(boost::shared_ptr<ChannelType>& channel);
	void continuation();
	~AsyncWriteHandle() {}
	struct Continuation;
public:
	template<size_t N>
	void doWrite(const char(&)[N]);

	template<size_t N>
	void doWrite(const unsigned char(&)[N]);

	template<size_t N>
	void doWrite(const char*);

	template<size_t N>
	void doWrite(const unsigned char*);

	template<template<typename, size_t> class theArray, typename T, size_t size>
	void doWrite(const theArray<T, size>&);

	void doWrite(const std::vector<char>& package);

	void doWrite(const std::string& package);
	
	void doWrite(const char * package, size_t s);

	void doWrite(const unsigned char * package, size_t s);

	template<size_t N>
	void doWriteCopy(const char(&)[N]);

	template<size_t N>
	void doWriteCopy(const unsigned char(&)[N]);

	template<size_t N>
	void doWriteCopy(const char*);

	template<size_t N>
	void doWriteCopy(const unsigned char*);

	template<template<typename, size_t> class theArray, typename T, size_t size>
	void doWriteCopy(const theArray<T, size>&);

	void doWriteCopy(const std::vector<char>& package);

	void doWriteCopy(const std::string& package);

	void doWriteCopy(const char * package, size_t s);

	void doWriteCopy(const unsigned char * package, size_t s);

protected:

	boost::shared_ptr<ChannelType> commChannel_;

	WriteHandler whandler_;

	std::queue<std::function<void()>> asyncOpQueue_;

	std::mutex mutex_;
	
	Continuation continuationHandler_;

	bool continue_ = false;
};


#include "AsyncWriteHandleImpl.h"

