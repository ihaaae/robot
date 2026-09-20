#pragma once
#include "boost/asio.hpp"
#include <Package/Package.h>

typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> Iterator;
typedef std::function<std::pair<Iterator, bool>(Iterator, Iterator)> MatchFunction;

namespace Communication
{
	class MatchClass
	{
	public:
		MatchClass();
		MatchClass(PackageHead head, int sizeOfLength); //sizeOfLength必须取值为1，2，3，4其中之一
		MatchClass(PackageHead head, PackageTail  tail, int sizeOfLength); //sizeOfLength必须取值为1，2，3，4其中之一
		explicit MatchClass(MatchFunction matchClass);

		MatchFunction getMatchFunction();

	private:
		bool isCheckFrameHead(const Iterator & begin, const Iterator & end,std::pair<Iterator, bool> & ret);
		bool isCalculateAndCheckDataSize(const Iterator & begin, const Iterator & end,std::pair<Iterator, bool> & ret, unsigned int & dataSize);
		std::pair<Iterator, bool> checkFrameTail(const Iterator & begin, const Iterator & end, unsigned int dataSize);

		std::vector<char> constructCharHead(PackageHead head);
		MatchFunction constructMatchClass();

	private:
		const std::vector<char> packageCharHead_;
		const PackageTail packageTail_;
		const int sizeOfLength_;

		const MatchFunction matchFunction_;
	};
}
