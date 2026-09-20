#pragma once
#include <boost/regex.hpp>
#include <initializer_list>
#include <Package/Package.h>

namespace Communication
{
	class PackageHelper
	{
	public:
		PackageHelper();
		PackageHelper(std::string delim);
		PackageHelper(PackageHead head);
		PackageHelper(PackageHead head, int wihtoutHeadPackageMaxSize, int sizeOfLength = 1);  // sizeOfLength 取值1和2
		PackageHelper(PackageHead head, int withoutHeadAndTailPackageMaxSize, PackageTail tail, int sizeOfLength = 1); // sizeOfLength 取值1和2
		explicit PackageHelper(boost::regex expr);

		boost::regex getRegex();

	private:
		std::string constructRegex(std::vector<unsigned char> head);
		std::string constructRegex(std::vector<unsigned char> head, int withoutHeadPackageLength);
		std::string constructRegex(std::vector<unsigned char> head, int withoutHeadAndTailPackageLength, std::vector<unsigned char> tail);
		std::string constructTwoOfLength(std::vector<unsigned char> head, int withoutHeadAndTailPackageLength);
		std::string constructTwoOfLength(std::vector<unsigned char> head, int withoutHeadAndTailPackageLength, std::vector<unsigned char> tail);

	private:
		boost::regex regExpr_;
	};
}
