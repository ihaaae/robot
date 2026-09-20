#ifndef DUAL_ARM_APP_FILTER_BASE_HPP
#define DUAL_ARM_APP_FILTER_BASE_HPP
#include <optional>

template<typename T>
class FilterBase {
public:
    FilterBase() = default;

    virtual ~FilterBase() = default;
public:
    virtual void observe(const T &state) = 0;

    virtual std::optional<T> update() = 0;
};

#endif //DUAL_ARM_APP_FILTER_BASE_HPP
