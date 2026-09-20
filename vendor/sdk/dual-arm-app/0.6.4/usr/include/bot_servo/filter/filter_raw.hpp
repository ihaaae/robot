#ifndef DUAL_ARM_APP_FILTER_RAW_HPP
#define DUAL_ARM_APP_FILTER_RAW_HPP
#include "filter_base.hpp"
template<typename T>
class FilterRaw: public FilterBase<T>{
public:
    FilterRaw(): FilterBase<T>(){

    }

    ~FilterRaw() override = default;
public:
    void observe(const T &state) override{

        if(last_data.size() == 0)
            m_data = state;
        else{
            double a = 1.; // full pass
            m_data = state * a + last_data * (1. - a);
        }
        first = false;
    }

    std::optional<T> update() override{
        if(!first){
            if(last_data.size() == 0)
            {
                last_data = m_data;
                return m_data;
            }
            else
            {
                last_data = m_data;
                return m_data;
            }
        }

        return std::nullopt;
    }

private:
    bool first = true;
    T m_data;
    T last_data;
};
#endif //DUAL_ARM_APP_FILTER_RAW_HPP
