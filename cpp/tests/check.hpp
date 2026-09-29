// A tiny assertion harness: no external dependency, so the default build needs nothing but a
// C++17 compiler. Tests register themselves; tests/main.cpp runs them.
#ifndef SHENSI_TEST_CHECK_HPP
#define SHENSI_TEST_CHECK_HPP

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <type_traits>
#include <vector>

namespace shensi::test {

struct Case {
    const char* name;
    void (*fn)();
};

inline std::vector<Case>& cases() {
    static std::vector<Case> registry;
    return registry;
}

inline int& failures() {
    static int count = 0;
    return count;
}

inline int& checks() {
    static int count = 0;
    return count;
}

struct Registrar {
    Registrar(const char* name, void (*fn)()) { cases().push_back(Case{name, fn}); }
};

inline void report_failure(const char* file, int line, const std::string& what) {
    std::fprintf(stderr, "    FAIL %s:%d: %s\n", file, line, what.c_str());
    ++failures();
}

inline void check(bool condition, const char* file, int line, const std::string& what) {
    ++checks();
    if (!condition) report_failure(file, line, what);
}

// Lowercase hex with no separators, which is how the golden vectors are written.
// Provided by the library as shensi::can::to_hex; the harness does not duplicate it.

inline std::string describe(bool value) { return value ? "true" : "false"; }
inline std::string describe(const std::string& value) { return '"' + value + '"'; }
inline std::string describe(double value) { return std::to_string(value); }

template <typename T>
std::string describe(const T& value) {
    if constexpr (std::is_integral_v<T>) {
        return std::to_string(value);
    } else {
        return "<unprintable>";
    }
}

template <typename Fn>
void check_throws(const char* file, int line, const std::string& what, Fn&& fn) {
    ++checks();
    try {
        fn();
    } catch (...) {
        return;
    }
    report_failure(file, line, what + ": expected it to throw, and it did not");
}

}  // namespace shensi::test

#define SHENSI_TEST_CASE(name)                                                          \
    static void name();                                                                 \
    static const ::shensi::test::Registrar shensi_registrar_##name(#name, &name);        \
    static void name()

#define CHECK(cond)                                                                     \
    ::shensi::test::check((cond), __FILE__, __LINE__, "expected: " #cond)

#define CHECK_MSG(cond, message)                                                        \
    ::shensi::test::check((cond), __FILE__, __LINE__, (message))

#define CHECK_EQ(got, want)                                                             \
    do {                                                                                \
        const auto& shensi_got_ = (got);                                                \
        const auto& shensi_want_ = (want);                                              \
        ::shensi::test::check(shensi_got_ == shensi_want_, __FILE__, __LINE__,          \
            std::string(#got " == " #want "\n      got  ") +                            \
            ::shensi::test::describe(shensi_got_) + "\n      want " +                   \
            ::shensi::test::describe(shensi_want_));                                    \
    } while (false)

#define CHECK_THROWS(expr)                                                              \
    ::shensi::test::check_throws(__FILE__, __LINE__, #expr, [&] { (void)(expr); })

#endif  // SHENSI_TEST_CHECK_HPP
