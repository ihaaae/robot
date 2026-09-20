/**
 * Common headers and erros
 */
#ifndef LOG_COMMON_H
#define LOG_COMMON_H

#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Initializers/ConsoleInitializer.h>
#include <map>
namespace plog {
    class MyFormatter {
    public:
        static util::nstring header() // This method returns a header for a new file. In our case it is empty.
        {
            return util::nstring();
        }

        static util::nstring format(const Record &record) // This method returns a string from a record.
        {
            tm t;
            util::localtime_s(&t, &record.getTime().time);

            util::nostringstream ss;
            ss << t.tm_year + 1900 << "-" << std::setfill(PLOG_NSTR('0')) << std::setw(2) << t.tm_mon + 1
               << PLOG_NSTR("-") << std::setfill(PLOG_NSTR('0')) << std::setw(2) << t.tm_mday << PLOG_NSTR(" ");
            ss << std::setfill(PLOG_NSTR('0')) << std::setw(2) << t.tm_hour << PLOG_NSTR(":")
               << std::setfill(PLOG_NSTR('0')) << std::setw(2) << t.tm_min << PLOG_NSTR(":")
               << std::setfill(PLOG_NSTR('0')) << std::setw(2) << t.tm_sec << PLOG_NSTR(".")
               << std::setfill(PLOG_NSTR('0')) << std::setw(3) << static_cast<int> (record.getTime().millitm)
               << PLOG_NSTR(" ");
            ss << std::setfill(PLOG_NSTR(' ')) << std::setw(5) << std::left << severityToString(record.getSeverity())
               << PLOG_NSTR(" ");
            ss << PLOG_NSTR("[") << record.getTid() << PLOG_NSTR("] ");
            ss << PLOG_NSTR("[") << strrchr(record.getFile(), '/') + 1 << PLOG_NSTR(":") << record.getFunc()
               << PLOG_NSTR("@") << record.getLine() << PLOG_NSTR("] ");
            ss << record.getMessage() << PLOG_NSTR("\n");

            return ss.str();
        }
    };

    template<class Formatter>
    class DynamicRollingFileAppender : public RollingFileAppender<Formatter> {
    public:
        DynamicRollingFileAppender(const std::string &defaultFilePath, size_t maxFileSize, int maxFiles,
                                   const std::map<std::string, std::string> &extralKeysAndFileNames = {})
                : RollingFileAppender<Formatter>("", maxFileSize, maxFiles),
                  defaultFilePath_(defaultFilePath),
                  extralKeysAndFileNames_(extralKeysAndFileNames) {
        }


        void write(const Record &record) override {
            std::string message = record.getMessage();
            this->setFileName(defaultFilePath_.c_str());
            for(const auto&  item : extralKeysAndFileNames_){
                if(message.find(item.first) !=std::string::npos){
                    this->setFileName(item.second.c_str());
                    break;
                }
            }
            RollingFileAppender<Formatter>::write(record);
        }

    private:
        std::string defaultFilePath_;
        std::map<std::string, std::string>extralKeysAndFileNames_;
    };
}

inline void setPlog(bool toConsole = true, const std::string &level = "Debug", const std::string &log_path = "",
                    size_t maxFileSize = 1000000, int maxFiles = 20, const std::map<std::string, std::string> &extralKeysAndFileNames = {}) {
    auto plevel = plog::severityFromString(level.c_str());
    if (!toConsole && !log_path.empty()) {
        static plog::DynamicRollingFileAppender<plog::MyFormatter> rollingFileAppender(log_path, maxFileSize, maxFiles, extralKeysAndFileNames);
        plog::init<PLOG_DEFAULT_INSTANCE_ID>(plevel, &rollingFileAppender);
    } else {
        static plog::ColorConsoleAppender<plog::MyFormatter> consoleAppender;
        plog::init(plevel, &consoleAppender);
    }
}


#endif //LOG_COMMON_H
