//
// Created by Maxim Claeys on 31/05/2017.
//

#ifndef LOGGER_H
#define LOGGER_H

#include "spdlog/spdlog.h"
#include "spdlog/logger.h"
#include "spdlog/common.h"
#include <memory>
#include <string>
#include <cstdint>
#include <map>
#include <typeinfo>
#include <typeindex>
#include "LogLevel.h"

class Logger
{
private:
    static Logger m_instance;
    std::map<std::type_index, std::shared_ptr<spdlog::logger> > m_logconsoles;

    Logger();

    template<typename T>
    std::shared_ptr<spdlog::logger> get_or_create_logger()
    {
        std::map<std::type_index, std::shared_ptr<spdlog::logger> >::iterator lookup = m_logconsoles.find(typeid(T));
        if(lookup!=m_logconsoles.end()){
            return lookup->second;
        }else{
            std::shared_ptr<spdlog::logger> nlogger = spdlog::stdout_color_mt(typeid(T).name());
            m_logconsoles.insert(std::pair<std::type_index, std::shared_ptr<spdlog::logger> >(typeid(T), nlogger));
            return nlogger;
        }
    }

    static spdlog::level::level_enum translate_level(loglevel mylevel)
    {
        switch(mylevel){
            case TRACE:
                return spdlog::level::trace;
            case DEBUG:
                return spdlog::level::debug;
            case INFO:
                return spdlog::level::info;
            case WARNING:
                return spdlog::level::warn;
            case ERROR:
                return spdlog::level::err;
            case CRITICAL:
                return spdlog::level::critical;
            default:
                return spdlog::level::off;
        }
    }
public:
    Logger(Logger const&) = delete;

    template<typename T>
    static void log_critical(const std::string& message)
    {
        m_instance.get_or_create_logger<T>()->critical(message.c_str());
    }

    template<typename T, typename Arg1, typename... Args>
    static void log_critical(const std::string& message, const Arg1& arg1, const Args&... args)
    {
        m_instance.get_or_create_logger<T>()->critical(message.c_str(), arg1, args...);
    }

    template<typename T>
    static void log_error(const std::string& message)
    {
        m_instance.get_or_create_logger<T>()->error(message.c_str());
    }

    template<typename T, typename Arg1, typename... Args>
    static void log_error(const std::string& message, const Arg1& arg1, const Args&... args)
    {
        m_instance.get_or_create_logger<T>()->error(message.c_str(), arg1, args...);
    }

    template<typename T>
    static void log_warning(const std::string& message)
    {
        m_instance.get_or_create_logger<T>()->warn(message.c_str());
    }

    template<typename T, typename Arg1, typename... Args>
    static void log_warning(const std::string& message, const Arg1& arg1, const Args&... args)
    {
        m_instance.get_or_create_logger<T>()->warn(message.c_str(), arg1, args...);
    }

    template<typename T>
    static void log_info(const std::string& message)
    {
        m_instance.get_or_create_logger<T>()->info(message.c_str());
    }

    template<typename T, typename Arg1, typename... Args>
    static void log_info(const std::string& message, const Arg1& arg1, const Args&... args)
    {
        m_instance.get_or_create_logger<T>()->info(message.c_str(), arg1, args...);
    }

    template<typename T>
    static void log_debug(const std::string& message)
    {
        m_instance.get_or_create_logger<T>()->debug(message.c_str());
    }

    template<typename T, typename Arg1, typename... Args>
    static void log_debug(const std::string& message, const Arg1& arg1, const Args&... args)
    {
        m_instance.get_or_create_logger<T>()->debug(message.c_str(), arg1, args...);
    }

    template<typename T>
    static void log_trace(const std::string& message)
    {
        m_instance.get_or_create_logger<T>()->trace(message.c_str());
    }

    template<typename T, typename Arg1, typename... Args>
    static void log_trace(const std::string& message, const Arg1& arg1, const Args&... args)
    {
        m_instance.get_or_create_logger<T>()->trace(message.c_str(), arg1, args...);
    }

    template<typename T>
    static void set_log_level(loglevel mylevel){
        spdlog::level::level_enum spdlevel = translate_level(mylevel);
        std::shared_ptr<spdlog::logger> logger = m_instance.get_or_create_logger<T>();
        logger->set_level(spdlevel);
    }

    static void set_global_log_level(loglevel mylevel){
        spdlog::level::level_enum spdlevel = translate_level(mylevel);
        spdlog::set_level(spdlevel);
    }
};

#endif //LOGGER_H
