//
// Created by Maxim Claeys on 31/05/2017.
//

//
// Created by Maxim Claeys on 31/05/2017.
//
#include "Logger.h"

Logger Logger::m_instance;

Logger::Logger(){
    //Set default logging to ERROR
    spdlog::set_level(spdlog::level::err);
    spdlog::set_pattern("[%d/%m/%Y %H:%M:%S.%F] [%n] [%l] %v");
}
