

#ifndef LOGGER_H
#define LOGGER_H


#ifndef ENABLE_LOGGING
#define ENABLE_LOGGING 0
#endif

#include <string>


std::string UserIdPrompt();
std::string UserIdPrompt(int, char**);
void InitLogfile(const std::string&, const std::string&);
void UpdateLogfile(const int, const int);
void CloseLogfile();
void CloseCompletedLogfile();


#endif // LOGGER_H


