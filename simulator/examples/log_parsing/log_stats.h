

#ifndef LOG_STATS_H
#define LOG_STATS_H


#include "log_parser.h"


float Average(const std::vector<float>& data);
float StandardDeviation(const std::vector<float>& data);
float StandardDeviation(const std::vector<float>& data, const float average);
float TotalMouseDistance(const LOG_DATA& log_data);
float TotalShepherdDistance(const LOG_DATA& log_data, const int which_shepherd);
std::vector<float> TotalShepherdDistance(const LOG_DATA& log_data);
float AverageSheepSeparation(const LOG_DATA& log_data);
float AverageNumberOfGroups(const LOG_DATA& log_data, const float distance_threshold = 5.0f);

#endif // LOG_STATS_H


