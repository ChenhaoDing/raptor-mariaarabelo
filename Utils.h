//
// Created by maria on 10/28/2024.
//

#ifndef RAPTOR_UTILS_H
#define RAPTOR_UTILS_H


#include <sstream>
#include <iomanip>
#include <algorithm>
#include <utility> // for std::pair
#include <cmath>   // for std::sin, std::cos, std::atan2, std::sqrt
#include <vector>

#include "Constants.h"

class Utils {
public:

  static double haversine(const double& lat1, const double& lon1,
                          const double& lat2, const double& lon2);

  static int getDuration(const std::string& string_lat1, const std::string& string_lon1,
                         const std::string& string_lat2, const std::string& string_lon2);

  static std::string secondsToTime(int seconds);
  static int timeToSeconds(const std::string& timeStr);

  static std::vector<std::string> split(const std::string& str, char delimiter);
  static std::string getFirstWord(const std::string& str);
  static std::string trim(const std::string& str);

  static bool isNumber(const std::string &str);

};

#endif //RAPTOR_UTILS_H
