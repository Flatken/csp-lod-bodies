#ifndef CSP_WMS_UTILS_HPP
#define CSP_WMS_UTILS_HPP

#include <string>
#include <boost/filesystem.hpp>
#include <iostream>
#include <vector>
#include <regex>

#include "../../../src/cs-utils/utils.hpp"
#include "../../../src/cs-utils/convert.hpp"

namespace csp::lodbodies {

struct timeInterval {
    boost::posix_time::ptime startTime;
    boost::posix_time::ptime endTime;
    std::string mFormat;
    int mIntervalDuration;
};

struct Wms {
    std::string mName;
    std::string mCopyringht;
    std::string mUrl;
    int mWidth;
    int mHeight;
    std::optional<std::string> mTime;
    std::optional<int> preFetch;
    std::string mLayers;
};

std::string timeToString(std::string format, boost::posix_time::ptime time);
void timeDuration(std::string isoString, int &duration, std::string &format);
void parseIsoString(std::string isoString, std::vector<timeInterval> &timeIntervals);
bool timeInIntervals(boost::posix_time::ptime time, std::vector<timeInterval> &timeIntervals, boost::posix_time::time_duration &timeSinceStart, int &intervalDuration, std::string &format);
boost::posix_time::ptime getStartTime(boost::posix_time::ptime time, int intervalDuration);

}// namespace csp::simpleWmsBodies

#endif // CSP_WMS_UTILS_HPP