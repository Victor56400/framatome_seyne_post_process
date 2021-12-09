#include <forssea_utilities/DynamicParameters.hpp>


namespace forssea_utilities {
void addToMsg(const std::string &name, int value, dynamic_reconfigure::Config &conf) {
  conf.ints.emplace_back();
  conf.ints.back().name = name;
  conf.ints.back().value = value;
}

void addToMsg(const std::string &name, double value, dynamic_reconfigure::Config &conf) {
  conf.doubles.emplace_back();
  conf.doubles.back().name = name;
  conf.doubles.back().value = value;
}

void addToMsg(const std::string &name, bool value, dynamic_reconfigure::Config &conf) {
  conf.bools.emplace_back();
  conf.bools.back().name = name;
  conf.bools.back().value = value;
}

void addToMsg(const std::string &name, const std::string &value, dynamic_reconfigure::Config &conf) {
  conf.strs.emplace_back();
  conf.strs.back().name = name;
  conf.strs.back().value = value;
}

}