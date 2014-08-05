// disable warnings for deprecated code from pcl headers (fopen, etc.)
#ifdef _MSC_VER
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
// also disable specific warnings from MSVC Compiler on Windows
#pragma warning(disable: 4503 4305 4514 4711 4996)
#pragma warning(push, 1)
#endif

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <algorithm>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/trim_all.hpp>

#include "Configuration.hpp"
#include "Util.hpp"

// re-enable warnings for Windows
#ifdef _MSC_VER
#pragma warning(pop)
#endif

Configuration::Configuration() {
  setDefaultConfigurations();
}

void Configuration::setDefaultConfigurations() {
  filter = NOFILTER;
  filterLeafSize = 0.01f;
  nearestNeighborsToSearch = 5;
  featureFormat = XYZRGBFEATURE;
}

void Configuration::loadFromFile(const std::string &filename) {
  std::ifstream input(filename);
  std::string line;
  while (std::getline(input, line)) {
    size_t commentpos = line.find("#");
    if (commentpos != std::string::npos)
      line = line.substr(0, commentpos);
    line = trim(line);
    size_t space = line.find(" ");
    if (space == std::string::npos)
      continue;
    std::string property = boost::trim_copy(line.substr(0, space));
    std::string value = boost::trim_copy(line.substr(space));
    std::transform(property.begin(), property.end(), property.begin(), ::tolower);
    assignProperty(property, value);
  }
}

void Configuration::assignProperty(const std::string& property,
    const std::string& value) {
  if (boost::iequals(property, "filter")) {
    if (boost::iequals(value, "none"))
      filter = NOFILTER;
    else if (boost::iequals(value, "voxelgrid"))
      filter = VOXELGRIDFILTER;
    else
      std::cerr << "[assignProperty] Unknown filter: " << value << std::endl;
  } else if (boost::iequals(property, "feature")) {
    if (boost::iequals(value, "perfect"))
      featureFormat = PERFECTFEATURE;
    else if (boost::iequals(value, "pfh"))
      featureFormat = PFHFEATURE;
    else if (boost::iequals(value, "rift"))
      featureFormat = RIFTFEATURE;
    else if (boost::iequals(value, "xyzrgb"))
      featureFormat = XYZRGBFEATURE;
    else
      std::cerr << "[assignProperty] Unknown feature: " << value << std::endl;
  } else if (boost::iequals(property, "neighbors")) {
    try {
      nearestNeighborsToSearch = std::stoi(value);
    } catch (const std::invalid_argument&) {
      std::cerr << "[assignProperty] Not an integer: " << value << std::endl;
    }
  } else if (boost::iequals(property, "filterleafsize")) {
    try {
      filterLeafSize = std::stof(value);
    } catch (const std::invalid_argument&) {
      std::cerr << "[assignProperty] Not a float: " << value << std::endl;
    }
  } else {
    std::cerr << "[assignProperty] Unknown property added: " << property << " = " << value << std::endl;
    map[property] = value;
    return;
  }
  std::cout << "[assignProperty] " << property << " = " << value << std::endl;
}

std::string Configuration::trim(const std::string& string) {
  size_t start = string.find_first_not_of(" \t"); // First letter not " " or "\t"
  if (start == std::string::npos)
    return "";
  size_t end = string.find_last_not_of(" \t");
  return string.substr(start, end - start + 1);
}

std::string Configuration::get(const std::string& p) {
  return map[p];
}

float Configuration::getFloat(const std::string& p,float defaultvalue) {
  float ret = defaultvalue;
  try {
    ret = std::stof(map[p]);
  } catch (const std::invalid_argument& e) {
    ret = defaultvalue;
  }
  return ret;
}

int Configuration::getInt(const std::string& p,int defaultvalue) {
  int ret = defaultvalue;
  try {
    ret = std::stoi(map[p]);
  } catch (const std::invalid_argument& e) {
    ret = defaultvalue;
  }
  return ret;
}

bool Configuration::getBool(const std::string& property) {
  std::string &value = map[property];
  std::vector<std::string> truthy_values = {"yes", "on", "true", "1"};

  for (std::string &truthy : truthy_values)
    if (boost::iequals(value, truthy))
      return true;

  return false;
}

float Configuration::getFilterLeafSize() {
  return filterLeafSize;
}

FEATUREFORMAT Configuration::getFeatureFormat() {
  return featureFormat;
}

FILTERMETHOD Configuration::getFilterMethod() {
  return filter;
}

int Configuration::getNearestNeighborsToSearch() {
  return nearestNeighborsToSearch;
}
