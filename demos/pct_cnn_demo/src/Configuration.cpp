#include "Configuration.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <algorithm>

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
		std::string property = trim(line.substr(0, space));
		std::string value = trim(line.substr(space));
		std::transform(property.begin(), property.end(), property.begin(), ::tolower);
		assignProperty(property, value);
	}
}

void Configuration::assignProperty(const std::string& property,
		const std::string& value) {
	std::cerr << " ASSIGN Property " << property << "=" << value << std::endl;
	if (property.compare("filtermethod") == 0) {
		if (value.compare("NOFILTER") == 0)
			filter = NOFILTER;
		else if (value.compare("VOXELGRIDFILTER") == 0)
			filter = VOXELGRIDFILTER;
		else
			std::cerr << "WRONG Filter Method" << value << std::endl;
	} else if (property.compare("featureformat") == 0) {
		if (value.compare("PERFECTFEATURE") == 0)
			featureFormat = PERFECTFEATURE;
		else if (value.compare("PFHFEATURE") == 0)
			featureFormat = PFHFEATURE;
                else if (value.compare("RIFTFEATURE") == 0)
                        featureFormat = RIFTFEATURE;
		else if (value.compare("XYZRGBFEATURE") == 0)
			featureFormat = XYZRGBFEATURE;
		else
			std::cerr << "WRONG Feature Format" << value << std::endl;
	} else if (property.compare("nearestneighborstosearch") == 0) {
		try {
			nearestNeighborsToSearch = std::stoi(value);
		} catch (const std::invalid_argument& e) {
			std::cerr << "Wrong Argument " << value << std::endl;
		}

	} else if (property.compare("filterleafsize") == 0) {
		try {
			filterLeafSize = std::stof(value);
		} catch (const std::invalid_argument& e) {
			std::cerr << "Wrong Argument " << value << std::endl;
		}
	} else {
		std::cerr << "Unknown Property added to map " << property<<"="<<value << std::endl;
		map[property]=value;
	}

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
float Configuration::getFloat(const std::string& p) {
	float ret = 0;
	try {
		ret = std::stof(map[p]);
	} catch (const std::invalid_argument& e) {
		ret = nan("");
	}
	return ret;
}

int Configuration::getInt(const std::string& p) {
	int ret = 0;
	try {
		ret = std::stoi(map[p]);
	} catch (const std::invalid_argument& e) {
		ret = -1;
	}
	return ret;
}


bool Configuration::getBool(const std::string& p) {
	std::string &value=map[p];
	if(value.compare("YES")==0 || value.compare("yes")==0|| value.compare("on")==0 ||value.compare("ON")==0 ||value.compare("1")==0)
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
