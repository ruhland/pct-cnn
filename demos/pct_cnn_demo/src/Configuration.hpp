#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP
#include <string>
enum FILTERMETHOD {NOFILTER,VOXELGRIDFILTER};
enum FEATUREFORMAT {PERFECTFEATURE,XYZRGBFEATURE,PFHFEATURE};

class Configuration{
public:

	Configuration();
	void loadFromFile(const std::string  &filename);
	FILTERMETHOD getFilterMethod();
	int getNearestNeighborsToSearch();
	FEATUREFORMAT getFeatureFormat();
	float getFilterLeafSize();
private:
	FILTERMETHOD filter;
	FEATUREFORMAT featureFormat;
	int nearestNeighborsToSearch;
	float filterLeafSize;

	void setDefaultConfigurations();
	std::string trim(const std::string& string);
	void assignProperty(const std::string& property, const std::string& value);
};
#endif
