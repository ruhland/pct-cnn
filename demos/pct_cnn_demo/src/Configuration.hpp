#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP
#include <string>
enum FILTERMETHOD {NOFILTER,VOXELGRIDFILTER};

class Configuration{
public:

	Configuration();
	void loadFromFile(const std::string  &filename);
	FILTERMETHOD getFilterMethod();
	int getNearestNeighborsToSearch();
private:
	FILTERMETHOD filter;
	int nearestNeighborsToSearch;
	void setDefaultConfigurations();
	std::string trim(const std::string& string);
	void assignProperty(const std::string& property, const std::string& value);
};
#endif
