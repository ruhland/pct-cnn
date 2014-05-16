#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP
#include <string>
#include <map>
#include <algorithm>
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
	std::string get(const std::string&);
	int getInt(const std::string& p,int defaultvalue=0);
	float getFloat(const std::string&, float defaultvalue=nan(""));
	bool getBool(const std::string &);
private:
	FILTERMETHOD filter;
	FEATUREFORMAT featureFormat;
	int nearestNeighborsToSearch;
	float filterLeafSize;
	std::map<std::string,std::string> map;
	void setDefaultConfigurations();
	std::string trim(const std::string& string);
	void assignProperty(const std::string& property, const std::string& value);
};
#endif
