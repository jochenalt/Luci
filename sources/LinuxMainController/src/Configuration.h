/*
 * Configuration.h
 *
 *  Created on: 25.02.2015
 *      Author: JochenAlt
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

class Configuration {
public:
	Configuration();

	virtual ~Configuration() {};
	static Configuration& getInstance() {
		static Configuration configuration;
		return configuration;
	}

	void loop();
	void saveConfigFile();
	void saveIfModified();
	void loadConfigFile();

	std::string getOrCreateStr(std::string key, std::string defaultValue);
	void putStr(std::string key, std::string value);
	std::string getStr(std::string key);

	float getOrCreateFloat(std::string key, float defaultValue);
	void putFloat(std::string key, float value);
	float getFloat(std::string key);

	int getOrCreateInt(std::string key, int defaultValue);
	void putInt(std::string key, int value);
	int getInt(std::string key);

	std::string configFileName;
	boost::property_tree::ptree pt;
	bool modified;
};

#endif /* CONFIGURATION_H_ */
