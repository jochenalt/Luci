/*
 * Configuration.cpp
 *
 *  Created on: 25.02.2015
 *      Author: JochenAlt
 */

#include "Configuration.h"
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include "Util.h"

using namespace std;

Configuration::Configuration() {
	configFileName = "luci-config.conf";
	modified = false;
};

void Configuration::saveConfigFile()
{
   using boost::property_tree::ptree;
   write_ini( configFileName, pt );
   modified = false;
}

void Configuration::saveIfModified() {
	if (modified)
		saveConfigFile();
}

void Configuration::loadConfigFile() {
	if (boost::filesystem::exists(configFileName))
		boost::property_tree::ini_parser::read_ini(configFileName, pt);
}

std::string Configuration::getOrCreateStr(std::string key, std::string defaultValue) {
	boost::optional<string> value = pt.get_optional<string>(key);
	if (!value.is_initialized())
	   putStr(key, defaultValue);

	return getStr(key);
}

void Configuration::putStr(std::string key, std::string value) {
	pt.put(key, value);
	modified = true;
}

std::string Configuration::getStr(std::string key) {
	return pt.get<std::string>(key);
}


float Configuration::getOrCreateFloat(std::string key, float defaultValue) {
	boost::optional<float> value = pt.get_optional<float>(key);
		if (!value.is_initialized())
		   putFloat(key, defaultValue);

		return getFloat(key);
}

void Configuration::putFloat(std::string key, float value) {
	pt.put(key, value);
	modified = true;
}

float  Configuration::getFloat(std::string key) {
	return pt.get<float>(key);
}

int Configuration::getOrCreateInt(std::string key, int defaultValue) {
	boost::optional<int> value = pt.get_optional<int>(key);
		if (!value.is_initialized())
		   putInt(key, defaultValue);

		return getInt(key);
}

void Configuration::putInt(std::string key, int value) {
	pt.put(key, value);
	modified = true;
}

int Configuration::getInt(std::string key) {
	return pt.get<int>(key);
}


void Configuration::loop() {
	static unsigned long lastTimeConfigHasBeenWritten_s= millis();
	unsigned long now_s= millis();

	if (modified) {
		if (now_s - lastTimeConfigHasBeenWritten_s > 5000) {
			saveIfModified();
			lastTimeConfigHasBeenWritten_s = now_s;
		}
	}
}
