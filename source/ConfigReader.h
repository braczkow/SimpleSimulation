#pragma once

#include "RoboParts.h"
#include "json/json.h"
#include <map>

namespace robo
{
typedef std::map<std::string, std::string> ConfigType;

struct RoboConfig
{
	struct RoboPartDescription
	{
		RoboPartDescription() :
			color(Color(0, 0, 0, 0)),
			x(0.0f), y(0.0f), angle(0.0f) {}

		robo::Color color;
		float x, y;
		float angle;
	};

	std::map<std::string, RoboPartDescription> roboParts;
};

struct DebugConfig
{
	DebugConfig() :
		debugLevel(0), autorun(1) {}

	int debugLevel;
	bool autorun;
};

class ConfigReader
{
public:
	ConfigReader() {}
	~ConfigReader() {}

	bool tryParse(std::string filePath);

	DebugConfig getDebugConfig();

	RoboConfig getRoboConfig();

private:
	DebugConfig debugConfig;
	RoboConfig roboConfig;
};

}

