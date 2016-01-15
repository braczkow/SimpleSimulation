#include "ConfigReader.h"
#include <iostream>
#include <fstream>


namespace robo
{
robo::Color toColor(std::string colorName)
{
	Color c(1, 1, 1, 0);

	if (colorName == "red")
		c = Color(1, 0, 0, 0);
	else if (colorName == "green")
		c = Color(0, 1, 0, 0);
	else if (colorName == "blue")
		c = Color(0, 0, 1, 0);
	else if (colorName == "yellow")
		c = Color(1, 1, 0, 0);

	return c;
}


bool ConfigReader::tryParse(std::string filePath)
{
	Json::Value root;
	Json::Reader reader;
	std::ifstream in_file("../../config.json", std::ifstream::binary);
	auto suc = reader.parse(in_file, root);

	if (!suc)
		return false;
	if (root.isMember("debug"))
	{
		Json::Value debug = root["debug"];
		if (debug.isMember("autorun"))
		{
			debugConfig.autorun = debug["autorun"].asInt();
		}

		if (debug.isMember("logLevel"))
		{
			debugConfig.debugLevel = debug["logLevel"].asInt();
		}
	}

	if (root.isMember("robo"))
	{
		Json::Value robo = root["robo"];
		if (robo.isMember("parts"))
		{
			auto parts = robo["parts"];
			auto partNames = parts.getMemberNames();
			for (auto partName = partNames.begin(); partName != partNames.end(); partName++)
			{
				RoboConfig::RoboPartDescription roboPartDesc;
				roboPartDesc.name = *partName;

				auto partDesc = parts[*partName];
				if (partDesc.isMember("color"))
				{
					roboPartDesc.color = toColor(partDesc["color"].asString());
				}

				roboConfig.roboParts.push_back(roboPartDesc);
			}

		}

	}

}

DebugConfig ConfigReader::getDebugConfig()
{
	return debugConfig;
}

RoboConfig ConfigReader::getRoboConfig()
{
	return roboConfig;
}

} //namespace robo

