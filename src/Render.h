#pragma once

#include <QImage>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <filesystem>
#include "gif.h"

class Render
{
public:
	Render() { outputFilename = "C:\\Dev\\render\\"; }

	void saveVideo();
	void addframe(QImage frame);
	int sizeframes() { return frames.size(); }
	void savepPicture();

private:
	
	std::vector<QImage> frames;
	std::string outputFilename;

};
