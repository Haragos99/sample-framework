#pragma once
#include <QImage>
#include <vector>

class Render
{
public:
	Render();



	~Render();

private:
	QImage frame;
	std::vector<QImage> frames;

};

Render::Render()
{
}

Render::~Render()
{
}