#include "Lines.h"


void Line::draw(Vis::Visualization& vis)
{
	glLineWidth(width);
	glBegin(GL_LINES);
	glColor3d(color.x, color.y, color.z);
	glVertex3dv(start);
	glVertex3dv(end);
	glEnd();
}
void Line::drawWithNames(Vis::Visualization& vis) const
{
	//TODO: implementation this
}
Vec Line::postSelection(const int p)
{
	return Vec();
}
void Line::movement(int selected, const Vector& position)
{
	//TODO: implementation this
}
void Line::rotate(int selected, Vec angel)
{
	//TODO: implementation this
}


void Line::scale(float scale)
{
	width *= scale;
}
void Line::animate(float time)
{
	//TODO: implementation this
}

void Line::setStart(Vec& _start) { start = _start; }

void Line::setEnd(Vec& _end) { end = _end; }

Vec Line::getStart() { return start; }

Vec Line::getEnd() { return end; }

void Line::setcolor(Vec& _color) { color = _color; }


void Line::setCameraFocus(Vector& min, Vector& max)
{

}

void DeltaLines::draw(Vis::Visualization& vis)
{
	for (auto line : lines)
	{
		line->draw(vis);
	}
}
void DeltaLines::drawWithNames(Vis::Visualization& vis) const
{
	for (auto line : lines)
	{
		line->drawWithNames(vis);
	}
}
Vec DeltaLines::postSelection(const int p)
{
	return Vec();
}
void DeltaLines::movement(int selected, const Vector& position)
{

}
void DeltaLines::rotate(int selected, Vec angel)
{

}
void DeltaLines::animate(float time)
{
	for (auto line : lines)
	{
		line->animate(time);
	}
}
void DeltaLines::scale(float scale)
{
	for (auto line : lines)
	{
		line->scale(scale);
	}
}

void DeltaLines::setDeltas(std::vector<Eigen::Vector4d> deltas)
{
	for (int i = 0; i < lines.size(); i++)
	{
		Vec newposition = lines[i]->getEnd() + Vec(deltas[i].data());
		lines[i]->setEnd(newposition);
	}
}


void DeltaLines::addLine(std::shared_ptr<Line> line)
{
	lines.push_back(line);
}

void DeltaLines::setCameraFocus(Vector& min, Vector& max)
{

}
