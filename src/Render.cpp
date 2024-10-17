#include "Render.h"

void Render::saveVideo()
{
    //savepPicture();
    const char* filename = "C:\\Dev\\render\\MyGif.gif";
    GifWriter writer = {};
    QSize size = frames[0].size();
    GifBegin(&writer, filename, size.width(), size.height(), 2, 8, true);
    for (auto frame : frames)
    {
        uint8_t* image = frame.bits();
        GifWriteFrame(&writer, image, size.width(), size.height(), 2, 8, true);
    }
    GifEnd(&writer);
}

void Render::savepPicture()
{
    int frameNumber = 0;
    outputFilename += "frame%1.png";
    for (auto f : frames)
    {
        
        QString filename = QString(outputFilename.c_str()).arg(frameNumber++, 4, 10, QChar('0'));
        f.save(filename, "PNG");
    }
}

void Render::addframe(QImage frame)
{
    frames.push_back(frame);
}