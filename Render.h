#pragma once
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/avutil.h>
#include <libswscale/swscale.h>
}
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
	//cv::Mat QImageToCvMat(const QImage& inImage);
	void encodeFrame(AVCodecContext* codecCtx, AVFrame* frame, AVPacket* pkt, AVFormatContext* formatCtx, AVStream* stream);
	

};
