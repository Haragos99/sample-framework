#include "Render.h"



void Render::encodeFrame(AVCodecContext* codecCtx, AVFrame* frame, AVPacket* pkt, AVFormatContext* formatCtx, AVStream* stream) {
    // Send frame to the encoder
    if (avcodec_send_frame(codecCtx, frame) >= 0) {
        // Receive the encoded packet
        while (avcodec_receive_packet(codecCtx, pkt) >= 0) {
            pkt->stream_index = stream->index;
            av_packet_rescale_ts(pkt, codecCtx->time_base, stream->time_base);
            av_interleaved_write_frame(formatCtx, pkt);
            av_packet_unref(pkt);
        }
    }
}

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

/*
cv::Mat Render::QImageToCvMat(const QImage& inImage)
{
    cv::Mat mat(inImage.height(), inImage.width(), CV_8UC4,
        const_cast<uchar*>(inImage.bits()), inImage.bytesPerLine());

    return mat;
}
*/
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