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
    int fps = 30;
    // Setup the output format and codec
    AVFormatContext* formatCtx = nullptr;
    avformat_alloc_output_context2(&formatCtx, nullptr, nullptr, outputFilename.c_str());

    const AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!codec) {
        //std::cerr << "Codec not found!" << std::endl;
        return;
    }

    AVStream* stream = avformat_new_stream(formatCtx, nullptr);
    AVCodecContext* codecCtx = avcodec_alloc_context3(codec);

    codecCtx->codec_id = codec->id;
    codecCtx->width = frames[0].width();
    codecCtx->height = frames[0].height();
    codecCtx->time_base = { 1, fps };  // 1/fps (frame duration)
    codecCtx->framerate = { fps, 1 };
    codecCtx->gop_size = 12;
    codecCtx->pix_fmt = AV_PIX_FMT_YUV420P;

    // Open codec
    if (avcodec_open2(codecCtx, codec, nullptr) < 0) {
        //std::cerr << "Could not open codec" << std::endl;
        return;
    }

    // Allocate video frames
    AVFrame* frame = av_frame_alloc();
    frame->format = codecCtx->pix_fmt;
    frame->width = codecCtx->width;
    frame->height = codecCtx->height;
    av_frame_get_buffer(frame, 32);

    AVPacket* pkt = av_packet_alloc();

    // Open the output file
    if (!(formatCtx->flags & AVFMT_NOFILE)) {
        avio_open(&formatCtx->pb, outputFilename.c_str(), AVIO_FLAG_WRITE);
    }

    // Write the file header
    avformat_write_header(formatCtx, nullptr);

    // Convert QImage to YUV420P and encode each frame
    SwsContext* swsCtx = sws_getContext(codecCtx->width, codecCtx->height, AV_PIX_FMT_RGB24,
        codecCtx->width, codecCtx->height, AV_PIX_FMT_YUV420P,
        SWS_BILINEAR, nullptr, nullptr, nullptr);

    for (size_t i = 0; i < frames.size(); ++i) {
        QImage image = frames[i].convertToFormat(QImage::Format_RGB888);

        uint8_t* srcData[1] = { const_cast<uint8_t*>(image.bits()) };
        int srcLinesize[1] = { static_cast<int>(image.bytesPerLine()) };

        // Scale and convert to YUV420P
        sws_scale(swsCtx, srcData, srcLinesize, 0, codecCtx->height, frame->data, frame->linesize);
        frame->pts = i;

        // Encode the frame
        encodeFrame(codecCtx, frame, pkt, formatCtx, stream);
    }

    // Write the trailer and close the output
    av_write_trailer(formatCtx);

    // Cleanup
    avcodec_free_context(&codecCtx);
    av_frame_free(&frame);
    av_packet_free(&pkt);
    sws_freeContext(swsCtx);
    avio_closep(&formatCtx->pb);
    avformat_free_context(formatCtx);
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