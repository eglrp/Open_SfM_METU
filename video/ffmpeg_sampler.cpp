

#include <vector>
#include <iostream>
#include <stdio.h>
#include <string>


#include "Open_SfM_METU/video/ffmpeg_sampler.hpp"
#include "Open_SfM_METU/image/image_IO.hpp"



namespace Open_SfM_METU {
namespace video {



FfmpegSampler::~FfmpegSampler(){

}

void SaveFrame(AVFrame *pFrame, int width, int height, int iFrame, std::string& outPath) {
	FILE *pFile;
	char szFilename[32];
	int  y;

	// Open file
	// /Users/akincaliskan/Documents/workspace_research/Descriptor_Learning/Experiments/Descriptor_Learning/demo/sample_output_video/

	std::stringstream ss;
	ss << (iFrame);
	std::string str_number = ss.str();

	std::string full_name_temp =  outPath + std::string("frame_") + str_number + std::string(".ppm");
	//std::string full_name_temp =  outPath + str_number + std::string(".ppm");

	//sprintf(szFilename, "frame%d.ppm", iFrame);

	//pFile=fopen(szFilename, "wb");
	pFile=fopen(full_name_temp.c_str(), "wb");
	if(pFile==NULL)
	return;


	// Write header
	fprintf(pFile, "P6\n%d %d\n255\n", width, height);

	// Write pixel data
	for(y=0; y<height; y++)
	fwrite(pFrame->data[0]+y*pFrame->linesize[0], 1, width*3, pFile);

	// Close file
	fclose(pFile);
}


bool FfmpegSampler::SampleVideo(const char * inputPath, const char * outputPath){


	std::cout << "inside ffmpeg video sampler " << std::endl;

	AVFormatContext *pFormatCtx = NULL;
	int             i, videoStream;
	AVCodecContext  *pCodecCtx = NULL;
	AVCodec         *pCodec = NULL;
	AVFrame         *pFrame = NULL; 
	AVFrame         *pFrameRGB = NULL;
	AVPacket        packet;
	int             frameFinished;
	int             numBytes;
	uint8_t         *buffer = NULL;
 
 	AVDictionary    *optionsDict = NULL;
	struct SwsContext      *sws_ctx = NULL;

	// Register all formats and codecs
	av_register_all();

	// Open video file
	if(avformat_open_input(&pFormatCtx, inputPath, NULL, NULL)!=0)
		return -1; // Couldn't open file


	 // Retrieve stream information
	if(avformat_find_stream_info(pFormatCtx, NULL)<0)
		return -1; // Couldn't find stream information

	// Dump information about file onto standard error
	av_dump_format(pFormatCtx, 0, inputPath, 0);

	// Find the first video stream
	videoStream=-1;
	for(i=0; i<pFormatCtx->nb_streams; i++)
		if(pFormatCtx->streams[i]->codec->codec_type==AVMEDIA_TYPE_VIDEO) {
		  videoStream=i;
		  break;
		}
	if(videoStream==-1)
		return -1; // Didn't find a video stream

	// Get a pointer to the codec context for the video stream
	pCodecCtx=pFormatCtx->streams[videoStream]->codec;

	// Find the decoder for the video stream
	pCodec=avcodec_find_decoder(pCodecCtx->codec_id);

	std::cout << "codec type " << pCodec << std::endl;
	
	if(pCodec==NULL) {
		fprintf(stderr, "Unsupported codec!\n");
		return -1; // Codec not found
	}
	// Open codec
	if(avcodec_open2(pCodecCtx, pCodec, &optionsDict)<0)
		return -1; // Could not open codec

	// Allocate video frame
	pFrame=av_frame_alloc();

	// Allocate an AVFrame structure
	pFrameRGB=av_frame_alloc();
	if(pFrameRGB==NULL)
		return -1;

	// Determine required buffer size and allocate buffer
	numBytes=avpicture_get_size(AV_PIX_FMT_RGB24, pCodecCtx->width, pCodecCtx->height);

	buffer=(uint8_t *)av_malloc(numBytes*sizeof(uint8_t));
	
		
	sws_ctx = sws_getContext(
		    pCodecCtx->width,
		    pCodecCtx->height,
		    pCodecCtx->pix_fmt,
		    pCodecCtx->width,
		    pCodecCtx->height,
		    AV_PIX_FMT_RGB24,
		    SWS_BILINEAR,
		    NULL,
		    NULL,
		    NULL
		);

	std::stringstream outPath_ss;
	std::string 	  outPath_str;
	outPath_ss << outputPath ;
	outPath_ss >> outPath_str;

	std::cout << "outputh path is " << outputPath << std::endl;

	// Assign appropriate parts of buffer to image planes in pFrameRGB
	// Note that pFrameRGB is an AVFrame, but AVFrame is a superset
	// of AVPicture
	avpicture_fill((AVPicture *)pFrameRGB, buffer, AV_PIX_FMT_RGB24,
		 pCodecCtx->width, pCodecCtx->height);

	  // Read frames and save first five frames to disk
	  i=0;
	  while(av_read_frame(pFormatCtx, &packet)>=0) {
	    // Is this a packet from the video stream?
	    if(packet.stream_index==videoStream) {
	      
	      int len; 

	      // Decode video frame
	      len = avcodec_decode_video2(pCodecCtx, pFrame, &frameFinished, &packet);

			if (len < 0) {
				std::cout << "error in decoding frame " << std::endl;
			}
	      
	      // Did we get a video frame?
			if(frameFinished) {
				// Convert the image from its native format to RGB
				sws_scale
				(
				    sws_ctx,
				    (uint8_t const * const *)pFrame->data,
				    pFrame->linesize,
				    0,
				    pCodecCtx->height,
				    pFrameRGB->data,
				    pFrameRGB->linesize
				);


				// Save the frame to disk
				if(++i<=100 && (i%5 == 0 )){
					SaveFrame(pFrameRGB, pCodecCtx->width, pCodecCtx->height, i, outPath_str);
					std::cout << "frame is saved " << std::endl; 
				}
			}


	    
	    }
	    
	    // Free the packet that was allocated by av_read_frame
	    av_free_packet(&packet);


	  }
	

	// Free the RGB image
	av_free(buffer);
	av_free(pFrameRGB);

	// Free the YUV frame
	av_free(pFrame);

	// Close the codec
	avcodec_close(pCodecCtx);

	// Close the video file
	avformat_close_input(&pFormatCtx);


}


}
}