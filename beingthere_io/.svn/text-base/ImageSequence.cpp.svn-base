#include <iostream>
#include <cv.h>
#include <highgui.h>

#include <Filesystem.h>
#include <Exception.h>

#include "ImageSequence.h"

ImageSequence::ImageSequence()
{
    mCurrentTimestamp = 0;
    mUndistort = false;
	mWidth = mHeight = 0;
	mFrame = 0;
}

ImageSequence::~ImageSequence()
{
}

void ImageSequence::open()
{
    if (mFiles.size() > 0)
    {
        std::string firstFile = mBaseDirectory + "/" + mFiles.at(0);
		mFrame = cvLoadImage(firstFile.c_str(), CV_LOAD_IMAGE_UNCHANGED);

        assert(mFrame != NULL);

        mWidth = mFrame->width;
        mHeight = mFrame->height;
    }
    else
    {
        throw(Exception("Please make sure " + mBaseDirectory + " contains " + mFileExtension + " images."));
    }
}

void ImageSequence::captureNext()
{
	if(mCurrentIdx >= mFiles.size()) return;

    if (!ready()) restartFromIdx(0);

    mCurrentFile = mFiles.at(mCurrentIdx);

    mCurrentTimestamp = extractTimeStamp(mCurrentFile);

    std::string nextFileName = mBaseDirectory + "/" + mCurrentFile;

	mFrame = cvLoadImage(nextFileName.c_str(), CV_LOAD_IMAGE_UNCHANGED);

    mCurrentIdx++;
}

long ImageSequence::getTimeStamp()
{
    return mCurrentTimestamp;
}

IplImage* ImageSequence::image() const
{
	return mFrame;
}
