#include <stdlib.h>
#include <iostream>
#include <PlainFileParser.h>
#include <Filesystem.h>
#include "FileSequence.h"


FileSequence::FileSequence() :
        mCurrentFile(""),
        mCurrentIdx(0),
        mFileExtension(""),
        mFilePrefix("")
{
}

void
FileSequence::init(std::string dir, std::string extension, std::string prefix)
{
    mBaseDirectory = dir;
    mFilePrefix = prefix;
    mFileExtension = extension;
    mFiles = listFiles();
}

void
FileSequence::open()
{
    if (mFiles.size() == 0)
    {
        std::cout << "Please make sure the directory '" << mBaseDirectory << "' contains data files." << std::endl;
        exit(0);
    }
}

bool
FileSequence::ready()
{
    return mCurrentIdx < mFiles.size();
}

std::string
FileSequence::currentFile()
{
    return mCurrentFile;
}

std::vector<std::string>
FileSequence::listFiles()
{
    // list all files with the given extension
    std::vector<std::string> temps = Filesystem::ls(mBaseDirectory, mFileExtension);
	std::vector<std::string> files;
	for(int i=0; i<temps.size(); ++i){
		if(boost::starts_with(temps[i], mFilePrefix))
			files.push_back(temps[i]);
	}
	temps.empty();
	
    // sort by timestamp

    for (unsigned int i = 0; i < files.size(); i++)
    {
        for (unsigned int j = i + 1; j < files.size(); j++)
        {
            long cmpi = extractTimeStamp(files[i]);
            long cmpj = extractTimeStamp(files[j]);

            if (cmpi > cmpj)
            {
                std::string tmp = files[i];
                files[i] = files[j];
                files[j] = tmp;
            }
        }
    }

    return files;
}

long
FileSequence::extractTimeStamp(std::string filename)
{

    return atol(filename.substr(mFilePrefix.size(), filename.size() - mFilePrefix.size() - mFileExtension.size()).c_str());
}

