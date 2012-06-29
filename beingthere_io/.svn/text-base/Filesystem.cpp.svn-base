
#include <boost/filesystem.hpp>
//#include <boost/filesystem/operations.hpp> // includes boost/filesystem/path.hpp
//#include <boost/filesystem/fstream.hpp>
#include <Exception.h>
#include "Filesystem.h"
#include <boost/version.hpp>

//#define DEBUG_OUT

#if BOOST_VERSION > 103501
#define NEW_BOOST_VERSION 1
#endif


std::vector<std::string> Filesystem::ls(std::string dir, std::string filterExtension)
{
#ifdef DEBUG_OUT
    std::cout << "Filesystem::ls(\"" << dir << "\", \"" << filterExtension << "\")" << std::endl;
#endif
    std::vector<std::string> files = std::vector<std::string>();
    boost::filesystem::path full_path(boost::filesystem::initial_path());
    full_path = boost::filesystem::system_complete(boost::filesystem::path(dir, boost::filesystem::native));

    if (!boost::filesystem::exists(full_path) || dir == "")
    {
        throw Exception("\nNot found: " + full_path.string()); 
    }

    if (boost::filesystem::is_directory(full_path))
    {
        boost::filesystem::directory_iterator end_iter;

        for (boost::filesystem::directory_iterator dir_itr(full_path); dir_itr != end_iter; ++dir_itr)
        {
            try
            {
                // is this a file?
                if (boost::filesystem::is_regular(*dir_itr))
                {
#ifndef NEW_BOOST_VERSION
                    std::string name = dir_itr->leaf();
#else
                    std::string name = dir_itr->path().filename().string();
#endif

                    if (name.size() >= filterExtension.size())
                    {
                        // does it have the right extension (postfix)?
                        if (name.substr(name.size() - filterExtension.size(), filterExtension.size()) == filterExtension)
                        {
                            files.push_back(name);
#ifdef DEBUG_OUT
                            std::cout << "  File: " << name << std::endl;
#endif
                        }
                    }
                }
            }
            catch (const std::exception & ex)
            {
#ifndef NEW_BOOST_VERSION
                throw Exception(dir_itr->leaf() + " " + ex.what());
#else
                throw Exception(dir_itr->path().filename().string() + " " + ex.what());
#endif
            }
        }
    }
    else // must be a file
    {
        throw Exception("\n'" + dir + "' is a file, but must be a directory.");
    }

    return files;
}

void Filesystem::ls(std::string dir, std::vector<std::string>& files, std::vector<std::string>& subdirs)
{

    boost::filesystem::path full_path(boost::filesystem::initial_path());

    full_path = boost::filesystem::system_complete(boost::filesystem::path(dir, boost::filesystem::native));

//	unsigned long file_count = 0;
//	unsigned long dir_count = 0;
//	unsigned long err_count = 0;

    if (!boost::filesystem::exists(full_path) || dir == "")
    {
        //std::cout << "\nNot found: " << full_path.native_file_string() << std::endl;
		throw Exception("\nNot found: " + full_path.string());
        //return 1;
    }

    if (boost::filesystem::is_directory(full_path))
    {
        //std::cout << "\nIn directory: " << full_path.native_directory_string() << "\n\n";
        boost::filesystem::directory_iterator end_iter;

        for (boost::filesystem::directory_iterator dir_itr(full_path); dir_itr != end_iter; ++dir_itr)
        {
            try
            {
                if (boost::filesystem::is_directory(*dir_itr))
                {
                    //++dir_count;
#ifndef NEW_BOOST_VERSION
                    subdirs.push_back(dir_itr->leaf());
#else
                    subdirs.push_back(dir_itr->path().filename().string());
#endif
                    //std::cout << dir_itr->leaf()<< " [directory]\n";
                }
                else
                {
                    //++file_count;
#ifndef NEW_BOOST_VERSION
                    files.push_back(dir_itr->leaf());
#else
                    files.push_back(dir_itr->path().filename().string());
#endif
                    //std::cout << dir_itr->leaf() << "\n";
                }
            }
            catch (const std::exception & ex)
            {
                //++err_count;
#ifndef NEW_BOOST_VERSION
                throw Exception(dir_itr->leaf() + " " + ex.what());
#else
				throw Exception(dir_itr->path().filename().string() + " " + ex.what());
#endif
                //std::cout << dir_itr->leaf() << " " << ex.what() << std::endl;
            }
        }

//		std::cout << "\n" << file_count << " files\n"
//		  << dir_count << " directories\n"
//		  << err_count << " errors\n";
    }
    else // must be a file
    {
        throw Exception("\n'" + dir + "' is a file, but must be a directory.");
//		std::cout << "\nFound: " << full_path.native_file_string() << "\n";
    }
}

bool Filesystem::exists(std::string name)
{
    boost::filesystem::path full_path(boost::filesystem::initial_path());
    full_path = boost::filesystem::system_complete(boost::filesystem::path(name, boost::filesystem::native));

    if (boost::filesystem::exists(full_path))
        return true;

    return false;
}

bool Filesystem::rm(std::string name)
{
    try
    {
        boost::filesystem::path p(name);
        // remove directory content if 'name' is a directory

        if (boost::filesystem::is_directory(p))
            boost::filesystem::remove_all(p);

        // delete the file (or empty dir)
#ifndef NEW_BOOST_VERSION
        success = boost::filesystem::remove(p);

#else
        boost::filesystem::remove(p);

#endif
    }
    catch (const std::exception& ex)
    {
        throw Exception("Filesystem exception: rm(" + name + ") failed with " + ex.what());
    }

#ifndef NEW_BOOST_VERSION
    return success;

#else
    return true;	// newer boost versions only throw an exception.

#endif
}

bool Filesystem::mkdir(std::string dir)
{
    boost::filesystem::path p(dir);
    return boost::filesystem::create_directory(p);
}

bool Filesystem::cp(std::string src, std::string dst)
{
    try
    {
        boost::filesystem::path source(src);
        boost::filesystem::path destination(dst);
        boost::filesystem::copy_file(source, destination);
    }
    catch (const std::exception&)
    {
        throw Exception("Filesystem exception: cp(" + src + ", " + dst + ") failed. Does " + dst + " already exist?");
    }

    return exists(dst);
}

bool Filesystem::mv(std::string src, std::string dst)
{
    try
    {
        boost::filesystem::path source(src);
        boost::filesystem::path destination(dst);
        boost::filesystem::rename(source, destination);
    }
    catch (const std::exception& ex)
    {
        throw Exception("Filesystem exception: mv(" + src + ", " + dst + ") failed with " + ex.what());
    }

    return exists(dst);
}
