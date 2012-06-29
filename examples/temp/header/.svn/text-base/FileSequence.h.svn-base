/** BEGIN LICENCE ************************************************************/
/* Copyright 2008-2009 Technische Universitaet Muenchen.                     */
/* All Rights Reserved.                                                      */
/* Permission to use, copy, modify OR distribute this software and its       */
/* documentation for educational, research and non-profit purposes, without  */
/* fee, and without a written agreement is hereby granted, provided that the */
/* above copyright notice and the following three paragraphs appear in all   */
/* copies.                                                                   */
/*                                                                           */
/* IN NO EVENT SHALL THE TECHNISCHE UNIVERSITAET MUENCHEN BE                 */
/* LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR         */
/* CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE         */
/* USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE TECHNISCHE        */
/* UNIVERSITAET MUENCHEN HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH        */
/* DAMAGES.                                                                  */
/*                                                                           */
/* THE TECHNISCHE UNIVERSITAET MUENCHEN SPECIFICALLY DISCLAIMS ANY           */
/* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF      */
/* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE       */
/* PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE TECHNISCHE             */
/* UNIVERSITAET MUENCHEN HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE,          */
/* SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.                         */
/*                                                                           */
/* Contact author: Giorgio Panin                                             */
/* E-mail: panin@in.tum.de                                                   */
/* If you have any questions regarding this agreement or the licensing of    */
/* this technology, please send them to the contact author above.            */
/*                                                                           */
/* Author(s):                                                                */
/*   Thomas Mueller                                                          */
/** END LICENCE **************************************************************/

/*Modified By Stanley (nguy0066@e.ntu.edu.sg)*/

#ifndef FileSequence_H_
#define FileSequence_H_

#include <string>
#include <vector>
#include <assert.h>
#include <macros.h>

#include <boost/algorithm/string.hpp>

class OPENTL_API FileSequence
{

    public:
        FileSequence();
        virtual ~FileSequence() {};

        /**
         * Initialize the sensor.
         * By default
         *  - files are read from the current directory (dir = "."),
         *  - prefixes of filenames have zero length (prefix = ""),
         *  - file extension is arbitrary (extension = "")
         */
        virtual void init()
        {
            init(".");
        }

        virtual void init(std::string dir, std::string extension = "", std::string prefix = "");
        virtual void open();
        virtual void captureStart() {}

        virtual void captureNext() = 0;
        virtual void captureStop() {}

        virtual void close() {}

        virtual long getTimeStamp() = 0;
        virtual std::string currentFile();

        virtual bool ready();
        virtual unsigned int currentIdx()
        {
            return mCurrentIdx;
        }

        virtual void restartFromIdx(unsigned int idx)
        {
            assert(idx < mFiles.size() && "Start index must be less or equal to number of files read.");
            mCurrentIdx = idx;
        }

    public:
        /** @brief extract the timestamp from files '[prefix][timestamp][extension]' **/
        long extractTimeStamp(std::string filename);
        /** @brief list files and sort them by timestamp **/
        std::vector<std::string> listFiles();

        /// base directory containing files
        std::string mBaseDirectory;
        /// current data file
        std::string mCurrentFile;
        /// index of the current data file
        unsigned int mCurrentIdx;
        /// data files must have this extension
        std::string mFileExtension;
        /// prefix of timestamp in data file names
        std::string mFilePrefix;
        /// sorted list of data files (by timestamp)
        std::vector<std::string> mFiles;

    private:
};

#endif /*FileSequence_H_*/
