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
/*   Thomas Mueller                                                           */
/** END LICENCE **************************************************************/

/*Modified By Stanley (nguy0066@e.ntu.edu.sg)*/

#ifndef IMAGESEQUENCE_H_
#define IMAGESEQUENCE_H_

#include <vector>
#include <string>

#include <FileSequence.h>
#include <macros.h>

#include <cv.h>
#include <highgui.h>
#include <openni_capture.h>

class OPENTL_API ImageSequence : public FileSequence, public pcl::gpu::CaptureOpenNI
{

    public:
        ImageSequence();
        
        virtual ~ImageSequence();

        virtual void init()
        {
            FileSequence::init();
        }

        virtual void init(std::string dir, std::string extension, std::string prefix = "")
        {
            FileSequence::init(dir, extension, prefix);
        }

        virtual void open();
        virtual void captureStart()
        {
            FileSequence::captureStart();
        }

        virtual void captureStop()
        {
            FileSequence::captureStop();
        }

        virtual void close()
        {
        }

        virtual void captureNext();
        virtual long getTimeStamp();

        // the pointer to the image-member variable
        virtual IplImage* image() const;


    private:
        unsigned long mCurrentTimestamp;
        bool mUndistort;

		int mWidth;
		int mHeight;

		IplImage* mFrame;
};


#endif /*IMAGESEQUENCE_H_*/