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
/*   Giorgio Panin                                                           */
/*   Thorsten R�der                                                          */
/** END LICENCE **************************************************************/

#ifndef __OPENTL_CORE_EXCEPTION_H__
#define __OPENTL_CORE_EXCEPTION_H__

#include <string>

#include <macros.h>

#ifdef DEBUG
#include <iostream>
#endif

/**
 * \brief OpenTL basic Exception class for wrapping around std::exception
 */

class OPENTL_API Exception : public std::exception
{

    public:
        Exception(std::string msg = "unspecified error") throw() : mMsg(msg)
        {
#ifdef DEBUG
            // if compiled in debug mode, we also print the "what()"-string to the console
            std::cerr << "Exception Msg: " << this->what() << std::endl;
#endif
        }

        ~Exception() throw() {}

        virtual const char* what() const throw()
        {
            return mMsg.c_str();
        }

        virtual const std::string& whatString() const throw()
        {
            return mMsg;
        }

    private:
        std::string mMsg;
};

#endif
