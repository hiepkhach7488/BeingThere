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

#ifndef PLAINFILEPARSER_H_
#define PLAINFILEPARSER_H_

#include <string>
#include <fstream>
#include <macros.h>

//using namespace std;

class OPENTL_API PlainFileParser
{

    public:

        /**
         * open a file handle for the specified file
         */
        PlainFileParser(std::string filename)
        {
            //std::fstream input(filename.c_str());
            //std::fstream input;
            input.open(filename.c_str());
            input.exceptions(std::ios::badbit | std::ios::failbit | std::ios::goodbit);
        }

        /**
         * close file handle
         */
        virtual ~PlainFileParser()
        {
            input.close();
        }

        /**
         * read a whole line from the file-handle
         */

        std::string readLine()
        {
            std::string nextline = "";
            std::getline(input, nextline);
            return nextline;
        }

        /**
         * read next token from file-handle
         */
        template<typename T>
        inline
        T read()
        {
            T token;
            input >> token;
            return token;
        }

        /**
         * check status
         */
        bool ready()
        {
            return input.good();
        }

    protected:

    private:
        /**
         * the file handle
         */
        std::ifstream input;

};

#endif /*PLAINFILEPARSER_H_*/
