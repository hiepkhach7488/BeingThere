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

#ifndef FILESYSTEM_H_
#define FILESYSTEM_H_

#include <iostream>
#include <string>
#include <vector>

#include <macros.h>


/**
 * Provides platform independent implementation of some common filesystem operations.
 * The class is basically a wrapper for boost-filesystem calls.
 */

class OPENTL_API Filesystem
{

    public:

        /**
         * @brief list files in a directory concerning the given filter (postfix)
         */
        static std::vector<std::string> ls(std::string dir, std::string extension);

        /**
         * @brief list contents of a directory
         * push_back() of the results on files- and subdirs-vector
         */
        static void ls(std::string dir, std::vector<std::string>& files, std::vector<std::string>& subdirs);

        /**
         * @brief remove a file or a directory with all its contents
         */
        static bool rm(std::string file);

        /**
         * @brief creates a directory with the given name (relative to current path or absolute)
         */
        static bool mkdir(std::string dir);

        /**
         * @brief checks if file or directory exists
         */
        static bool exists(std::string name);

        /**
         * @brief copy a file (or directory) from src to dst
         * @throws an exception if dst already exists
         * @return true if dst exists afterwards
         */
        static bool cp(std::string src, std::string dst);

        /**
         * @brief move/rename a file (or directory) from src to dst
         * ATTENTION: deletes an existing dst
         * @return true if dst exists afterwards
         */
        static bool mv(std::string src, std::string dst);


};


#endif /*FILESYSTEM_H_*/
