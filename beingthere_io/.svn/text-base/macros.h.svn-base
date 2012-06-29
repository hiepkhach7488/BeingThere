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
/*   Thorsten Roeder                                                         */
/** END LICENCE **************************************************************/

#ifndef _LIB_MACROS_H_
#define _LIB_MACROS_H_



// THE FOLLOWING LICENSE IS VALID FOR THE CODE BELOW
/* This file is part of the KDE libraries
    Copyright (c) 2002-2003 KDE Team

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Library General Public
    License as published by the Free Software Foundation; either
    version 2 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Library General Public License for more details.

    You should have received a copy of the GNU Library General Public License
    along with this library; see the file COPYING.LIB.  If not, write to
    the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
    Boston, MA 02110-1301, USA.
*/

/**
 * @def LIB_DEPRECATED
 *
 * The LIB_DEPRECATED macro can be used to trigger compile-time warnings
 * with newer compilers when deprecated functions are used.
 *
 * For non-inline functions, the macro gets inserted at front of the
 * function declaration, right before the return type:
 *
 * \code
 * LIB_DEPRECATED void deprecatedFunctionA();
 * LIB_DEPRECATED int deprecatedFunctionB() const;
 * \endcode
 *
 * For functions which are implemented inline,
 * the LIB_DEPRECATED macro is inserted at the front, right before the return
 * type, but after "static", "inline" or "virtual":
 *
 * \code
 * LIB_DEPRECATED void deprecatedInlineFunctionA() { .. }
 * virtual LIB_DEPRECATED int deprecatedInlineFunctionB() { .. }
 * static LIB_DEPRECATED bool deprecatedInlineFunctionC() { .. }
 * inline LIB_DEPRECATED bool deprecatedInlineFunctionD() { .. }
 * \endcode
 *
 * You can also mark whole structs or classes as deprecated, by inserting the
 * LIB_DEPRECATED macro after the struct/class keyword, but before the
 * name of the struct/class:
 *
 * \code
 * class LIB_DEPRECATED DeprecatedClass { };
 * struct LIB_DEPRECATED DeprecatedStruct { };
 * \endcode
 *
 * \note
 * LIB_DEPRECATED cannot be used for constructors,
 * use LIB_CONSTRUCTOR_DEPRECATED instead.
 */

#ifdef __cplusplus
#  ifndef LIB_DEPRECATED
#    ifdef DECL_DEPRECATED
#      undef DECL_DEPRECATED
#    endif
#    if defined(LIB_NO_DEPRECATED)
#      define DECL_DEPRECATED
#    else
#      if defined(__GNUC__) && !defined(__INTEL_COMPILER) && (__GNUC__ - 0 > 3 || (__GNUC__ - 0 == 3 && __GNUC_MINOR__ - 0 >= 2))	// GNU compiler
#        define DECL_DEPRECATED __attribute__ ((__deprecated__))
#      elif defined(_MSC_VER) && (_MSC_VER >= 1300)		// newer MS compiler or Intel compiler
#        define DECL_DEPRECATED __declspec(deprecated)
#      else
#        define DECL_DEPRECATED		// every other compiler
#      endif
#    endif
#      define LIB_DEPRECATED DECL_DEPRECATED
#  endif

#  ifdef __APPLE__
#    define LIB_NO_DEPRECATED_CONSTRUCTORS
#  endif

#  ifndef LIB_DECL_CONSTRUCTOR_DEPRECATED
#    define LIB_DECL_CONSTRUCTOR_DEPRECATED DECL_DEPRECATED
#  endif
#endif	// __cplusplus

// code and description taken from KDE4 (TODO: check licence)
/**
 * @def LIB_CONSTRUCTOR_DEPRECATED
 *
 * The LIB_CONSTRUCTOR_DEPRECATED macro can be used to trigger compile-time
 * warnings with newer compilers when deprecated constructors are used.
 *
 * For non-inline constructors, the macro gets inserted at front of the
 * constructor declaration, right before the return type:
 *
 * \code
 * LIB_CONSTRUCTOR_DEPRECATED classA();
 * \endcode
 *
 * For constructors which are implemented inline,
 * the LIB_CONSTRUCTOR_DEPRECATED macro is inserted at the front,
 * but after the "inline" keyword:
 *
 * \code
 * LIB_CONSTRUCTOR_DEPRECATED classA() { .. }
 * \endcode
 */

#ifdef __cplusplus
#  ifndef LIB_CONSTRUCTOR_DEPRECATED
#    ifdef __GNUC__
#      if __GNUC__ == 3 && __GNUC_MINOR__ <= 3
#        define LIB_CONSTRUCTOR_DEPRECATED
#      else
#        define LIB_CONSTRUCTOR_DEPRECATED LIB_DECL_CONSTRUCTOR_DEPRECATED
#      endif
#    else
#      define LIB_CONSTRUCTOR_DEPRECATED LIB_DECL_CONSTRUCTOR_DEPRECATED
#    endif
#  endif
#endif


// disable some annoying and useless warnings in Visual Studio
#ifdef _MSC_VER
    #pragma warning(disable: 4100) // disable "unreferenced formal parameter" warning
    #pragma warning(disable: 4512) // disable "assignment operator could not be generated": shown for any class with a const member variable and no assignment operator
    #pragma warning(disable: 4125) // disable "decimal digit terminates octal escape sequence": about the hard-coded image data of the OpenTL's logo
#endif


/**
 * macros for export handling of symbols
 */
// Generic helper definitions for shared library support
#if defined _WIN32 || defined __CYGWIN__
#define OPENTL_HELPER_DLL_IMPORT __declspec(dllimport)
#define OPENTL_HELPER_DLL_EXPORT __declspec(dllexport)
#define OPENTL_HELPER_DLL_LOCAL
#else
#if __GNUC__ >= 4
#define OPENTL_HELPER_DLL_IMPORT __attribute__ ((visibility("default")))
#define OPENTL_HELPER_DLL_EXPORT __attribute__ ((visibility("default")))
#define OPENTL_HELPER_DLL_LOCAL  __attribute__ ((visibility("hidden")))
#else
#define OPENTL_HELPER_DLL_IMPORT
#define OPENTL_HELPER_DLL_EXPORT
#define OPENTL_HELPER_DLL_LOCAL
#endif
#endif

// Now we use the generic helper definitions above to define OPENTL_API and OPENTL_LOCAL.
// OPENTL_API is used for the public API symbols. It either DLL imports or DLL exports (or does nothing for static build)
// OPENTL_LOCAL is used for non-api symbols.

#ifdef OPENTL_DLL // defined if OPENTL is compiled as a DLL
#ifdef OPENTL_DLL_EXPORTS // defined if we are building the OPENTL DLL (instead of using it)
#define OPENTL_API OPENTL_HELPER_DLL_EXPORT
#else
#define OPENTL_API OPENTL_HELPER_DLL_IMPORT
#endif // OPENTL_DLL_EXPORTS
#define OPENTL_LOCAL OPENTL_HELPER_DLL_LOCAL
#else // OPENTL_DLL is not defined: this means OPENTL is a static lib.
#define OPENTL_API
#define OPENTL_LOCAL
#endif // OPENTL_DLL





#endif /* _LIB_MACROS_H_ */
