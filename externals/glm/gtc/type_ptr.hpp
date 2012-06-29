///////////////////////////////////////////////////////////////////////////////////
/// OpenGL Mathematics (glm.g-truc.net)
///
/// Copyright (c) 2005 - 2011 G-Truc Creation (www.g-truc.net)
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
/// 
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
/// 
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
/// THE SOFTWARE.
///
/// @ref gtc_type_ptr
/// @file glm/gtc/type_ptr.hpp
/// @date 2009-05-06 / 2011-06-05
/// @author Christophe Riccio
///
/// @see core (dependence)
/// @see gtc_half_float (dependence)
/// @see gtc_quaternion (dependence)
///
/// @defgroup gtc_type_ptr GLM_GTC_type_ptr: Memory layout access
/// @ingroup gtc
///
/// @brief Used to get a pointer to the memory layout of a basic type.
/// 
/// This extension defines an overloaded function, glm::value_ptr, which
/// takes any of the \ref core_template "core template types". It returns
/// a pointer to the memory layout of the object. Matrix types store their values
/// in column-major order.
/// 
/// This is useful for uploading data to matrices or copying data to buffer objects.
///
/// Example:
/// @code
/// #include <glm/glm.hpp>
/// #include <glm/gtc/type_ptr.hpp>
/// 
/// glm::vec3 aVector(3);
/// glm::mat4 someMatrix(1.0);
/// 
/// glUniform3fv(uniformLoc, 1, glm::value_ptr(aVector));
/// glUniformMatrix4fv(uniformMatrixLoc, 1, GL_FALSE, glm::value_ptr(someMatrix));
/// @endcode
/// 
/// <glm/gtc/type_ptr.hpp> need to be included to use these functionalities.
///////////////////////////////////////////////////////////////////////////////////

#ifndef GLM_GTC_type_ptr
#define GLM_GTC_type_ptr GLM_VERSION

// Dependency:
#include "../glm.hpp"
#include "../gtc/half_float.hpp"
#include "../gtc/quaternion.hpp"
#include <cstring>

#if(defined(GLM_MESSAGES) && !defined(glm_ext))
#	pragma message("GLM: GLM_GTC_type_ptr extension included")
#endif

namespace glm
{ 
	/// @addtogroup gtc_type_ptr
	/// @{

	/// Return the constant address to the data of the input parameter.
	/// From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER T const * value_ptr
	(
		detail::tvec2<T> const & vec
	)
	{
		return &(vec.x);
	}

	//! Return the constant address to the data of the input parameter.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER T * value_ptr
	(
		detail::tvec2<T> & vec
	)
	{
		return &(vec.x);
	}

	//! Return the constant address to the data of the input parameter.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER T const * value_ptr
	(
		detail::tvec3<T> const & vec
	)
	{
		return &(vec.x);
	}

	//! Return the constant address to the data of the input parameter.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER T * value_ptr
	(
		detail::tvec3<T> & vec
	)
	{
		return &(vec.x);
	}
		
	//! Return the constant address to the data of the input parameter.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER T const * value_ptr
	(	
		detail::tvec4<T> const & vec
	)
	{
		return &(vec.x);
	}

	//! Return the constant address to the data of the input parameter.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER T * value_ptr
	(	
		detail::tvec4<T> & vec
	)
	{
		return &(vec.x);
	}

	//! Return the constant address to the data of the input parameter.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER T const * value_ptr
	(
		detail::tmat2x2<T> const & mat
	)
	{
		return &(mat[0].x);
	}

	//! Return the constant address to the data of the input parameter.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER T * value_ptr
	(
		detail::tmat2x2<T> & mat
	)
	{
		return &(mat[0].x);
	}
		
	//! Return the constant address to the data of the input parameter.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER T const * value_ptr
	(
		detail::tmat3x3<T> const & mat
	)
	{
		return &(mat[0].x);
	}

	//! Return the constant address to the data of the input parameter.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER T * value_ptr
	(
		detail::tmat3x3<T> & mat
	)
	{
		return &(mat[0].x);
	}
		
	//! Return the constant address to the data of the input parameter.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER T const * value_ptr
	(
		detail::tmat4x4<T> const & mat
	)
	{
		return &(mat[0].x);
	}

	//! Return the constant address to the data of the input parameter.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER T * value_ptr
	(
		detail::tmat4x4<T> & mat
	)
	{
		return &(mat[0].x);
	}

	//! Return the constant address to the data of the input parameter.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER T const * value_ptr
	(
		detail::tmat2x3<T> const & mat
	)
	{
		return &(mat[0].x);
	}

	//! Return the constant address to the data of the input parameter.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER T * value_ptr
	(
		detail::tmat2x3<T> & mat
	)
	{
		return &(mat[0].x);
	}
		
	//! Return the constant address to the data of the input parameter.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER T const * value_ptr
	(
		detail::tmat3x2<T> const & mat
	)
	{
		return &(mat[0].x);
	}

	//! Return the constant address to the data of the input parameter.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER T * value_ptr
	(
		detail::tmat3x2<T> & mat
	)
	{
		return &(mat[0].x);
	}
		
	//! Return the constant address to the data of the input parameter.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER T const * value_ptr
	(
		detail::tmat2x4<T> const & mat
	)
	{
		return &(mat[0].x);
	}

	//! Return the constant address to the data of the input parameter.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER T * value_ptr
	(
		detail::tmat2x4<T> & mat
	)
	{
		return &(mat[0].x);
	}
		
	//! Return the constant address to the data of the input parameter.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER T const * value_ptr
	(
		detail::tmat4x2<T> const & mat
	)
	{
		return &(mat[0].x);
	}

	//! Return the constant address to the data of the input parameter.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER T * value_ptr
	(	
		detail::tmat4x2<T> & mat
	)
	{
		return &(mat[0].x);
	}
		
	//! Return the constant address to the data of the input parameter.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER T const * value_ptr
	(
		detail::tmat3x4<T> const & mat
	)
	{
		return &(mat[0].x);
	}

	//! Return the constant address to the data of the input parameter.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER T * value_ptr
	(
		detail::tmat3x4<T> & mat
	)
	{
		return &(mat[0].x);
	}
		
	//! Return the constant address to the data of the input parameter.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER T const * value_ptr
	(
		detail::tmat4x3<T> const & mat
	)
	{
		return &(mat[0].x);
	}
    
	//! Return the constant address to the data of the input parameter.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER T const * value_ptr
	(
        detail::tquat<T> const & q
    )
	{
		return &(q[0]);
	}
    
	//! Get the address of the matrix content.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER T * value_ptr(detail::tmat4x3<T> & mat)
	{
		return &(mat[0].x);
	}

	//! Build a vector from a pointer.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER detail::tvec2<T> make_vec2(T const * const ptr)
	{
		detail::tvec2<T> Result;
		memcpy(value_ptr(Result), ptr, sizeof(detail::tvec2<T>));
		return Result;
	}

	//! Build a vector from a pointer.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER detail::tvec3<T> make_vec3(T const * const ptr)
	{
		detail::tvec3<T> Result;
		memcpy(value_ptr(Result), ptr, sizeof(detail::tvec3<T>));
		return Result;
	}

	//! Build a vector from a pointer.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER detail::tvec4<T> make_vec4(T const * const ptr)
	{
		detail::tvec4<T> Result;
		memcpy(value_ptr(Result), ptr, sizeof(detail::tvec4<T>));
		return Result;
	}

	//! Build a matrix from a pointer.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER detail::tmat2x2<T> make_mat2x2(T const * const ptr)
	{
		detail::tmat2x2<T> Result;
		memcpy(value_ptr(Result), ptr, sizeof(detail::tmat2x2<T>));
		return Result;
	}
        
	//! Build a matrix from a pointer.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER detail::tmat2x3<T> make_mat2x3(T const * const ptr)
	{
		detail::tmat2x3<T> Result;
		memcpy(value_ptr(Result), ptr, sizeof(detail::tmat2x3<T>));
		return Result;
	}
        
	//! Build a matrix from a pointer.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER detail::tmat2x4<T> make_mat2x4(T const * const ptr)
	{
		detail::tmat2x4<T> Result;
		memcpy(value_ptr(Result), ptr, sizeof(detail::tmat2x4<T>));
		return Result;
	}
        
	//! Build a matrix from a pointer.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER detail::tmat3x2<T> make_mat3x2(T const * const ptr)
	{
		detail::tmat3x2<T> Result;
		memcpy(value_ptr(Result), ptr, sizeof(detail::tmat3x2<T>));
		return Result;
	}
        
	//! Build a matrix from a pointer.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER detail::tmat3x3<T> make_mat3x3(T const * const ptr)
	{
		detail::tmat3x3<T> Result;
		memcpy(value_ptr(Result), ptr, sizeof(detail::tmat3x3<T>));
		return Result;
	}

	//! Build a matrix from a pointer.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER detail::tmat3x4<T> make_mat3x4(T const * const ptr)
	{
		detail::tmat3x4<T> Result;
		memcpy(value_ptr(Result), ptr, sizeof(detail::tmat3x4<T>));
		return Result;
	}
        
	//! Build a matrix from a pointer.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER detail::tmat4x2<T> make_mat4x2(T const * const ptr)
	{
		detail::tmat4x2<T> Result;
		memcpy(value_ptr(Result), ptr, sizeof(detail::tmat4x2<T>));
		return Result;
	}
        
	//! Build a matrix from a pointer.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER detail::tmat4x3<T> make_mat4x3(T const * const ptr)
	{
		detail::tmat4x3<T> Result;
		memcpy(value_ptr(Result), ptr, sizeof(detail::tmat4x3<T>));
		return Result;
	}
        
	//! Build a matrix from a pointer.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER detail::tmat4x4<T> make_mat4x4(T const * const ptr)
	{
		detail::tmat4x4<T> Result;
		memcpy(value_ptr(Result), ptr, sizeof(detail::tmat4x4<T>));
		return Result;
	}
        
	//! Build a matrix from a pointer.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER detail::tmat2x2<T> make_mat2(T const * const ptr)
	{
		return make_mat2x2(ptr);
	}
        
	//! Build a matrix from a pointer.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER detail::tmat3x3<T> make_mat3(T const * const ptr)
	{
		return make_mat3x3(ptr);
	}
		
	//! Build a matrix from a pointer.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER detail::tmat4x4<T> make_mat4(T const * const ptr)
	{
		return make_mat4x4(ptr);
	}
 
	//! Build a quaternion from a pointer.
	//! From GLM_GTC_type_ptr extension.
	template<typename T>
	GLM_FUNC_QUALIFIER detail::tquat<T> make_quat(T const * const ptr)
	{
		detail::tquat<T> Result;
		memcpy(value_ptr(Result), ptr, sizeof(detail::tquat<T>));
		return Result;
	}
    
	/// @}
}//namespace glm

#include "type_ptr.inl"

#endif//GLM_GTC_type_ptr

