#ifndef BEINGTHERE_COMMON_H
#define BEINGTHERE_COMMON_H

#include "pcl/gpu/containers/device_array.h"
#include "pcl/gpu/utils/safe_call.hpp"
using namespace pcl::gpu;

namespace beingthere{
	namespace gpu{
		typedef unsigned short ushort;		
		typedef DeviceArray2D<float> MapArr;
		typedef DeviceArray2D<ushort> DepthMap;
		typedef float4 PointType;

		//Intrinsic Struct
		struct Intr
		{
			float fx, fy, cx, cy;
			Intr () {}
			Intr (float fx_, float fy_, float cx_, float cy_) : fx(fx_), fy(fy_), cx(cx_), cy(cy_) {}

			Intr operator()(int level_index) const
			{ 
				int div = 1 << level_index; 
				return (Intr (fx / div, fy / div, cx / div, cy / div));
			}
			//friend ostream& operator<<(ostream& os, const Intr& dt);
		};



		//typedef struct Dist{
		//	float d1, d2, d3, d4, d5;
		//	Dist(){}
		//	Dist(float d1_, float d2_, float d3_, float d4_) : d1(d1_), d2(d2_), d3(d3_), d4(d4_){}
		//	friend ostream& operator<<(ostream& os, const Dist& dt);
		//};
		//ostream& operator<<(ostream& os, const Dist& distortion)
		//{
		//	os << "d1:= " << distortion.d1 << "d2:= " << distortion.d2 << " " <<  
		//		"d3:= " << distortion.d3 << "d4:= " << distortion.d4 << "d5:= " << distortion.d5;
		//	return os;
		//}

		//Device Matrix
		struct Mat33
		{
			float3 data[3];
		};

		struct float8  { float x, y, z, w, c1, c2, c3, c4; };
		struct float12 { float x, y, z, w, normal_x, normal_y, normal_z, n4, c1, c2, c3, c4;};

		//Create Vertex Map
		void createVMap (const Intr& intr, const DepthMap& depth, MapArr& vmap);

		//Create Normal Map
		void createNMap (const MapArr& vmap, MapArr& nmap);

		//Transformation
		void tranformMaps (const MapArr& vmap_src, const MapArr& nmap_src, const Mat33& Rmat, const float3& tvec, MapArr& vmap_dst, MapArr& nmap_dst);

		//Truncate Depth
		void truncateDepth(DepthMap& depth, float max_distance);

		template<typename T> 
		void convert (const MapArr& vmap, DeviceArray2D<T>& output);

	}//end namespace gpu
}//end namespace beingthere

#endif