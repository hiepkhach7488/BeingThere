#ifndef V3D_COMMON_HPP_
#define V3D_COMMON_HPP_

#include "v3d_common.h"

namespace beingthere{

	//Read Image <K> and Convert It to <T>
	template<typename T, typename K>
	void readImage(string img_path, V3D::Image<T>& img, short nchannels){
		ifstream file;
		file.open(img_path.c_str(), ios::in | ios::binary);

		K* dataBuffer = new K[640*480*nchannels];
		memset(dataBuffer, 0, 640*480*nchannels*sizeof(K));

		if (!file){
			cout << "No background depth file - please capture background" << endl;
		} else {
			file.read((char*) dataBuffer, 640*480*nchannels*sizeof(K));	
		}

		file.close();

		for(int i=0; i<480; ++i)
			for(int j=0; j<640; ++j){
				int idx = i * 640 * nchannels + nchannels*j;
				for(int n=0; n<nchannels; ++n)
					img(j, i, n) = dataBuffer[idx+n];
			}
	}//end read image

	template<typename T, typename K>
	void convertImage(V3D::Image<T>& src, cv::Mat& dst, short nchannels, bool to_ucharDepth){
		for(int i=0; i<480; ++i)
			for(int j=0; j<640; ++j){
				int idx = i * 640 * nchannels + nchannels*j;
				K* dst_ptr = (K*) dst.data;

				for(int n=0; n<nchannels; ++n)
					if(!to_ucharDepth){
						dst_ptr[idx + n] = (K) src(j,i,n);
					}
					else {
						dst_ptr[idx + n] = (K) ((ushort)src(j,i,n) >> 3);
					}
			}
	}//end convert Image

	template<typename T, typename K>
	void convertImage(cv::Mat& src, V3D::Image<K>& dst, short nchannels, bool to_ucharDepth){
		for(int i=0; i<480; ++i)
			for(int j=0; j<640; ++j){
				int idx = i * 640 * nchannels + nchannels*j;
				T* src_ptr = (T*) src.data;

				for(int n=0; n<nchannels; ++n)
					if(!to_ucharDepth){
						dst(j,i,n) = (K)src_ptr[idx + n];
					}
					else {
						dst(j,i,n) = (K)((ushort)src_ptr[idx + n] >> 3);
					}
			}
	} //end convert Image

	template<typename T, typename K>
	void convertImage(V3D::Image<T>& src, V3D::Image<K>& dst, short nchannels, bool to_ucharDepth){
		for(int i=0; i<480; ++i)
			for(int j=0; j<640; ++j)
				for(int n=0; n<nchannels; ++n)
					if(!to_ucharDepth){
						dst(j,i,n) = (K) src(j,i,n);
					}
					else{
						dst(j,i,n) = (K) ((ushort)src(j,i,n) >> 3);
					}

	} // end convert Image

	template<typename T, typename K>
	void convertImage(cv::Mat& src, cv::Mat& dst, short nchannels, bool to_ucharDepth){
		for(int i=0; i<480; ++i)
			for(int j=0; j<640; ++j){
				int idx = i * 640 * nchannels + nchannels*j;

				T* src_ptr = (T*) src.data;
				K* dst_ptr = (K*) dst.data;

				for(int n=0; n<nchannels; ++n)
					if(!to_ucharDepth){
						dst_ptr[idx+n] = (K) src_ptr[idx + n];
					}
					else {
						dst_ptr[idx+n] = (K) ((ushort)src_ptr[idx + n] >> 3);
					}
			}
	}//end convert Image

	template<typename T>
	void minConvolveImageHorizontal(V3D::Image<T> const& src, int const kernelSize, int const center, V3D::Image<T>& dst)
	{
		int const w = src.width();
		int const h = src.height();
		int const nChannels = src.numChannels();

		//dst.resize(w, h, nChannels);

		//for (int ch = 0; ch < nChannels; ++ch)
		{
			int ch = 0;
			for (int y = 0; y < h; ++y)
			{
				for (int x = 0; x < w; ++x)
				{
					T accum;
					{
						int xx = x - center; // dx = 0
						//xx = (xx < 0) ? (-xx - 1) : xx; // mirror at 0, if necessary
						//xx = (xx >= w) ? (2*w - xx - 1) : xx; // mirror at w-1, xx = w - (xx - w + 1) = 2*w - xx - 1
						xx = (std::max)(0, (std::min)(w-1, xx));
						accum = src(xx, y, ch);
					}
					for (int dx = 1; dx < kernelSize; ++dx)
					{
						int xx = x - center + dx;
						//xx = abs(xx); // mirror at 0, if necessary
						//xx = (xx < 0) ? (-xx - 1) : xx; // mirror at 0, if necessary
						//xx = (xx >= w) ? (2*w - xx - 1) : xx; // mirror at w-1, xx = w - (xx - w + 1) = 2*w - xx - 1
						xx = (std::max)(0, (std::min)(w-1, xx));
						accum = min(accum, src(xx, y, ch));
					} // end for (dx)
					dst(x, y, ch) = accum;
				} // end for (dx)
			} // end for (y)
		} // end for (ch)
	} // end convolveImageHorizontal()

	template<typename T>
	void minConvolveImageVertical(V3D::Image<T> const& src, int const kernelSize, int const center, V3D::Image<T>& dst)
	{
		int const w = src.width();
		int const h = src.height();
		int const nChannels = src.numChannels();

		//dst.resize(w, h, nChannels);

		//for (int ch = 0; ch < nChannels; ++ch)
		{
			int ch = 0;
			for (int y = 0; y < h; ++y)
			{
				for (int x = 0; x < w; ++x)
				{
					T accum;
					{
						int yy = y - center; // dy = 0
						//yy = (yy < 0) ? (-yy - 1) : yy; // mirror at 0, if necessary
						//yy = (yy >= h) ? (2*h - yy - 1) : yy; // mirror at h-1
						yy = (std::max)(0, (std::min)(h-1, yy));
						accum = src(x, yy, ch);
					}

					for (int dy = 1; dy < kernelSize; ++dy)
					{
						int yy = y - center + dy;
						//yy = abs(yy); // mirror at 0, if necessary
						//yy = (yy < 0) ? (-yy - 1) : yy; // mirror at 0, if necessary
						//yy = (yy >= h) ? (2*h - yy - 1) : yy; // mirror at h-1
						yy = (std::max)(0, (std::min)(h-1, yy));
						accum = min(accum, src(x, yy, ch));
					} // end for (dx)
					dst(x, y, ch) = accum;
				} // end for (dx)
			} // end for (y)
		} // end for (ch)
	} // end convolveImageVertical()
}

#endif