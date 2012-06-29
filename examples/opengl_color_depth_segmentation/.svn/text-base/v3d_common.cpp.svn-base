#include "v3d_common.h"
using namespace V3D_GPU;

namespace beingthere{
	void erodeDepth(int const radius, V3D::Image<float>& depth)
	{
		int const w = depth.width();
		int const h = depth.height();

		//DepthErodeOp<float> op;
		V3D::Image<float> tmp(w, h, 1), tmp2(w, h, 1);

		minConvolveImageHorizontal<float>(depth, 2*radius+1, radius, tmp);
		minConvolveImageVertical<float>(tmp, 2*radius+1, radius, tmp2);

		//#pragma omp parallel for

		for (int y = 0; y < h; ++y)
			for (int x = 0; x < w; ++x)
			{
				float const d = depth(x, y);
				float const d1 = tmp2(x, y);

				depth(x, y) = (d1 > 0.0f) ? ((d < d1 + 200.f) ? d : -1.0f) : -1.0f;			
			}
	} // end erodeDepth()


	void renderEdgeWeights(float const alpha, unsigned int texId, RTT_Buffer& dst)
	{
		static Cg_FragmentProgram * shader = 0;
		if (shader == 0)
		{
			shader = new Cg_FragmentProgram("renderEdgeWeights::shader");

			char const * source =
				"void main(uniform sampler2D texture : TEXUNIT0, \n"
				"                  float2 st : TEXCOORD0, \n"
				"                  float4 stWE : TEXCOORD1, \n"
				"                  float4 stNS : TEXCOORD2, \n"
				"                  float  alpha : TEXCOORD4, \n"
				"              out float2 color_out : COLOR) \n"
				"{ \n"
				"   float2 stW = stWE.xy; \n"
				"   float2 stN = stNS.xy; \n"
				"   float2 stE = stWE.zw; \n"
				"   float2 stS = stNS.zw; \n"
				"   float3 col = tex2D(texture, st); \n"
				"   float3 colx = tex2D(texture, stE); \n"
				"   float3 coly = tex2D(texture, stS); \n"
				"   float3 dx = colx - col; \n"
				"   float3 dy = coly - col; \n"
				"   float  dx2 = dot(dx, dx); \n"
				"   float  dy2 = dot(dy, dy); \n"
				"   color_out.x = 1.0 / (1.0f + alpha*dx2); \n"
				"   color_out.y = 1.0 / (1.0f + alpha*dy2); \n"
				"} \n";
			shader->setProgram(source);
			shader->compile();
			checkGLErrorsHere0();
		}

		int const w = dst.width();
		int const h = dst.height();
		float const ds = 1.0f / w;
		float const dt = 1.0f / h;

		dst.activate();
		setupNormalizedProjection();

		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, texId);
	
		glEnable(GL_TEXTURE_2D);

		shader->enable();		
			glMultiTexCoord1f(GL_TEXTURE4, alpha);
			renderNormalizedQuad(GPU_SAMPLE_NEIGHBORS, ds, dt);
		shader->disable();

		glDisable(GL_TEXTURE_2D);

		dst.getFBO().disableFBORendering();
	} // end renderEdgeWeights()


	void copyTexture(unsigned int src, unsigned int dst){
		static GLuint _copyFbo;
		glGenFramebuffers(1, &_copyFbo);

		glBindFramebuffer(GL_FRAMEBUFFER, _copyFbo);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, dst, 0);
		
			glViewport(0, 0, 640, 480);
			glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

			//setup normalized projection
			V3D_GPU::setupNormalizedProjection();
		
			glEnable(GL_TEXTURE_2D);
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, src);

			V3D_GPU::renderNormalizedQuad();
		
			glBindTexture(GL_TEXTURE_2D, 0);
			glDisable(GL_TEXTURE_2D);

		glBindFramebuffer(GL_FRAMEBUFFER, 0);

	}//end copy texture

}