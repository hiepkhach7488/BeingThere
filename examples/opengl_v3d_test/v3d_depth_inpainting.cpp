#include "v3d_depth_inpainting.h"
#include "Base/v3d_image.h"

using namespace std;
using namespace V3D;
using namespace V3D_GPU;

GPU_DepthInpainting::GPU_DepthInpainting()
: _theta(0.05), _tau(0.24f), _nIterations(300)
{
	_uBufs[0] = new RTT_Buffer("r=32f tex2D enableTextureRG", "GPU_DepthInpainting::uBufs[0]");
	_uBufs[1] = new RTT_Buffer("r=32f tex2D enableTextureRG", "GPU_DepthInpainting::uBufs[1]");

	_pBufs[0] = new RTT_Buffer("rg=16f tex2D enableTextureRG", "GPU_DepthInpainting::pBufs[0]");
	_pBufs[1] = new RTT_Buffer("rg=16f tex2D enableTextureRG", "GPU_DepthInpainting::pBufs[1]");
}

GPU_DepthInpainting::~GPU_DepthInpainting(){

}

void
GPU_DepthInpainting::allocate(int const w, int const h)
{
	_width = w;
	_height = h;

	_uBufs[0]->allocate(w, h);
	_uBufs[1]->allocate(w, h);

	_pBufs[0]->allocate(w, h);
	_pBufs[1]->allocate(w, h);

	_uOrigTex.allocateID();
	_uOrigTex.reserve(w, h, "r=32f enableTextureRG");

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glActiveTexture(GL_TEXTURE0);
} // end GPU_DepthInpainting::allocate()

void
GPU_DepthInpainting::setDepth(float const * u0)
{
	_uOrigTex.overwriteWith(u0, 1);
	//_uBufs[0]->getTexture().overwriteWith(u0, 1);
}

void
GPU_DepthInpainting::initialize(float const initDepth)
{
	_uBufs[0]->activate();
	glClearColor(initDepth, initDepth, initDepth, initDepth);
	glClear(GL_COLOR_BUFFER_BIT);

	glClearColor(0, 0, 0, 0);
	_pBufs[0]->activate();
	glClear(GL_COLOR_BUFFER_BIT);

	_uBufs[0]->getFBO().disableFBORendering();
	_pBufs[0]->getFBO().disableFBORendering();
}

void
GPU_DepthInpainting::iterate(unsigned int origTexId, unsigned int edgeTexId)
{
	static Cg_FragmentProgram * shader_u = 0;
	static Cg_FragmentProgram * shader_p = 0;

	string const SHADER_ROOT_PATH = BEINGTHERE_EXAMPLE_ROOT_DIR + "opengl_v3d_test/shader/";

	if (shader_u == 0)
	{
		shader_u = new Cg_FragmentProgram("GPU_DepthInpainting::depth_inpainting_relaxed_update_u.cg");
		shader_u->setProgramFromFile((SHADER_ROOT_PATH + "depth_inpainting_relaxed_update_u.cg").c_str());
		shader_u->compile();
		checkGLErrorsHere0();
	}
	if (shader_p == 0)
	{
		shader_p = new Cg_FragmentProgram("GPU_DepthInpainting::depth_inpainting_relaxed_update_p.cg");
		shader_p->setProgramFromFile((SHADER_ROOT_PATH + "depth_inpainting_relaxed_update_p.cg").c_str());
		shader_p->compile();
		checkGLErrorsHere0();
	}

	int const w = _width;
	int const h = _height;

	RTT_Buffer * &ubuffer0 = _uBufs[0];
	RTT_Buffer * &ubuffer1 = _uBufs[1];

	RTT_Buffer * &pbuffer0 = _pBufs[0];
	RTT_Buffer * &pbuffer1 = _pBufs[1];

	float const ds = 1.0f / w;
	float const dt = 1.0f / h;

	setupNormalizedProjection();

	shader_u->parameter("theta", _theta);
	shader_p->parameter("timestep_over_theta", _tau/_theta);

	checkGLErrorsHere0();

	//_uOrigTex.enable(GL_TEXTURE2_ARB);
	if (origTexId == 0) origTexId = _uOrigTex.textureID();
	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, origTexId);
	glEnable(GL_TEXTURE_2D);

	glActiveTexture(GL_TEXTURE3);
	glBindTexture(GL_TEXTURE_2D, edgeTexId);
	glEnable(GL_TEXTURE_2D);

	for (int iter = 0; iter < _nIterations; ++iter)
	{
		pbuffer1->activate();

		ubuffer0->enableTexture(GL_TEXTURE0_ARB);
		pbuffer0->enableTexture(GL_TEXTURE1_ARB);

		shader_p->enable();
		renderNormalizedQuad(GPU_SAMPLE_REVERSE_NEIGHBORS, ds, dt);
		shader_p->disable();

		//ubuffer-->disableTexture(GL_TEXTURE0_ARB);
		pbuffer0->disableTexture(GL_TEXTURE1_ARB);

		std::swap(pbuffer0, pbuffer1);

		ubuffer1->activate();

		//ubuffer0->enableTexture(GL_TEXTURE0_ARB);
		pbuffer0->enableTexture(GL_TEXTURE1_ARB);

		shader_u->enable();
		renderNormalizedQuad(GPU_SAMPLE_REVERSE_NEIGHBORS, ds, dt);
		shader_u->disable();

		ubuffer0->disableTexture(GL_TEXTURE0_ARB);
		pbuffer0->disableTexture(GL_TEXTURE1_ARB);

		std::swap(ubuffer0, ubuffer1);
	} // end for (iter)

	_uOrigTex.disable(GL_TEXTURE2_ARB);
	_uOrigTex.disable(GL_TEXTURE3_ARB);

	_uBufs[0]->getFBO().disableFBORendering();
	_pBufs[0]->getFBO().disableFBORendering();
	_uBufs[1]->getFBO().disableFBORendering();
	_pBufs[1]->getFBO().disableFBORendering();
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glActiveTexture(GL_TEXTURE0);
	
} // end GPU_DepthInpainting::run()
