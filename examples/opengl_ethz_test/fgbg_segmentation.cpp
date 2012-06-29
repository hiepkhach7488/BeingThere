#include "fgbg_segmentation.h"

#include "Base/v3d_imageprocessing.h"
#include "Base/v3d_timer.h"
#include <float.h>

using namespace V3D;

FGBG_Segmentation::FGBG_Segmentation()
: _costBuf("r=32f enableTextureRG", "costBuf"),
_edgeWeightBuf("rg=32f enableTextureRG", "edgeWeightBuf"),
_depthBufH("r=32f enableTextureRG", "depthBufH"),
_depthBufV("r=32f enableTextureRG", "depthBufV"),
_segmentation(false)
{
	//cout << "FGBG_Segmentation::FGBG_Segmentation()" << endl;
	//cout << "tex name = " << _segmentation.getResultBuffer().getTexName() << endl;
	//default parameters
	_alpha = 80.0f;
	_lambda_color = 5.0f;
	_color_bias = 1.0f/20;
	_lambda_depth = 5.0f;
	_depth_bias = 1.0f/20;
	_nIterations = 100;
}

FGBG_Segmentation::~FGBG_Segmentation()
{
	//cout << "FGBG_Segmentation::~FGBG_Segmentation()" << endl;
}

void FGBG_Segmentation::init(int w, int h)
{
	_width = w;
	_height = h;
	//cout << "tex name = " << _segmentation.getResultBuffer().getTexName() << endl;
	_segmentation.allocate(w, h);
	_segmentation.setTimesteps(0.249f, 2.0f);
	_edgeWeightBuf.allocate(w, h);
	_costBuf.allocate(w, h);
	_depthBufH.allocate(w, h);
	_depthBufV.allocate(w, h);
}

void FGBG_Segmentation::clear(){
	_segmentation.deallocate();
	_edgeWeightBuf.deallocate();
	_costBuf.deallocate();
	_depthBufH.deallocate();
	_depthBufV.deallocate();
}

//======================================================================

/*
namespace
{

template <typename T>
struct DepthErodeOp
{
	T operator()(T a, T b) const
	{
		return (std::min)(a, b);
	}
};

template<typename T>
inline void
minConvolveImageHorizontal(V3D::Image<T> const& src, int const kernelSize, int const center, V3D::Image<T>& dst)
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
					xx = (std::min)(0, (std::max)(w-1, xx));
					accum = src(xx, y, ch);
				}
				for (int dx = 1; dx < kernelSize; ++dx)
				{
					int xx = x - center + dx;
					//xx = abs(xx); // mirror at 0, if necessary
					//xx = (xx < 0) ? (-xx - 1) : xx; // mirror at 0, if necessary
					//xx = (xx >= w) ? (2*w - xx - 1) : xx; // mirror at w-1, xx = w - (xx - w + 1) = 2*w - xx - 1
					xx = (std::min)(0, (std::max)(w-1, xx));
					accum = min(accum, src(xx, y, ch));
				} // end for (dx)
				dst(x, y, ch) = accum;
			} // end for (dx)
		} // end for (y)
	} // end for (ch)
} // end convolveImageHorizontal()

template<typename T>
inline void
minConvolveImageVertical(V3D::Image<T> const& src, int const kernelSize, int const center, V3D::Image<T>& dst)
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
*/

/*
inline
void FGBG_Segmentation::erodeDepth(int const radius, V3D::Image<float>& depth)
{
	int const w = depth.width();
	int const h = depth.height();

	//DepthErodeOp<float> op;
	Image<float> tmp(w, h, 1), tmp2(w, h, 1);
	minConvolveImageHorizontal(depth, 2*radius+1, radius, tmp);
	minConvolveImageVertical(tmp, 2*radius+1, radius, tmp2);
#pragma omp parallel for
	for (int y = 0; y < h; ++y)
		for (int x = 0; x < w; ++x)
		{
			float const d = depth(x, y);
			float const d1 = tmp2(x, y);
			depth(x, y) = (d1 > 0.0f) ? ((d < d1 + 0.2f) ? d : -1.0f) : -1.0f;
		}
} // end erodeDepth()
*/

void FGBG_Segmentation::run(unsigned int const bgColorTexId, unsigned int const bgDepthTexId, unsigned int const curColorTexId, unsigned int const curDepthTexId)
{
	//ScopedTimer t("FGBG_Segmentation::run");
	renderEdgeWeights(_alpha, curColorTexId, _edgeWeightBuf);
#if 0
	glBindTexture(GL_TEXTURE_2D, curDepthTexId);
	Image<float> depth(_width, _height, 1);
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RED, GL_FLOAT, depth.begin(0));
	{
		//cout << "_width = " << _width << " _height = " << _height << endl;
		//ScopedTimer t("erode");
		erodeDepth(5, depth);
	}
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, _width, _height, GL_RED, GL_FLOAT, depth.begin(0));
#elif 1
	{
		static Cg_FragmentProgram * shader = 0;
		static Cg_FragmentProgram * shader2 = 0;
		if (!shader)
		{
			shader = new Cg_FragmentProgram("FGBG_Segmentation::depthShader");
			char const * source =
				"#define RADIUS 8\n"
				"void main(uniform sampler2D depth_tex : TEXUNIT0, \n"
				"                  float2 st : TEXCOORD0, \n"
				"                  float2 deltaST : TEXCOORD4, \n"
				"              out float  color_out : COLOR) \n"
				"{ \n"
				"   float val = 1e30f; \n"
				"#pragma unroll \n"
				"   for (int i = -RADIUS; i <= RADIUS; ++i) \n"
				"      val = min(val, tex2D(depth_tex, st + i*deltaST).x); \n"
				"   color_out = val; \n"
				"} \n";
			shader->setProgram(source);
			shader->compile();
			checkGLErrorsHere0();
		}

		if (!shader2)
		{
			shader2 = new Cg_FragmentProgram("FGBG_Segmentation::combineShader");
			char const * source =
				"#define RADIUS 8\n"
				"void main(uniform sampler2D depth_tex : TEXUNIT0, \n"
				"          uniform sampler2D minConv_tex : TEXUNIT1, \n"
				"                  float2 st : TEXCOORD0, \n"
				"                  float  depthThreshold : TEXCOORD4, \n"
				"              out float  color_out : COLOR) \n"
				"{ \n"
				"   float d = tex2D(depth_tex, st); \n"
				"   float d1 = tex2D(minConv_tex, st); \n"
				"   color_out = (d1 > 0) ? ((d < d1 + depthThreshold) ? d : -1) : -1; \n"
				"} \n";
			shader2->setProgram(source);
			shader2->compile();
			checkGLErrorsHere0();
		}

		int const w = _width;
		int const h = _height;
		float const ds = 1.0f / w;
		float const dt = 1.0f / h;

		_depthBufH.activate();
		setupNormalizedProjection();
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, curDepthTexId);
		glEnable(GL_TEXTURE_2D);

		shader->enable();

		glMultiTexCoord2f(GL_TEXTURE4, ds, 0.0f);
		renderNormalizedQuad();

		_depthBufV.activate();
		_depthBufH.enableTexture(GL_TEXTURE0);

		glMultiTexCoord2f(GL_TEXTURE4, 0.0f, dt);
		renderNormalizedQuad();

		shader->disable();
		_depthBufH.disableTexture(GL_TEXTURE0);

		_depthBufH.activate();

		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, curDepthTexId);
		glEnable(GL_TEXTURE_2D);
		_depthBufV.enableTexture(GL_TEXTURE1);

		shader2->enable();
		glMultiTexCoord1f(GL_TEXTURE4, 0.2);
		renderNormalizedQuad();
		shader2->disable();

		_depthBufV.disableTexture(GL_TEXTURE0);
		_depthBufV.disableTexture(GL_TEXTURE1);
	}
#endif
	//renderSegmentationCost(_lambda_col, _color_bias, _lambda_depth, _depth_bias, bgColorTexId, bgDepthTexId, curColorTexId, curDepthTexId, _costBuf);
	renderSegmentationCost(_lambda_color, _color_bias, _lambda_depth, _depth_bias, bgColorTexId, bgDepthTexId, curColorTexId, _depthBufH.textureID(), _costBuf);
	_segmentation.initializeIterations();
	//_segmentation.setInitialValue(u.begin()); //optional
	_segmentation.iterate(_costBuf.getTexture().textureID(), _edgeWeightBuf.textureID(), _nIterations);
	//glFinish();
}

void FGBG_Segmentation::getResult(GLuint texId){
	_segmentation.getResultBuffer().activate();
	float* buffer = new float[_width*_height];
	glReadPixels(0, 0, _width, _height, GL_RED, GL_FLOAT, buffer);	
	glBindTexture(GL_TEXTURE_2D, texId);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, _width, _height, 0, GL_RED, GL_FLOAT, buffer);
	delete[] buffer;
}

GLuint FGBG_Segmentation::getResult()
{
	return _segmentation.getResultBuffer().textureID();
}

void 
FGBG_Segmentation ::renderEdgeWeights(float const alpha, unsigned int texId, RTT_Buffer& dst)
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
} // end renderEdgeWeights()

void
FGBG_Segmentation::renderSegmentationCost(float const lambdaColor, float const colorBias, float const lambdaDepth, float const depthBias,
										  unsigned int const bgColorTexId, unsigned int const bgDepthTexId,
										  unsigned int const curColorTexId, unsigned int const curDepthTexId,
										  RTT_Buffer& dst)
{
	static Cg_FragmentProgram * shader = 0;
	if (shader == 0)
	{
		shader = new Cg_FragmentProgram("renderEdgeWeights::shader");

		char const * source =
			"void main(uniform sampler2D bgCol_tex    : TEXUNIT0, \n"
			"          uniform sampler2D bgDepth_tex  : TEXUNIT1, \n"
			"          uniform sampler2D curCol_tex   : TEXUNIT2, \n"
			"          uniform sampler2D curDepth_tex : TEXUNIT3, \n"
			"                  float2 st0 : TEXCOORD0, \n"
			"                  float4 params : TEXCOORD4, \n"
			"          uniform float3x3 A_color,"
			"              out float  cost_out : COLOR) \n"
			"{ \n"
			"   float3 bgCol    = tex2D(bgCol_tex, st0); \n"
			"   float  bgDepth  = tex2D(bgDepth_tex, st0).x; \n"
			"   float3 curCol   = tex2D(curCol_tex, st0); \n"
			"   float  curDepth = tex2D(curDepth_tex, st0).x; \n"
			"   curCol = mul(A_color, curCol); \n"
			"   bgCol  = mul(A_color, bgCol); \n"
			"   float3 colDiff  = abs(bgCol - curCol); \n"
			"   colDiff.x /= 4; \n"
			"   cost_out = params.x * (dot(float3(1), colDiff) - params.y); \n"
			"   float depthDiff = max(0.0f, abs(bgDepth - curDepth) - 0.5f) - params.w; \n"
			"   depthDiff = (bgDepth > 0) ? depthDiff : 0.0f; \n"
			"   depthDiff = (curDepth > 0) ? depthDiff : 0.0f; \n"
			"   cost_out += params.z * depthDiff; \n"
			"} \n";
		shader->setProgram(source);
		shader->compile();
		checkGLErrorsHere0();

		double const colorWeights[9] = { // RGB->YUV conversion matrix
			0.29900,   0.58700,   0.14400,
			-0.14713,  -0.28886,   0.43600,
			0.61500,  -0.51500,  -0.10000 };

			Matrix3x3d A_color;
			for (int i = 0; i < 3; ++i)
				for (int j = 0; j < 3; ++j) A_color[i][j] = colorWeights[3*i+j];
			shader->matrixParameterR("A_color", 3, 3, &A_color[0][0]);
	}

	dst.activate();
	setupNormalizedProjection();
	glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, bgColorTexId); glEnable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, bgDepthTexId); glEnable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE2); glBindTexture(GL_TEXTURE_2D, curColorTexId); glEnable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE3); glBindTexture(GL_TEXTURE_2D, curDepthTexId); glEnable(GL_TEXTURE_2D);

	shader->enable();
	glMultiTexCoord4f(GL_TEXTURE4, lambdaColor, colorBias, lambdaDepth, depthBias);
	renderNormalizedQuad();
	shader->disable();

	glActiveTexture(GL_TEXTURE0); glDisable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE1); glDisable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE2); glDisable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE3); glDisable(GL_TEXTURE_2D);
} // end renderSegmentationCost()


/*int main(int argc, char * argv[])
{
	unsigned int win;
	glutInitWindowPosition(0, 0);
	glutInitWindowSize(640, 480);
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);

	if (!(win = glutCreateWindow("GPU KLT Test")))
	{
		cerr << "Error, couldn't open window" << endl;
		return -1;
	}

	glewInit();
	Cg_ProgramBase::initializeCg();

	int const w = 1024;
	int const h = 768;

	Image<float> bgDepth(w, h, 1), curDepth(w, h, 1);
	Image<unsigned char> bgColor(w, h, 3), curColor(w, h, 3);;

	readFloatImage("background/c0-background-depth.dat", bgDepth);
	readColorImage("background/c0-background-color.dat", bgColor);

	readFloatImage("cur/c0-visibility-frame0000.dat", curDepth);
	readColorImage("cur/c0-frame0000.dat", curColor);

	flipImageUpsideDown(bgDepth);
	flipImageUpsideDown(curDepth);

	saveImageFile(bgColor, "color_bg.png");
	saveImageFile(curColor, "color_cur.png");

	saveImageChannel(bgDepth, 0, 0.0f, 4.0f, "depth_bg.png");
	saveImageChannel(curDepth, 0, 0.0f, 4.0f, "depth_cur.png");

	float const alpha = 80.0f;

	ImageTexture2D bgColorTex;
	bgColorTex.allocateID();
	bgColorTex.reserve(w, h, "rgb=8");
	bgColorTex.overwriteWith(bgColor.begin(0), bgColor.begin(1), bgColor.begin(2));

	ImageTexture2D bgDepthTex;
	bgDepthTex.allocateID();
	bgDepthTex.reserve(w, h, "r=32f enableTextureRG");
	bgDepthTex.overwriteWith(bgDepth.begin(0), 1);

	ImageTexture2D curColorTex;
	curColorTex.allocateID();
	curColorTex.reserve(w, h, "rgb=8");
	curColorTex.overwriteWith(curColor.begin(0), curColor.begin(1), curColor.begin(2));

	ImageTexture2D curDepthTex;
	curDepthTex.allocateID();
	curDepthTex.reserve(w, h, "r=32f enableTextureRG");
	curDepthTex.overwriteWith(curDepth.begin(0), 1);

	RTT_Buffer edgeWeightBuf("rg=32f enableTextureRG", "edgeWeightBuf");
	edgeWeightBuf.allocate(w, h);

	RTT_Buffer costBuf("r=32f enableTextureRG", "costBuf");
	costBuf.allocate(w, h);

	renderEdgeWeights(alpha, curColorTex.textureID(), edgeWeightBuf);

	Image<float> g(w, h, 3, 1.0f);
	edgeWeightBuf.getTexture().readback(g.begin(0), g.begin(1), g.begin(2));

	saveImageChannel(g, 0, "g_x.png");
	saveImageChannel(g, 1, "g_y.png");

	float const lambda_col = 10.0f;
	float const color_bias = 0.1;
	float const lambda_depth = 1.0f;
	float const depth_threshold = 0.5f;

	renderSegmentationCost(lambda_col, color_bias, lambda_depth, depth_threshold,
		bgColorTex.textureID(), bgDepthTex.textureID(), curColorTex.textureID(), curDepthTex.textureID(), costBuf);

	Image<float> costs(w, h, 1, 0.0f);
	costBuf.getTexture().readback(costs.begin(0), 1);
	cout << "min(costs) = " << costs.minElement() << " max(costs) = " << costs.maxElement() << endl;
	saveImageChannel(costs, 0, "costs.png");


#if 1
	BinarySegmentationUzawa seg(false);
	//seg.setTimesteps(0.7f, 0.7f);
	//seg.setTimesteps(0.49f, 1.0f);
	seg.setTimesteps(0.249f, 2.0f);
#elif 0
	BinarySegmentationPD seg(false);
	//seg.setTimesteps(0.124f, 1.0f);
	seg.setTimesteps(0.124f*2, 1.0f/2);
#elif 0
	BinarySegmentationFwBw seg(true);
	seg.epsilon = 0.0125f; seg.tau = 0.24f;
#else
	BinarySegmentationRelaxed seg(false);
	seg.theta = 0.125; seg.tau = 0.24f;
#endif
	seg.allocate(w, h);

	Image<float> u(w, h, 1);
	for (int y = 0; y < h; ++y)
		for (int x = 0; x < w; ++x)
			u(x, y) = (costs(x, y) > 0) ? 0.0f : 1.0f;
	saveImageChannel(u, 0, 0.0f, 1.0f, "u0.png");

	int const nIterations = 50;

	{
		ScopedTimer t("bin seg");
		for (int k = 0; k < 1; ++k)
		{
			seg.initializeIterations();
			seg.setInitialValue(u.begin()); //optional
			seg.iterate(costTex.textureID(), edgeWeightTex.textureID(), nIterations);
			glFinish();
		}
	}

	seg.getResultBuffer().activate();
	glReadPixels(0, 0, w, h, GL_RED, GL_FLOAT, u.begin(0));
	FrameBufferObject::disableFBORendering();
	saveImageChannel(u, 0, 0.0f, 1.0f, "u_seg_pd.png");

	return 0;
}*/
