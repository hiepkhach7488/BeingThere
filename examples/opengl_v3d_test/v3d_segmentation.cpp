#include "v3d_segmentation.h"

using namespace V3D_GPU;

V3dSegmentation::V3dSegmentation():
	_costBuf("r=32f enableTextureRG", "costBuf"),
	_edgeWeightBuf("rg=32f enableTextureRG", "edgeWeightBuf"),
	_depthBufH("r=32f enableTextureRG", "depthBufH"),
	_depthBufV("r=32f enableTextureRG", "depthBufV"),
	_segmentation(false){

		int w = 640; int h = 480;
		_depthBufH.allocate(w, h);
		_depthBufV.allocate(w, h);			
		_costBuf.allocate(w, h);
		_edgeWeightBuf.allocate(w, h);

		_segmentation.allocate(w, h);
		_segmentation.setTimesteps(0.249f, 2.0f);

		glGenTextures(1, &erodedTex);
		glBindTexture(GL_TEXTURE_2D, erodedTex); 
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, 640, 480 , 0, GL_RED, GL_FLOAT, 0);

		//Temp Texture
		bgColorTexNoBorder.allocateID();
		bgColorTexNoBorder.reserve(640, 480, "rgb=8");

		curColorTexNoBorder.allocateID();
		curColorTexNoBorder.reserve(640, 480, "rgb=8");

		v3dShader = new V3dShader();
		v3dShader->initTextureFBO();
		v3dShader->createShader();

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glActiveTexture(GL_TEXTURE0);
}


V3dSegmentation::~V3dSegmentation(){

}

void V3dSegmentation::performSegmentation(GLuint bgColor, GLuint bgDepth, GLuint curColor, GLuint curDepth, GLuint segTex){
	V3D_GPU::Cg_ProgramBase::initializeCg();

	//Mask Color To Create New Color Texs
	v3dShader->maskColorFromDepth(bgColor, bgColorTexNoBorder.textureID(), bgDepth);
	v3dShader->maskColorFromDepth(curColor, curColorTexNoBorder.textureID(), curDepth);

	beingthere::renderEdgeWeights(80.0f, curColorTexNoBorder.textureID(), _edgeWeightBuf);

	//Erode Texture
	erodeTextureCG(curDepth, erodedTex);

	//Segmentation	
	float const lambda_col = 5.0f;
	float const color_bias = 1.0f/20;
	float const lambda_depth = 5.0f;
	float const depth_threshold = 1.0f/20;
	int _nIterations = 100;

	//renderSegmentationCost(lambda_col, color_bias, lambda_depth, depth_threshold, bgColorTex.textureID(), bgDepthTex.textureID(), curColorTex.textureID(), erodedTex , _costBuf);
	renderSegmentationCost(lambda_col, color_bias, lambda_depth, depth_threshold, bgColorTexNoBorder.textureID(), bgDepth, curColorTexNoBorder.textureID(), curDepth, _costBuf);

	_segmentation.initializeIterations();
	//_segmentation.setInitialValue(u.begin()); //optional

	_segmentation.iterate(_costBuf.getTexture().textureID(), _edgeWeightBuf.getTexture().textureID(), _nIterations);

	glActiveTexture(GL_TEXTURE0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	beingthere::copyTexture(_segmentation.getResultBuffer().textureID(), segTex);
	//beingthere::copyTexture(curColorTexNoBorder.textureID(), segTex);
	//beingthere::copyTexture(curDepth, segTex);
}


void V3dSegmentation::renderSegmentationCost(float const lambdaColor, float const colorBias, float const lambdaDepth, float const depthBias,
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

	dst.getFBO().disableFBORendering();
} // end renderSegmentationCost()


void V3dSegmentation::erodeTextureCG(GLuint inTex, GLuint outTex){
	static V3D_GPU::Cg_FragmentProgram * shader = 0;
	static V3D_GPU::Cg_FragmentProgram * shader2 = 0;

	if (!shader)
	{
		shader = new V3D_GPU::Cg_FragmentProgram("FGBG_Segmentation::depthShader");
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
		shader2 = new V3D_GPU::Cg_FragmentProgram("FGBG_Segmentation::combineShader");
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

	int w = 640; int h = 480;
	float const ds = 1.0f / w;
	float const dt = 1.0f / h;

	_depthBufH.activate();
	V3D_GPU::setupNormalizedProjection();

	//render pass 1
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, inTex);
	glEnable(GL_TEXTURE_2D);
	shader->enable();
	glMultiTexCoord2f(GL_TEXTURE4, ds, 0.0f);

	V3D_GPU::renderNormalizedQuad();

	_depthBufH.getFBO().disableFBORendering();
	glBindTexture(GL_TEXTURE_2D, 0);

	//render pass 2
	_depthBufV.activate();
	_depthBufH.enableTexture(GL_TEXTURE0);
	glMultiTexCoord2f(GL_TEXTURE4, 0.0f, dt);

	V3D_GPU::renderNormalizedQuad();

	//after render - disable
	shader->disable();
	_depthBufV.getFBO().disableFBORendering();
	glBindTexture(GL_TEXTURE_2D, 0);

	//combine shader
	_depthBufH.activate();

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, inTex);
	_depthBufV.enableTexture(GL_TEXTURE1);

	shader2->enable();
	glMultiTexCoord1f(GL_TEXTURE4, 200);

	V3D_GPU::renderNormalizedQuad();

	shader2->disable();
	_depthBufH.getFBO().disableFBORendering();
	glDisable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE0);

	//copy result back
	//rtt_buffer_temp = new V3D_GPU::RTT_Buffer("r=32f enableTextureRG", "rtt_buffer");
	//rtt_buffer_temp->allocate(w, h);

	beingthere::copyTexture(_depthBufH.textureID(), outTex);
}