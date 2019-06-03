#ifndef _SHADOW_MAP_GLSL_
#define _SHADOW_MAP_GLSL_

#include <GL/glew.h>

#ifdef MACOS
#include <GLUT/glut.h>
#else
#include <stdlib.h>
#include <GL/glut.h>
#endif

#include <iostream>

// if reshape the window, remember to do:
//
//		ShadowMap_GLSL::clean();
//		ShadowMap_GLSL::generateShadowFBO();
//		ShadowMap_GLSL::loadShadowShader();
//



#define SHADOW_MAP_RATIO (1.0)

class ShadowMap_GLSL
{
public:

	ShadowMap_GLSL(void);
	~ShadowMap_GLSL(void);

	static GLuint loadShader(unsigned char* filename, unsigned int type);
	static void loadShadowShader();
	static void loadShadowShader(const char * vertex_shader, const char * frag_shader);

	static void generateShadowFBO(int w, int h, float lightPx, float lightPy, float lightPz);
	static void setupMatrices(float position_x,float position_y,float position_z,float lookAt_x,float lookAt_y,float lookAt_z);
	static void setTextureMatrix(void);
	static void clean(void);
	static void beginShadowMap();
	static void endShadowMap();
	static void beginSceneRender();
	static void endSceneRender();
	static void printLog(GLuint obj);

	static void setM(float * M)
	{
		glUniformMatrix4fv(shadowMap_ModelMatrix_Uniform, 1, GL_FALSE, M);
	}

	static void setV(float *  V)
	{
		glUniformMatrix4fv(shadowMap_ViewMatrix_Uniform, 1, GL_FALSE, V);
	}

	// Hold id of the framebuffer for light POV rendering
	static GLuint fboId;

	// Z values will be rendered to this texture when using fboId framebuffer
	static GLuint depthTextureId;

	// Use to activate/disable shadowShader
	static GLuint shadowShaderId;

	static GLuint shadowMapUniform;
	static GLuint shadowMapStepXUniform;
	static GLuint shadowMapStepYUniform;
	static GLuint shadowMap_ViewMatrix_Uniform;
	static GLuint shadowMap_ModelMatrix_Uniform;

	// Use to active/disable lightShader
	static GLuint lightShaderId;

	static float lightPos[3];
	static float screenWidth;
	static float screenHeight;
};

#endif
