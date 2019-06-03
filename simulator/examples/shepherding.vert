varying vec4 ShadowCoord;
varying vec4 diffuse,ambient;
varying vec3 normal,lightDir,halfVector;

uniform mat4 V;
uniform mat4 M;


void main()
{	
	// phone shading 
	normal = normalize(gl_NormalMatrix * gl_Normal); //normal in camera space

    vec3 vertexPosition_cameraspace = vec3(gl_ModelViewMatrix*gl_Vertex);
  
	lightDir = normalize(vec3(gl_LightSource[0].position) - vertexPosition_cameraspace);

  // Normalize the halfVector to pass it to the fragment shader 
  {
    // compute eye vector and normalize it 
    vec3 eye = normalize(-vertexPosition_cameraspace);

    // compute the half vector
    halfVector = normalize(lightDir + eye);
  }


	diffuse = gl_FrontMaterial.diffuse * gl_LightSource[0].diffuse;
	ambient = gl_FrontMaterial.ambient * gl_LightSource[0].ambient;
	ambient += gl_LightModel.ambient * gl_FrontMaterial.ambient;

	gl_Position = ftransform(); //gl_ProjectionMatrix*V*M*gl_Vertex; //ftransform();

	// shadow map in light coordinate....
	ShadowCoord= gl_TextureMatrix[0] * M *gl_Vertex;
	gl_FrontColor = gl_Color;
}

