
uniform sampler2DShadow ShadowMap;
varying vec4 ShadowCoord;
uniform float xPixelOffset;
uniform float yPixelOffset;

varying vec4 diffuse, ambient;
varying vec3 normal,lightDir,halfVector;
varying vec3 Position;

float lookup( vec2 offSet)
{
	return shadow2DProj(ShadowMap, ShadowCoord + vec4(offSet.x * xPixelOffset * ShadowCoord.w, offSet.y * yPixelOffset * ShadowCoord.w, 0.00002, 0.0) ).w;
}

void main()
{	

	// phong shading
	vec3 n,l, halfV;
	float NdotL,NdotHV;		
	vec4 color = ambient*gl_Color;


	// shadow map
	float shadow=0.0;
	vec2 offset;
	float sample;


	// shadow map
	if (ShadowCoord.w > 1.0)
	{
		// PCF (4*4)
		float x,y;
    float dx=xPixelOffset * ShadowCoord.w;
    float dy=yPixelOffset * ShadowCoord.w;

		for (y = -1.5 ; y <=1.5 ; y+=1.0)
    {
			for (x = -1.5 ; x <=1.5 ; x+=1.0)
      {
				shadow += shadow2DProj(ShadowMap, ShadowCoord + vec4(x *dx , y *dy , 0.00002, 0.0) ).w;
      }
		}

		shadow /= 16.0 ;
	}


	// phong shading
	n = normalize(normal);		
  l = normalize(lightDir);    

	NdotL = max(dot(n,l),0.0);


	if (NdotL > 0.0) {
		
    color += diffuse * NdotL;
		
    halfV = normalize(halfVector);
		NdotHV = max(dot(n,halfV),0.0);

		color += gl_FrontMaterial.specular * gl_LightSource[0].specular * pow(NdotHV, gl_FrontMaterial.shininess);

	}


	// blending...
	gl_FragColor = 0.75*color + 0.25*(shadow)*gl_Color;
}
