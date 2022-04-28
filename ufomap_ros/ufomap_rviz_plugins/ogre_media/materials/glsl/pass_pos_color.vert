#version 150 compatibility

// This merely passes over the position and color

out gl_PerVertex
{
	vec4 gl_Position;
	vec4 gl_FrontColor;
};

void main()
{
	gl_Position = gl_Vertex;
	gl_FrontColor = gl_Color;
}