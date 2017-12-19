#version 430 core

in vec4 vertex;
in vec4 inColor;

out VS_OUT
{
  vec4 fP;
  vec4 out_color;
}vs_out;

uniform mat4 mvp_matrix;
uniform mat4 mv_matrix;

void main(void)
{
   vs_out.out_color=inColor;
   vs_out.fP = mv_matrix * vertex;
   gl_Position = mvp_matrix * vertex;
}
