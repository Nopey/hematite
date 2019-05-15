#version 450

layout(location = 0) in vec3 at_position;
layout(location = 1) in vec2 at_tex_coord;
layout(location = 2) in vec3 at_color;

layout(set = 0, binding = 0) uniform Locals {
    mat4 u_projection;
    mat4 u_view;
};

layout(location = 0) out vec2 v_tex_coord;
layout(location = 1) out vec3 v_color;

void main() {
    v_tex_coord = at_tex_coord;
    v_color = at_color;
    gl_Position = u_projection * u_view * vec4(at_position, 1.0);
}