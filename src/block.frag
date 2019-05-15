#version 450
layout(location = 0) out vec4 out_color;

layout(binding = 1) uniform texture2D t_Color;
layout(binding = 2) uniform sampler s_Color;
//layout(binding = 1) uniform sampler2D s_texture;

layout(location = 0) in vec2 v_tex_coord;
layout(location = 1) in vec3 v_color;

void main() {
    vec4 tex_color = texture(sampler2D(t_Color, s_Color), v_tex_coord);
    if(tex_color.a == 0.0) // Discard transparent pixels.
        discard;
    out_color = tex_color * vec4(v_color, 1.0);
    // out_color = vec4(v_tex_coord, 0.5, 1.0);
    // out_color = vec4(v_color, 1.0);
    // out_color = vec4(tex_color.rgb, 1.0);
}