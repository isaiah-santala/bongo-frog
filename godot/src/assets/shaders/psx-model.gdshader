shader_type spatial;
render_mode diffuse_burley, specular_schlick_ggx, blend_mix;

// Basic parameters
uniform float vertex_precision : hint_range(0.1, 2048.0) = 512.0; // Controls grid size for snapping
uniform float pixel_size : hint_range(1.0, 32.0) = 1.0;
uniform bool affine_texture_mapping = true;
uniform int color_depth : hint_range(2, 256) = 128; // Color quantization
uniform float edge_contrast : hint_range(0.0, 100.0) = 32.0; // Controls edge sharpness

// Alpha transparency control
uniform bool enable_alpha_transparency = false;
uniform float alpha_value : hint_range(0.0, 1.0) = 1.0;

// Texture tiling parameters
uniform float tiling_scale_x : hint_range(0.1, 10.0) = 1.0; // Controls horizontal tiling
uniform float tiling_scale_y : hint_range(0.1, 10.0) = 1.0; // Controls vertical tiling
uniform float rotation_angle : hint_range(0.0, 360.0) = 0.0; // Rotation in degrees
uniform float albedo_brightness : hint_range(0.0, 2.0) = 1.0; // Controls overall brightness
uniform sampler2D TEXTURE : source_color, filter_nearest_mipmap_anisotropic;
uniform vec4 tint_color : source_color = vec4(1.0, 1.0, 1.0, 1.0);
uniform float tint_intensity : hint_range(0.0, 1.0) = 0.5;

varying float vertex_distance;
varying vec3 vertex_normal;

// Function to quantize colors more aggressively
vec3 quantize_color(vec3 color) {
    return floor(color * float(color_depth)) / float(color_depth);
}

// Function to enhance edges
vec3 enhance_edges(vec3 color, vec3 normal, float edge_strength) {
    float edge = abs(dot(normal, vec3(0.0, 0.0, 1.0)));
    edge = 1.0 - pow(edge, edge_strength);
    return mix(color, quantize_color(color), edge);
}

// Function to rotate UV coordinates
vec2 rotate_uv(vec2 uv, float angle_degrees) {
    float angle = radians(angle_degrees);
    float sin_angle = sin(angle);
    float cos_angle = cos(angle);
    // Center UV coordinates before rotation
    vec2 centered_uv = uv - 0.5;
    // Perform rotation
    vec2 rotated_uv = vec2(
        centered_uv.x * cos_angle - centered_uv.y * sin_angle,
        centered_uv.x * sin_angle + centered_uv.y * cos_angle
    );
    // Return to original UV space
    return rotated_uv + 0.5;
}

void vertex() {
    // Transform to view space (camera space)
    vec4 view_pos = MODELVIEW_MATRIX * vec4(VERTEX, 1.0);
    // Apply snapping in view space before projection
    view_pos.xyz = floor(view_pos.xyz * vertex_precision) / vertex_precision;
    // Transform to clip space
    vec4 clip_position = PROJECTION_MATRIX * view_pos;
    // Store distance for perspective correction
    vertex_distance = clip_position.w;
    // Pass normal to fragment shader for edge detection
    vertex_normal = (MODELVIEW_MATRIX * vec4(NORMAL, 0.0)).xyz;
    // Output final position
    POSITION = clip_position;
    // Handle affine texture mapping
    if (affine_texture_mapping) {
        UV *= vertex_distance;
    }
}

void fragment() {
    vec2 uv = UV;
    // Apply perspective correction for texture coordinates
    if (affine_texture_mapping) {
        uv /= vertex_distance;
    }
    // Apply tiling scale
    uv *= vec2(tiling_scale_x, tiling_scale_y);
    // Apply rotation
    uv = rotate_uv(fract(uv), rotation_angle);
    // Pixelate the texture sampling
    vec2 texture_size = vec2(textureSize(TEXTURE, 0));
    vec2 pixelated_uv = floor(uv * texture_size / pixel_size) * pixel_size / texture_size;
    // Sample the texture
    vec4 color = texture(TEXTURE, pixelated_uv);
    // Apply color quantization and edge enhancement
    vec3 final_color = color.rgb;
    final_color = enhance_edges(final_color, normalize(vertex_normal), 1.0 + edge_contrast * 3.0);
    final_color = quantize_color(final_color);
    // Apply additional sharpening
    float luma = dot(final_color, vec3(0.299, 0.587, 0.114));
    float luma_quantized = floor(luma * float(color_depth)) / float(color_depth);
    final_color = mix(final_color, final_color * (1.0 + edge_contrast), abs(luma - luma_quantized));
    vec3 tinted_color = mix(final_color, final_color * tint_color.rgb, tint_intensity);
    ALBEDO = tinted_color * albedo_brightness;

    // Handle alpha transparency
    if (enable_alpha_transparency) {
        ALPHA = color.a * alpha_value;
    } else {
        ALPHA = color.a;
    }

    ROUGHNESS = 1.0;
    SPECULAR = 0.0;
}