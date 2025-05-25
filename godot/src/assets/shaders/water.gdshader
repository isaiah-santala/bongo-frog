// Water Shader for Godot 4.4 - Compatibility Renderer Compatible
// This shader is designed to work with the Compatibility renderer (GLES3)
shader_type spatial;

// Basic render modes - keep minimal for compatibility
render_mode cull_disabled, depth_draw_opaque, depth_test_disabled, diffuse_burley, specular_schlick_ggx;

// Water appearance uniforms
uniform vec4 water_color : source_color = vec4(0.1, 0.3, 0.5, 0.8);
uniform vec4 deep_water_color : source_color = vec4(0.0, 0.1, 0.3, 1.0);
uniform float transparency : hint_range(0.0, 1.0) = 0.8;
uniform float metallic : hint_range(0.0, 1.0) = 0.1;
uniform float roughness : hint_range(0.0, 1.0) = 0.1;

// Wave animation uniforms
uniform float wave_speed : hint_range(0.0, 2.0) = 0.5;
uniform float wave_height : hint_range(0.0, 1.0) = 0.1;
uniform float wave_frequency : hint_range(0.1, 10.0) = 2.0;
uniform float wave_scale : hint_range(0.1, 10.0) = 1.0;
uniform vec2 wave_direction_1 = vec2(1.0, 0.0);
uniform vec2 wave_direction_2 = vec2(0.0, 1.0);

// Foam uniforms
uniform vec4 foam_color : source_color = vec4(1.0, 1.0, 1.0, 1.0);
uniform float foam_threshold : hint_range(0.0, 1.0) = 0.1;
uniform float foam_size : hint_range(0.01, 1.0) = 0.05;

// Noise textures - optional but recommended
uniform sampler2D wave_noise : hint_default_black;
uniform sampler2D normal_map : hint_normal;
uniform float normal_strength : hint_range(0.0, 2.0) = 1.0;
uniform float texture_scale : hint_range(0.1, 10.0) = 1.0;

// Built-in textures for depth and screen effects
uniform sampler2D DEPTH_TEXTURE : hint_depth_texture, filter_linear_mipmap;
uniform sampler2D SCREEN_TEXTURE : hint_screen_texture, filter_linear_mipmap;

// Depth fade uniforms
uniform float depth_fade_distance : hint_range(0.1, 10.0) = 1.0;
uniform float edge_fade : hint_range(0.01, 1.0) = 0.1;

// Varying variables
varying vec3 world_position;
varying float vertex_height;
varying vec2 animated_uv;

// Utility functions
float fresnel_effect(float amount, vec3 normal, vec3 view) {
    return pow(1.0 - clamp(dot(normalize(normal), normalize(view)), 0.0, 1.0), amount);
}

// Simple wave function - more compatible than complex Gerstner waves
float generate_wave(vec2 pos, vec2 direction, float frequency, float time_offset) {
    float wave = sin(dot(pos, direction) * frequency + time_offset);
    return wave * 0.5 + 0.5; // Normalize to 0-1 range
}

void vertex() {
    // Calculate world position
    world_position = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;

    // Animate UV coordinates with scale control
    animated_uv = UV * texture_scale + TIME * wave_speed * 0.1;

    // Generate waves using simple sine waves for compatibility
    float time_factor = TIME * wave_speed;

    // Apply wave scale to world position for wave generation
    vec2 scaled_pos = world_position.xz * wave_scale;

    // Combine two wave directions for more natural look
    float wave1 = generate_wave(scaled_pos, wave_direction_1, wave_frequency, time_factor);
    float wave2 = generate_wave(scaled_pos, wave_direction_2, wave_frequency * 0.7, time_factor * 1.3);

    // Add noise texture if available
    float noise_sample = 0.0;
    if (textureSize(wave_noise, 0).x > 1) {
        noise_sample = texture(wave_noise, scaled_pos * 0.1 + TIME * wave_speed * 0.05).r;
    }

    // Combine waves and apply height (centered around 0)
    vertex_height = ((wave1 + wave2 * 0.5 + noise_sample) - 1.0) * wave_height;
    VERTEX.y += vertex_height;
}

void fragment() {
    // Sample depth texture for edge detection
    float depth_sample = texture(DEPTH_TEXTURE, SCREEN_UV).r;

    // Convert depth to linear space (simplified for compatibility)
    float linear_depth = depth_sample;
    float fragment_depth = FRAGCOORD.z;

    // Calculate depth difference for foam and transparency effects
    float depth_diff = abs(linear_depth - fragment_depth);

    // Generate animated UV for textures
    vec2 distorted_uv = animated_uv;

    // Sample normal map if available (use scaled UV)
    vec3 normal_sample = vec3(0.0, 0.0, 1.0);
    if (textureSize(normal_map, 0).x > 1) {
        normal_sample = texture(normal_map, distorted_uv).rgb;
        // Convert from [0,1] to [-1,1] range
        normal_sample = normal_sample * 2.0 - 1.0;
        normal_sample.xy *= normal_strength;
    }

    // Calculate fresnel for water transparency effect
    float fresnel = fresnel_effect(2.0, NORMAL, VIEW);

    // Mix water colors based on depth and fresnel
    vec4 final_color = mix(water_color, deep_water_color, fresnel * 0.5);

    // Add foam at edges
    float foam_factor = 1.0 - smoothstep(0.0, foam_threshold, depth_diff);
    foam_factor = pow(foam_factor, 2.0); // Make foam sharper

    final_color = mix(final_color, foam_color, foam_factor);

    // Apply screen texture distortion (subtle for compatibility)
    vec2 screen_distortion = normal_sample.xy * 0.01;
    vec4 screen_sample = texture(SCREEN_TEXTURE, SCREEN_UV + screen_distortion);

    // Blend with screen for refraction effect
    final_color.rgb = mix(screen_sample.rgb, final_color.rgb, transparency);

    // Set material properties
    ALBEDO = final_color.rgb;
    ALPHA = final_color.a;
    METALLIC = metallic;
    ROUGHNESS = roughness;
    NORMAL_MAP = normal_sample;

    // Add some emission for water glow (optional)
    EMISSION = final_color.rgb * 0.05;
}