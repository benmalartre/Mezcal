#version 330 core
in vec2 texCoords;
out float outColor;

uniform sampler2D ssao_map;


void main() {
    vec2 texelSize = 1.0 / vec2(textureSize(ssao_map, 0));
    float result = 0.0;
    for (int x = -2; x < 2; ++x) 
    {
        for (int y = -2; y < 2; ++y) 
        {
            vec2 offset = vec2(float(x), float(y)) * texelSize;
            result += texture(ssao_map, texCoords + offset).r;
        }
    }
    outColor = result / (4.0 * 4.0);
}  
