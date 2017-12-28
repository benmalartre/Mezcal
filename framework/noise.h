#pragma once

//#include "vector2.h"
#include "vector3.h"

namespace BOB {


float noise_1f_grad_hash( const float x );
//float noise_1f_grad_hash( const Vector2 & x, const int pe);
float noise_1f_grad_hash( const Vector3 & x);

float noise_1f_grad_lut( const float x );
//Vector2  noise_2f_grad_hash(const Vector2 & x, const int pe = 255);
//vec2  noise_2f_grad_hash(const vec3 & x );
Vector3  noise_3f_grad_hash(const float  x);
//Vector3  noise_3f_grad_hash(const Vector2 & x);
Vector3 noise_3f_grad_hash(const Vector3 & x);

Vector3  noise_3f_value_hash( const Vector3 & x );
float noise_1f_value_hash( const float x );
//Vector2  noise_2f_value_hash( const float x );
Vector3  noise_3f_value_hash( const float x );
//float noise_1f_value_hash( const Vector2 & x );
//float noise_1f_value_hash( const Vector2 & x );
//Vector3  noise_3f_value_hash( const Vector2 & x );
float noise_1f_value_hash( const Vector3 & x );
//Vector2  noise_2f_value_hash( const vec3 & x );
Vector3  noise_3f_value_hash( const Vector3 & x );

} // namespace BOB
