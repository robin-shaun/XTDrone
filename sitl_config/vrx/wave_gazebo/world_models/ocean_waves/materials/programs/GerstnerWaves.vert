// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.s


// Copyright (c) 2019 Rhys Mainwaring.
//
// Modified to accept vector parameters and use the form
// for Gerstner waves published in:
//
// Jerry Tessendorf, "Simulating Ocean Water", 1999-2004
//
// theta = k * dir . x - omega * t
//
// px = x - dir.x * a * k * sin(theta)
// py = y - dir.y * a * k * sin(theta)
// pz =         a * k * cos(theta)
//
// k is the wavenumber
// omega is the angular frequency
//
// The derivative terms (Tangent, Binormal, Normal) have been
// updated to be consistent with this convention.
//

/////////// Input parameters //////////
// Waves
uniform int Nwaves;
uniform vec3 eyePos;
uniform float rescale;
uniform vec2 bumpScale;
uniform vec2 bumpSpeed;
uniform float time;
uniform vec3 amplitude;
uniform vec3 wavenumber;
uniform vec3 omega;
uniform vec3 steepness;
uniform vec2 dir0;
uniform vec2 dir1;
uniform vec2 dir2;
uniform float tau;

/////////// Output variables to fragment shader //////////
varying mat3 rotMatrix;
varying vec3 eyeVec;
varying vec2 bumpCoord;

// Rtt coordinates
varying vec4 projectionCoord;

// Compute linear combination of Gerstner waves as described in
// GPU Gems, chapter 01: "Effective Water Simulation from Physical Models"
// http://http.developer.nvidia.com/GPUGems/gpugems_ch01.html

// Information regarding a single wave
struct WaveParameters {
  float k;      // wavenumber
  float a;      // amplitude
  float omega;  // phase constant of speed
  vec2 d;       // horizontal direction of wave
  float q;      // steepness for Gerstner wave (q=0: rolling sine waves)
};

void main(void)
{
  // Use combination of three waves. Values here are chosen rather arbitrarily.
  // Other parameters might lead to better-looking waves.

  WaveParameters waves[3];
  
  waves[0] = WaveParameters(wavenumber.x, amplitude.x, omega.x, dir0.xy, steepness.x);
  waves[1] = WaveParameters(wavenumber.y, amplitude.y, omega.y, dir1.xy, steepness.y);
  waves[2] = WaveParameters(wavenumber.z, amplitude.z, omega.z, dir2.xy, steepness.z);

  vec4 P = gl_Vertex;

  // Iteratively compute binormal, tangent, and normal vectors:
  vec3 B = vec3(1.0, 0.0, 0.0);
  vec3 T = vec3(0.0, 1.0, 0.0);
  vec3 N = vec3(0.0, 0.0, 1.0);

  // Wave synthesis using linear combination of Gerstner waves
  for(int i = 0; i < Nwaves; ++i)
  {
    // Evaluate wave equation:
    float k = waves[i].k;
    float a = waves[i].a * (1.0 - exp(-1.0*time/tau));
    float q = waves[i].q;
    float dx = waves[i].d.x;
    float dy = waves[i].d.y;
    float theta = dot(waves[i].d, P.xy)*k - time*waves[i].omega;
    float c = cos(theta);
    float s = sin(theta);

    // Displacement of point due to wave (Eq. 9)
    P.x -= q*a*dx*s;
    P.y -= q*a*dx*s;
    P.z += a*c;

    // Modify normals due to wave displacement (Eq. 10-12)
    float ka = a*k;
    float qkac = q*ka*c;
    float kas = ka*s;
    float dxy = dx*dy;

    B += vec3(-qkac*dx*dx, -qkac*dxy, -kas*dx);
    T += vec3(-qkac*dxy, -qkac*dy*dy, -kas*dy);
    N += vec3(dx*kas, dy*kas, -qkac);
  }

  // Compute (Surf2World * Rescale) matrix
  B = normalize(B)*rescale;
  T = normalize(T)*rescale;
  N = normalize(N);
  rotMatrix = mat3(B, T, N);

  gl_Position = gl_ModelViewProjectionMatrix*P;

  // Compute texture coordinates for bump map
  bumpCoord = gl_MultiTexCoord0.xy*bumpScale + time*bumpSpeed;

  eyeVec = P.xyz - eyePos; // eye position in vertex space

  // Calculate projection coordinates for refl/refr
  // Projective texture coordinates, adjust for mapping
  mat4 scalemat = mat4( 0.5,  0.0,  0.0,  0.0,
                        0.0, -0.5,  0.0,  0.0,
                        0.0,  0.0,  0.5,  0.0,
                        0.5,  0.5,  0.5,  1.0);
  projectionCoord = scalemat * gl_Position;
}
