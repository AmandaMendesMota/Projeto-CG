// Copyright (c) 2019, Danilo Peixoto. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Evita redefinição de símbolos do arquivo de cabeçalho (caso já tenha sido incluído)
#ifndef AURORA_MATH_H
#define AURORA_MATH_H

#include <aurora/Global.h>

// Constantes numéricas
#define AURORA_E         2.71828183f
#define AURORA_PI        3.14159265f
#define AURORA_INV_PI    0.31830989f
#define AURORA_SQRT_2    1.41421356f
#define AURORA_EPSILON   1.17549435e-38f
#define AURORA_INFINITY  3.40282347e+38f
#define AURORA_THRESHOLD 1.00000000e-06f

AURORA_NAMESPACE_BEGIN // Início do "namespace" da biblioteca

// Retorna valor delimitado por mínimo e máximo
float clamp(float x, float a, float b);
// Retorna valor delimitado no intervalo [0, 1]
float saturate(float x);
// Converte ângulo de radiano para grau
float degrees(float x);
// Converte ângulo de grau para radiano
float radians(float x);

// Fim de "namespace" da biblioteca
AURORA_NAMESPACE_END

#endif
