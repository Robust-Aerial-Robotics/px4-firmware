/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file poly_path.cpp
 *
 * Definition of a 1D path constructed from a sequence of polynomials.
 *
 * @author Kevin Cohen <kevincohen35@gmail.com>
 *
 */

#include "poly_path.hpp"
#include <matrix/math.hpp>

template <int numCoeff>
Polynomial<numCoeff>::Polynomial(float coeffs[]){
	_numCoeff = numCoeff;
	_setCoeffs(coeffs);
}

template <int numCoeff>
Polynomial<numCoeff>::Polynomial(const Polynomial<numCoeff> &orig){
	_numCoeff = numCoeff;
	const float* coeffs = orig.getCoeffs();
	_setCoeffs(coeffs);
}

template <int numCoeff>
Polynomial<numCoeff> Polynomial<numCoeff>::getDeriv(){
	float derivCoeffs[numCoeff];
	for(int i=1;i<numCoeff;i++)
		derivCoeffs[i-1] = _coeffs[i]*i;

	Polynomial<numCoeff> deriv(derivCoeffs);
	return deriv;
}

template <int numCoeff>
float Polynomial<numCoeff>::getEval(float t){
	float eval = _coeffs[0];
	float t_pow = 1;
	for(int i=1;i<_numCoeff;i++){
		t_pow *= t;
		eval += _coeffs[i] * t_pow;
	}
	return eval;
}

template <int numCoeff, int numPoly>
Poly_Path<numCoeff, numPoly>::Poly_Path(float taus[], Polynomial<numCoeff> polyList[]){
	_numPoly = numPoly;
	_numCoeff = numCoeff;

	_polyList0[0] = polyList[0];
	_taus[0] = taus[0];
	_cumTau[0] = _taus[0];
	for(int i=1;i<_numPoly;i++){
		_polyList0[i] = polyList[i];
		_taus[i] = taus[i];
		_cumTau[i] = _taus[i] + _cumTau[i-1];
	}
	_genDerivs();
}

template <int numCoeff, int numPoly>
void Poly_Path<numCoeff, numPoly>::_genDerivs(){
	for(int i=0;i<numPoly;i++){
		_polyList1[i] = _polyList0[i].getDeriv();
		_polyList2[i] = _polyList1[i].getDeriv();
		_polyList3[i] = _polyList2[i].getDeriv();
	}
}

template <int numCoeff, int numPoly>
void Poly_Path<numCoeff, numPoly>::getEval(float t, float* derivs){
	for(int i=0; i<_numPoly; i++){
		if(_cumTau[i] >= t){
			if(i==0){
				derivs[0] = _polyList0[i].getEval(t);
				derivs[1] = _polyList1[i].getEval(t);
				derivs[2] = _polyList2[i].getEval(t);
				derivs[3] = _polyList3[i].getEval(t);
				return;
			}
			else{
				derivs[0] = _polyList0[i].getEval(t-_cumTau[i-1]);
				derivs[1] = _polyList1[i].getEval(t-_cumTau[i-1]);
				derivs[2] = _polyList2[i].getEval(t-_cumTau[i-1]);
				derivs[3] = _polyList3[i].getEval(t-_cumTau[i-1]);
				return;
			}
		}
	}
}