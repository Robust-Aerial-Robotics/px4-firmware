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
 * Definition of a 1D path constructed from a sequence of polynomials.
 * Authors and acknowledgements in header.
 *
 */

#include "poly_path.hpp"

using matrix::Slice;
using matrix::Vector;

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
Polynomial<numCoeff> &Polynomial<numCoeff>::operator=(const Polynomial<numCoeff> &orig){
	_numCoeff = numCoeff;
	const float* coeffs = orig.getCoeffs();
	_setCoeffs(coeffs);

	return (*this);
}


template <int numCoeff>
Polynomial<numCoeff> Polynomial<numCoeff>::getDeriv(){
	float derivCoeffs[numCoeff];
	for(int i=1;i<numCoeff;i++)
		derivCoeffs[i-1] = _coeffs[i]*i;
	derivCoeffs[numCoeff-1] = 0;

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
Poly_Path<numCoeff, numPoly>::Poly_Path(float taus[], float ics[], float points[], float costs[]){
	_numPoly = numPoly;
	_numCoeff = numCoeff;
	_taus[0] = taus[0];
	_cumTau[0] = _taus[0];
	for(int i=1;i<numPoly;i++){
		_taus[i] = taus[i];
		_cumTau[i] = _taus[i] + _cumTau[i-1];
	}

	// A in eq. 15 of Bry & Richter
	for(int legNumber=0; legNumber<numPoly; legNumber++){
		for(int derivOrder=0;derivOrder<4;derivOrder++){
			for(int coeffOrder=0;coeffOrder<_numCoeff;coeffOrder++){
				_A->operator()(derivOrder+legNumber*BC_PER_LEG  , coeffOrder+legNumber*numCoeff) = _dP_dp(0					, derivOrder, coeffOrder);
				_A->operator()(derivOrder+legNumber*BC_PER_LEG+4, coeffOrder+legNumber*numCoeff) = _dP_dp(_taus[legNumber]	, derivOrder, coeffOrder);
			}
		}
	}

	// NOTATION: P^(1)(2)_3 would mean the 1st derivative of the 3rd polynomial in the path evaluated at 2.
	// continued.... ^T is the transpose operator

	// C is a matrix that maps vector [P^(0)(0)_0,  P^(1)(0)_0,  P^(2)(0)_0,  P^(3)(0)_0,
	//                                 P^(0)(tau)_0,P^(1)(tau)_0,P^(2)(tau)_0,P^(3)(tau)_0,
	//                                 P^(0)(0)_1,  P^(1)(0)_1,  P^(2)(0)_1,  P^(3)(0)_1,
	//                                 P^(0)(tau)_1,P^(1)(tau)_1,P^(2)(tau)_1,P^(3)(tau)_1,
	//                                 .
	//                                 .
	//                                 .]^T
	// to the vector of conditions that define the polynomials: [b_F^T, b_P^T]^T

	// First leg conditions
	#define FIXED_COND_PER_LEG 5 // Number of conditions per leg that are inputs to optimization ("specified")
	_C->operator()(0, 0) = 1; // Specify P^(0)(0)_0
	_C->operator()(1, 1) = 1; // Specify P^(1)(0)_0
	_C->operator()(2, 2) = 1; // Specify P^(2)(0)_0
	_C->operator()(3, 3) = 1; // Specify P^(3)(0)_0
	_C->operator()(4, 4) = 1; // Specify P^(3)(tau)_0

	#define FREE_COND_PER_LEG 2 // Number of conditions per leg that are outputs of optimization ("free" or "optimized")
	_C->operator()(5 +(numPoly-1)*FIXED_COND_PER_LEG, 5) = 1; // Optimize P(1)(tau)_0
	_C->operator()(6 +(numPoly-1)*FIXED_COND_PER_LEG, 6) = 1; // Optimize P(2)(tau)_0

	// All other leg conditions
	for(int legNumber=1; legNumber<numPoly;legNumber++){
		_C->operator()(0 +legNumber*FIXED_COND_PER_LEG, 0 +legNumber*BC_PER_LEG) = 1; _C->operator()(0 +legNumber*FIXED_COND_PER_LEG, 4+(legNumber-1)*BC_PER_LEG) = -1; // Specify P^(0)(0)_legNumber - P^(0)(tau)_legNumber-1
		_C->operator()(1 +legNumber*FIXED_COND_PER_LEG, 1 +legNumber*BC_PER_LEG) = 1; _C->operator()(1 +legNumber*FIXED_COND_PER_LEG, 5+(legNumber-1)*BC_PER_LEG) = -1; // Specify P^(1)(0)_legNumber - P^(1)(tau)_legNumber-1
		_C->operator()(2 +legNumber*FIXED_COND_PER_LEG, 2 +legNumber*BC_PER_LEG) = 1; _C->operator()(2 +legNumber*FIXED_COND_PER_LEG, 6+(legNumber-1)*BC_PER_LEG) = -1; // Specify P^(2)(0)_legNumber - P^(2)(tau)_legNumber-1
		_C->operator()(3 +legNumber*FIXED_COND_PER_LEG, 3 +legNumber*BC_PER_LEG) = 1; _C->operator()(3 +legNumber*FIXED_COND_PER_LEG, 7+(legNumber-1)*BC_PER_LEG) = -1; // Specify P^(3)(0)_legNumber - P^(3)(tau)_legNumber-1
		_C->operator()(4 +legNumber*FIXED_COND_PER_LEG, 4 +legNumber*BC_PER_LEG) = 1; // Specify P^(0)(tau)_legNumber

		_C->operator()(5 +legNumber*FREE_COND_PER_LEG+(numPoly-1)*FIXED_COND_PER_LEG, 5 +legNumber*BC_PER_LEG) = 1; // Optimize P(1)(tau)_legNumber
		_C->operator()(6 +legNumber*FREE_COND_PER_LEG+(numPoly-1)*FIXED_COND_PER_LEG, 6 +legNumber*BC_PER_LEG) = 1; // Optimize P(2)(tau)_legNumber
	}

	// inv(A)*C^T
	*_ACT = _C->operator*(*_A);
	*_ACT = _ACT->I();

	float Q[numPoly][numCoeff][numCoeff];
	for(int r=0; r<numCoeff; r++){
		for(int i=0; i<numCoeff; i++){
			for(int l=0; l<numCoeff; l++){
				if(i>=r && l>=r){
					int pi_prod = i*l;
					for(int m=1; m<=(r-1); m++){
						pi_prod *= (i-m)*(l-m);
					} 

					float common_multiple = costs[r]*(2*pi_prod/float(i+l-2*r+1));
					if(r==0){ // Initialize if it is the first time touching this element
						for(int legNumber=0; legNumber<numPoly; legNumber++){
							Q[legNumber][i][l] = common_multiple*std::pow(taus[legNumber], (float)i+l-2*r+1);
						}
					}else{ // Add on if it is not the first time touching this element
						for(int legNumber=0; legNumber<numPoly; legNumber++){
							Q[legNumber][i][l] += common_multiple*std::pow(taus[legNumber], (float)i+l-2*r+1);
						}	
					}
				}
			}			
		}
	}

	SquareMatrix<float,numPoly> selector;
	selector.setZero();

	selector(0,0) = 1;
	*_Q_block = selector.kron(SquareMatrix<float,numCoeff>(Q[0]));
	for(int legNumber=1; legNumber<numPoly; legNumber++){
		selector(legNumber-1, legNumber-1) = 0;
		selector(legNumber, legNumber) = 1;
		*_Q_block = _Q_block->operator+( selector.kron(SquareMatrix<float,numCoeff>(Q[legNumber])) );
	}

	*_R = _ACT->T()*(_Q_block->operator*(*_ACT));

	const int fSize = FIXED_COND_PER_LEG*numPoly;
	const int pSize = FREE_COND_PER_LEG*numPoly;
	// Matrix<float, fSize, fSize> R_ff(R.slice<fSize,fSize>(0,0)); 
	Matrix<float, fSize, pSize> R_fp(_R->template slice<fSize,pSize>(0,fSize)); 
	// Matrix<float, pSize, fSize> R_pf(R.slice<pSize,fSize>(fSize,0)); 
	SquareMatrix<float, pSize> R_pp(_R->template slice<pSize,pSize>(fSize,fSize)); 

	Vector<float, fSize> b_F;
	b_F.setZero();
	for(int i=0; i<4; i++){
		b_F(i) = ics[i];
	}
	for(int i=0; i<numPoly; i++){
		b_F(4+i*FIXED_COND_PER_LEG) = points[i];
	}

	Vector<float, pSize> b_P = -R_pp.I()*R_fp.T()*b_F;
	Vector<float, numCoeff*numPoly> p = _ACT->operator*(b_F.vcat(b_P));

	float newCoeffs[numCoeff];
	for (int legNumber=0; legNumber<numPoly; legNumber++){
		for(int coeffNumber=0; coeffNumber<numCoeff; coeffNumber++){
			newCoeffs[coeffNumber] = p(coeffNumber+legNumber*numCoeff);
		}
		_polyList0[legNumber]  = Polynomial<numCoeff>(newCoeffs); // The Polynomial constructor copies the coefficients
	}

	_genDerivs();
}

template <int numCoeff, int numPoly>
Poly_Path<numCoeff, numPoly>::~Poly_Path(){
	delete _A;
	delete _C;
	delete _Q_block;
	delete _ACT;
	delete _R;
}

template <int numCoeff, int numPoly>
float Poly_Path<numCoeff, numPoly>::_dP_dp(float t, int r, int n){ //n is coeff order, r is derivative order
	int pi_product = 1;
	for(int m=0; m<r; m++){
		pi_product *= (n-m);
	}

	float t_pow = 1;
	for(int i=0; i<(n-r); i++){
		t_pow *= t;
	}

	return pi_product*t_pow;
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
