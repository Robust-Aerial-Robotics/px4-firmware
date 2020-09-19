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
 * @file poly_path.hpp
 * Implementation of a 1D path constructed from a sequence of polynomials.
 *
 * @author Kevin Cohen <kevincohen35@gmail.com>
 * Acknowledgements and References:
 *    Optimal polynomial construction from fixed conditions is based on the approach discussed in [1].
 *
 *    [1] Bry, A., Richter, C., Bachrach, A., and Roy, N., “Aggressive flight of fixed-wing and quadrotor aircraft in dense indoor environments,”
 *    The International Journal of Robotics Research, Vol. 34, No. 7, 2015, pp. 969–1002. https://doi.org/10.1177/0278364914558129, 
 *    URL https://doi.org/10.1177/0278364914558129.
 *    
 */

#ifndef POLY_PATH_H
#define POLY_PATH_H

#include <matrix/math.hpp>
using matrix::Matrix;
using matrix::SquareMatrix;

template <int numCoeff>
class Polynomial{
	public:
		Polynomial() = default; // Default constructor
		Polynomial(float coeffs[]); // Construct from coefficients
		Polynomial(const Polynomial<numCoeff> &orig); // Copy constructor
		Polynomial<numCoeff> &operator=(const Polynomial<numCoeff> &orig); // Copy via = operator

		Polynomial getDeriv(); // Return derivative of this polynomial
		
		float getEval(float t); // Evaluate polynomial at t
		const float* getCoeffs() const { return _coeffs; }
	private:
		void _setCoeffs(const float coeffs[]){ for(int i=0;i<_numCoeff;i++) _coeffs[i] = coeffs[i]; } // Set coefficients by copy
		float _coeffs[numCoeff]; // List of coefficients
		int _numCoeff; // Number of coefficients
};

template <int numCoeff, int numPoly>
class Poly_Path{
	public:
		Poly_Path() = default; // Default constructor
		~Poly_Path();
		void update(float taus[], Polynomial<numCoeff> polyList[]); // Construct path from leg durations and corresponding polynomial
		void update(float taus[], float ics[], float points[], float costs[]);
		void getEval(float t, float* derivs); // Evaluate a derivative of the path at t. Select order with derivs
		Polynomial<numCoeff> _polyList0[numPoly]; // List of polynomials
		Polynomial<numCoeff> _polyList1[numPoly]; // 1st derivative of _polyList0
		Polynomial<numCoeff> _polyList2[numPoly]; // 2nd derivative of _polyList0
		Polynomial<numCoeff> _polyList3[numPoly]; // 3rd derivative of _polyList0

	private:
		float _dP_dp(float t, int r, int n);
		void _genDerivs(); // Sets _polyList1, polyList2, polyList3
		float _cumTau[numPoly];
		float _taus[numPoly];

		int _numCoeff;
		int _numPoly;

		// Heap allocations for *big* matrices used to compute optimal path 
		#define BC_PER_LEG 8 // Number of boundary values per leg that are mapped from coefficients by matrix A
		#define FIXED_COND_PER_LEG 5 // Number of conditions per leg that are inputs to optimization ("specified")
		#define FREE_COND_PER_LEG 2 // Number of conditions per leg that are outputs of optimization ("free" or "optimized")
		static const int fSize{FIXED_COND_PER_LEG*numPoly};
		static const int pSize{FREE_COND_PER_LEG*numPoly};
		Matrix<float, BC_PER_LEG*numPoly, numCoeff*numPoly>* _A = new Matrix<float, BC_PER_LEG*numPoly, numCoeff*numPoly>; // A in eq. 15 of Bry & Richter
		Matrix<float, numCoeff*numPoly, BC_PER_LEG*numPoly>* _C = new Matrix<float, numCoeff*numPoly, BC_PER_LEG*numPoly>;
		SquareMatrix<float, numCoeff*numPoly>* _Q_block = new SquareMatrix<float, numCoeff*numPoly>;
		SquareMatrix<float, numCoeff*numPoly>* _ACT = new SquareMatrix<float, numCoeff*numPoly>;
		SquareMatrix<float, numCoeff*numPoly>* _R = new SquareMatrix<float, numCoeff*numPoly>;
		Matrix<float, fSize, pSize>* _R_fp = new Matrix<float, fSize, pSize>;
		SquareMatrix<float, pSize>* _R_pp = new SquareMatrix<float, pSize>;
};

#endif /* POLY_PATH_H */
