/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"


void acado_acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 7;
const real_t* dx = in + 9;
/* Vector of auxiliary variables; number of elements: 2. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (cos(xd[2]));
a[1] = (sin(xd[2]));

/* Compute outputs: */
out[0] = (((((real_t)(0.0000000000000000e+00)-(xd[3]+xd[4]))/(real_t)(2.0000000000000000e+00))*a[0])+dx[0]);
out[1] = (((xd[3]+xd[4])/(real_t)(2.0000000000000000e+00))*a[1]);
out[2] = ((xd[3]-xd[4])/(real_t)(4.4000000000000000e-01));
out[3] = (((((real_t)(1.0473832767786508e+00)*xd[5])-((real_t)(-4.5488935141101061e-01)*u[1]))-((real_t)(1.1211034589358380e+00)*u[0]))+dx[3]);
out[4] = (((((real_t)(1.0473832767786508e+00)*xd[5])-((real_t)(-4.5488935141101061e-01)*u[0]))-((real_t)(1.1211034589358380e+00)*u[1]))+dx[4]);
out[5] = (dx[5]-xd[5]);
out[6] = (((((real_t)(5.0384343177534030e-01)*u[0])-((real_t)(-5.0384343177534030e-01)*u[1]))-((real_t)(1.6415756668643507e+01)*xd[5]))+dx[6]);
}



void acado_acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* dx = in + 9;
/* Vector of auxiliary variables; number of elements: 7. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[2])));
a[1] = ((real_t)(1.0000000000000000e+00)/(real_t)(2.0000000000000000e+00));
a[2] = (cos(xd[2]));
a[3] = (cos(xd[2]));
a[4] = ((real_t)(1.0000000000000000e+00)/(real_t)(2.0000000000000000e+00));
a[5] = (sin(xd[2]));
a[6] = ((real_t)(1.0000000000000000e+00)/(real_t)(4.4000000000000000e-01));

/* Compute outputs: */
out[0] = (real_t)(0.0000000000000000e+00);
out[1] = (real_t)(0.0000000000000000e+00);
out[2] = ((((real_t)(0.0000000000000000e+00)-(xd[3]+xd[4]))/(real_t)(2.0000000000000000e+00))*a[0]);
out[3] = ((((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[1])*a[2]);
out[4] = ((((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[1])*a[2]);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = (real_t)(0.0000000000000000e+00);
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (real_t)(1.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (((xd[3]+xd[4])/(real_t)(2.0000000000000000e+00))*a[3]);
out[19] = (a[4]*a[5]);
out[20] = (a[4]*a[5]);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = a[6];
out[36] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[6]);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(1.0473832767786508e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.1211034589358380e+00));
out[56] = ((real_t)(0.0000000000000000e+00)-(real_t)(-4.5488935141101061e-01));
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(1.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(0.0000000000000000e+00);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(0.0000000000000000e+00);
out[69] = (real_t)(1.0473832767786508e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = ((real_t)(0.0000000000000000e+00)-(real_t)(-4.5488935141101061e-01));
out[72] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.1211034589358380e+00));
out[73] = (real_t)(0.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = (real_t)(1.0000000000000000e+00);
out[78] = (real_t)(0.0000000000000000e+00);
out[79] = (real_t)(0.0000000000000000e+00);
out[80] = (real_t)(0.0000000000000000e+00);
out[81] = (real_t)(0.0000000000000000e+00);
out[82] = (real_t)(0.0000000000000000e+00);
out[83] = (real_t)(0.0000000000000000e+00);
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
out[86] = (real_t)(0.0000000000000000e+00);
out[87] = (real_t)(0.0000000000000000e+00);
out[88] = (real_t)(0.0000000000000000e+00);
out[89] = (real_t)(0.0000000000000000e+00);
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = (real_t)(0.0000000000000000e+00);
out[92] = (real_t)(0.0000000000000000e+00);
out[93] = (real_t)(0.0000000000000000e+00);
out[94] = (real_t)(1.0000000000000000e+00);
out[95] = (real_t)(0.0000000000000000e+00);
out[96] = (real_t)(0.0000000000000000e+00);
out[97] = (real_t)(0.0000000000000000e+00);
out[98] = (real_t)(0.0000000000000000e+00);
out[99] = (real_t)(0.0000000000000000e+00);
out[100] = (real_t)(0.0000000000000000e+00);
out[101] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.6415756668643507e+01));
out[102] = (real_t)(0.0000000000000000e+00);
out[103] = (real_t)(5.0384343177534030e-01);
out[104] = ((real_t)(0.0000000000000000e+00)-(real_t)(-5.0384343177534030e-01));
out[105] = (real_t)(0.0000000000000000e+00);
out[106] = (real_t)(0.0000000000000000e+00);
out[107] = (real_t)(0.0000000000000000e+00);
out[108] = (real_t)(0.0000000000000000e+00);
out[109] = (real_t)(0.0000000000000000e+00);
out[110] = (real_t)(0.0000000000000000e+00);
out[111] = (real_t)(1.0000000000000000e+00);
}







real_t acado_solve_dim7_system( real_t* const A, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 7; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (6); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*7+i]);
	for( j=(i+1); j < 7; j++ ) {
		temp = fabs(A[j*7+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 7; ++k)
{
	acadoWorkspace.rk_dim7_swap = A[i*7+k];
	A[i*7+k] = A[indexMax*7+k];
	A[indexMax*7+k] = acadoWorkspace.rk_dim7_swap;
}
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*7+i];
	for( j=i+1; j < 7; j++ ) {
		A[j*7+i] = -A[j*7+i]/A[i*7+i];
		for( k=i+1; k < 7; k++ ) {
			A[j*7+k] += A[j*7+i] * A[i*7+k];
		}
	}
}
det *= A[48];
det = fabs(det);
return det;
}

void acado_solve_dim7_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{
int i;
int index1;
int index2;
int j;

real_t tmp_var;

for (i = 0; i < 7; ++i)
{
j = rk_perm[i]*10;
acadoWorkspace.rk_dim7_bPerm[i * 10] = b[j+0];
acadoWorkspace.rk_dim7_bPerm[i * 10 + 1] = b[j+1];
acadoWorkspace.rk_dim7_bPerm[i * 10 + 2] = b[j+2];
acadoWorkspace.rk_dim7_bPerm[i * 10 + 3] = b[j+3];
acadoWorkspace.rk_dim7_bPerm[i * 10 + 4] = b[j+4];
acadoWorkspace.rk_dim7_bPerm[i * 10 + 5] = b[j+5];
acadoWorkspace.rk_dim7_bPerm[i * 10 + 6] = b[j+6];
acadoWorkspace.rk_dim7_bPerm[i * 10 + 7] = b[j+7];
acadoWorkspace.rk_dim7_bPerm[i * 10 + 8] = b[j+8];
acadoWorkspace.rk_dim7_bPerm[i * 10 + 9] = b[j+9];
}
for (j = 1; j < 7; ++j)
{
index1 = j * 10;
for (i = 0; i < j; ++i)
{
index2 = i * 10;
tmp_var = A[(j * 7) + (i)];
acadoWorkspace.rk_dim7_bPerm[index1+0] += tmp_var*acadoWorkspace.rk_dim7_bPerm[index2+0];
acadoWorkspace.rk_dim7_bPerm[index1+1] += tmp_var*acadoWorkspace.rk_dim7_bPerm[index2+1];
acadoWorkspace.rk_dim7_bPerm[index1+2] += tmp_var*acadoWorkspace.rk_dim7_bPerm[index2+2];
acadoWorkspace.rk_dim7_bPerm[index1+3] += tmp_var*acadoWorkspace.rk_dim7_bPerm[index2+3];
acadoWorkspace.rk_dim7_bPerm[index1+4] += tmp_var*acadoWorkspace.rk_dim7_bPerm[index2+4];
acadoWorkspace.rk_dim7_bPerm[index1+5] += tmp_var*acadoWorkspace.rk_dim7_bPerm[index2+5];
acadoWorkspace.rk_dim7_bPerm[index1+6] += tmp_var*acadoWorkspace.rk_dim7_bPerm[index2+6];
acadoWorkspace.rk_dim7_bPerm[index1+7] += tmp_var*acadoWorkspace.rk_dim7_bPerm[index2+7];
acadoWorkspace.rk_dim7_bPerm[index1+8] += tmp_var*acadoWorkspace.rk_dim7_bPerm[index2+8];
acadoWorkspace.rk_dim7_bPerm[index1+9] += tmp_var*acadoWorkspace.rk_dim7_bPerm[index2+9];
}
}
for (i = 6; -1 < i; --i)
{
index1 = i * 10;
for (j = 6; i < j; --j)
{
index2 = j * 10;
tmp_var = A[(i * 7) + (j)];
acadoWorkspace.rk_dim7_bPerm[index1+0] -= tmp_var*acadoWorkspace.rk_dim7_bPerm[index2+0];
acadoWorkspace.rk_dim7_bPerm[index1+1] -= tmp_var*acadoWorkspace.rk_dim7_bPerm[index2+1];
acadoWorkspace.rk_dim7_bPerm[index1+2] -= tmp_var*acadoWorkspace.rk_dim7_bPerm[index2+2];
acadoWorkspace.rk_dim7_bPerm[index1+3] -= tmp_var*acadoWorkspace.rk_dim7_bPerm[index2+3];
acadoWorkspace.rk_dim7_bPerm[index1+4] -= tmp_var*acadoWorkspace.rk_dim7_bPerm[index2+4];
acadoWorkspace.rk_dim7_bPerm[index1+5] -= tmp_var*acadoWorkspace.rk_dim7_bPerm[index2+5];
acadoWorkspace.rk_dim7_bPerm[index1+6] -= tmp_var*acadoWorkspace.rk_dim7_bPerm[index2+6];
acadoWorkspace.rk_dim7_bPerm[index1+7] -= tmp_var*acadoWorkspace.rk_dim7_bPerm[index2+7];
acadoWorkspace.rk_dim7_bPerm[index1+8] -= tmp_var*acadoWorkspace.rk_dim7_bPerm[index2+8];
acadoWorkspace.rk_dim7_bPerm[index1+9] -= tmp_var*acadoWorkspace.rk_dim7_bPerm[index2+9];
}
tmp_var = 1.0/A[i*8];
acadoWorkspace.rk_dim7_bPerm[index1+0] = tmp_var*acadoWorkspace.rk_dim7_bPerm[index1+0];
acadoWorkspace.rk_dim7_bPerm[index1+1] = tmp_var*acadoWorkspace.rk_dim7_bPerm[index1+1];
acadoWorkspace.rk_dim7_bPerm[index1+2] = tmp_var*acadoWorkspace.rk_dim7_bPerm[index1+2];
acadoWorkspace.rk_dim7_bPerm[index1+3] = tmp_var*acadoWorkspace.rk_dim7_bPerm[index1+3];
acadoWorkspace.rk_dim7_bPerm[index1+4] = tmp_var*acadoWorkspace.rk_dim7_bPerm[index1+4];
acadoWorkspace.rk_dim7_bPerm[index1+5] = tmp_var*acadoWorkspace.rk_dim7_bPerm[index1+5];
acadoWorkspace.rk_dim7_bPerm[index1+6] = tmp_var*acadoWorkspace.rk_dim7_bPerm[index1+6];
acadoWorkspace.rk_dim7_bPerm[index1+7] = tmp_var*acadoWorkspace.rk_dim7_bPerm[index1+7];
acadoWorkspace.rk_dim7_bPerm[index1+8] = tmp_var*acadoWorkspace.rk_dim7_bPerm[index1+8];
acadoWorkspace.rk_dim7_bPerm[index1+9] = tmp_var*acadoWorkspace.rk_dim7_bPerm[index1+9];
}
b[0] = acadoWorkspace.rk_dim7_bPerm[0];
b[1] = acadoWorkspace.rk_dim7_bPerm[1];
b[2] = acadoWorkspace.rk_dim7_bPerm[2];
b[3] = acadoWorkspace.rk_dim7_bPerm[3];
b[4] = acadoWorkspace.rk_dim7_bPerm[4];
b[5] = acadoWorkspace.rk_dim7_bPerm[5];
b[6] = acadoWorkspace.rk_dim7_bPerm[6];
b[7] = acadoWorkspace.rk_dim7_bPerm[7];
b[8] = acadoWorkspace.rk_dim7_bPerm[8];
b[9] = acadoWorkspace.rk_dim7_bPerm[9];
b[10] = acadoWorkspace.rk_dim7_bPerm[10];
b[11] = acadoWorkspace.rk_dim7_bPerm[11];
b[12] = acadoWorkspace.rk_dim7_bPerm[12];
b[13] = acadoWorkspace.rk_dim7_bPerm[13];
b[14] = acadoWorkspace.rk_dim7_bPerm[14];
b[15] = acadoWorkspace.rk_dim7_bPerm[15];
b[16] = acadoWorkspace.rk_dim7_bPerm[16];
b[17] = acadoWorkspace.rk_dim7_bPerm[17];
b[18] = acadoWorkspace.rk_dim7_bPerm[18];
b[19] = acadoWorkspace.rk_dim7_bPerm[19];
b[20] = acadoWorkspace.rk_dim7_bPerm[20];
b[21] = acadoWorkspace.rk_dim7_bPerm[21];
b[22] = acadoWorkspace.rk_dim7_bPerm[22];
b[23] = acadoWorkspace.rk_dim7_bPerm[23];
b[24] = acadoWorkspace.rk_dim7_bPerm[24];
b[25] = acadoWorkspace.rk_dim7_bPerm[25];
b[26] = acadoWorkspace.rk_dim7_bPerm[26];
b[27] = acadoWorkspace.rk_dim7_bPerm[27];
b[28] = acadoWorkspace.rk_dim7_bPerm[28];
b[29] = acadoWorkspace.rk_dim7_bPerm[29];
b[30] = acadoWorkspace.rk_dim7_bPerm[30];
b[31] = acadoWorkspace.rk_dim7_bPerm[31];
b[32] = acadoWorkspace.rk_dim7_bPerm[32];
b[33] = acadoWorkspace.rk_dim7_bPerm[33];
b[34] = acadoWorkspace.rk_dim7_bPerm[34];
b[35] = acadoWorkspace.rk_dim7_bPerm[35];
b[36] = acadoWorkspace.rk_dim7_bPerm[36];
b[37] = acadoWorkspace.rk_dim7_bPerm[37];
b[38] = acadoWorkspace.rk_dim7_bPerm[38];
b[39] = acadoWorkspace.rk_dim7_bPerm[39];
b[40] = acadoWorkspace.rk_dim7_bPerm[40];
b[41] = acadoWorkspace.rk_dim7_bPerm[41];
b[42] = acadoWorkspace.rk_dim7_bPerm[42];
b[43] = acadoWorkspace.rk_dim7_bPerm[43];
b[44] = acadoWorkspace.rk_dim7_bPerm[44];
b[45] = acadoWorkspace.rk_dim7_bPerm[45];
b[46] = acadoWorkspace.rk_dim7_bPerm[46];
b[47] = acadoWorkspace.rk_dim7_bPerm[47];
b[48] = acadoWorkspace.rk_dim7_bPerm[48];
b[49] = acadoWorkspace.rk_dim7_bPerm[49];
b[50] = acadoWorkspace.rk_dim7_bPerm[50];
b[51] = acadoWorkspace.rk_dim7_bPerm[51];
b[52] = acadoWorkspace.rk_dim7_bPerm[52];
b[53] = acadoWorkspace.rk_dim7_bPerm[53];
b[54] = acadoWorkspace.rk_dim7_bPerm[54];
b[55] = acadoWorkspace.rk_dim7_bPerm[55];
b[56] = acadoWorkspace.rk_dim7_bPerm[56];
b[57] = acadoWorkspace.rk_dim7_bPerm[57];
b[58] = acadoWorkspace.rk_dim7_bPerm[58];
b[59] = acadoWorkspace.rk_dim7_bPerm[59];
b[60] = acadoWorkspace.rk_dim7_bPerm[60];
b[61] = acadoWorkspace.rk_dim7_bPerm[61];
b[62] = acadoWorkspace.rk_dim7_bPerm[62];
b[63] = acadoWorkspace.rk_dim7_bPerm[63];
b[64] = acadoWorkspace.rk_dim7_bPerm[64];
b[65] = acadoWorkspace.rk_dim7_bPerm[65];
b[66] = acadoWorkspace.rk_dim7_bPerm[66];
b[67] = acadoWorkspace.rk_dim7_bPerm[67];
b[68] = acadoWorkspace.rk_dim7_bPerm[68];
b[69] = acadoWorkspace.rk_dim7_bPerm[69];
}



/** Column vector of size: 1 */
static const real_t Ah_mat[ 1 ] = 
{ 1.2500000000000001e-02 };


/* Fixed step size:0.025 */
int acado_integrate( real_t* const rk_eta, int rk_index )
{
int error;

int i;
int j;
int k;
int k_index;
int run;
int run1;
int shoot_index;
int tmp_index1;
int tmp_index2;
int tmp_index3;

real_t det;

shoot_index = rk_index;
acadoWorkspace.rk_ttt = 0.0000000000000000e+00;
acadoWorkspace.rk_xxx[7] = rk_eta[70];
acadoWorkspace.rk_xxx[8] = rk_eta[71];

acadoWorkspace.rk_delta[0] = rk_eta[0] - acadoVariables.rk_Xprev[shoot_index * 7];
acadoWorkspace.rk_delta[1] = rk_eta[1] - acadoVariables.rk_Xprev[shoot_index * 7 + 1];
acadoWorkspace.rk_delta[2] = rk_eta[2] - acadoVariables.rk_Xprev[shoot_index * 7 + 2];
acadoWorkspace.rk_delta[3] = rk_eta[3] - acadoVariables.rk_Xprev[shoot_index * 7 + 3];
acadoWorkspace.rk_delta[4] = rk_eta[4] - acadoVariables.rk_Xprev[shoot_index * 7 + 4];
acadoWorkspace.rk_delta[5] = rk_eta[5] - acadoVariables.rk_Xprev[shoot_index * 7 + 5];
acadoWorkspace.rk_delta[6] = rk_eta[6] - acadoVariables.rk_Xprev[shoot_index * 7 + 6];
acadoVariables.rk_Xprev[shoot_index * 7] = rk_eta[0];
acadoVariables.rk_Xprev[shoot_index * 7 + 1] = rk_eta[1];
acadoVariables.rk_Xprev[shoot_index * 7 + 2] = rk_eta[2];
acadoVariables.rk_Xprev[shoot_index * 7 + 3] = rk_eta[3];
acadoVariables.rk_Xprev[shoot_index * 7 + 4] = rk_eta[4];
acadoVariables.rk_Xprev[shoot_index * 7 + 5] = rk_eta[5];
acadoVariables.rk_Xprev[shoot_index * 7 + 6] = rk_eta[6];
acadoWorkspace.rk_delta[7] = rk_eta[70] - acadoVariables.rk_Uprev[shoot_index * 2];
acadoWorkspace.rk_delta[8] = rk_eta[71] - acadoVariables.rk_Uprev[shoot_index * 2 + 1];
acadoVariables.rk_Uprev[shoot_index * 2] = rk_eta[70];
acadoVariables.rk_Uprev[shoot_index * 2 + 1] = rk_eta[71];
acadoWorkspace.rk_xxx_lin[0] = rk_eta[0];
acadoWorkspace.rk_xxx_lin[1] = rk_eta[1];
acadoWorkspace.rk_xxx_lin[2] = rk_eta[2];
acadoWorkspace.rk_xxx_lin[3] = rk_eta[3];
acadoWorkspace.rk_xxx_lin[4] = rk_eta[4];
acadoWorkspace.rk_xxx_lin[5] = rk_eta[5];
acadoWorkspace.rk_xxx_lin[6] = rk_eta[6];
for (run = 0; run < 2; ++run)
{
if( run > 0 ) {
for (i = 0; i < 7; ++i)
{
acadoWorkspace.rk_diffsPrev2[i * 9] = rk_eta[i * 7 + 7];
acadoWorkspace.rk_diffsPrev2[i * 9 + 1] = rk_eta[i * 7 + 8];
acadoWorkspace.rk_diffsPrev2[i * 9 + 2] = rk_eta[i * 7 + 9];
acadoWorkspace.rk_diffsPrev2[i * 9 + 3] = rk_eta[i * 7 + 10];
acadoWorkspace.rk_diffsPrev2[i * 9 + 4] = rk_eta[i * 7 + 11];
acadoWorkspace.rk_diffsPrev2[i * 9 + 5] = rk_eta[i * 7 + 12];
acadoWorkspace.rk_diffsPrev2[i * 9 + 6] = rk_eta[i * 7 + 13];
acadoWorkspace.rk_diffsPrev2[i * 9 + 7] = rk_eta[i * 2 + 56];
acadoWorkspace.rk_diffsPrev2[i * 9 + 8] = rk_eta[i * 2 + 57];
}
}
else{
acadoWorkspace.rk_diffsPrev2[0] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[1] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[2] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[3] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[4] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[5] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[6] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[7] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[8] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[9] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[10] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[11] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[12] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[13] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[14] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[15] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[16] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[17] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[18] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[19] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[20] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[21] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[22] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[23] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[24] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[25] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[26] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[27] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[28] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[29] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[30] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[31] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[32] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[33] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[34] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[35] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[36] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[37] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[38] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[39] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[40] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[41] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[42] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[43] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[44] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[45] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[46] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[47] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[48] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[49] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[50] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[51] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[52] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[53] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[54] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[55] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[56] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[57] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[58] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[59] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[60] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[61] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[62] = 0.0000000000000000e+00;
}
k_index = ((shoot_index * 2) + (run)) * (7);
for (i = 0; i < 7; ++i)
{
j = (k_index) + (i);
tmp_index1 = j * 9;
for (run1 = 0; run1 < 1; ++run1)
{
acadoVariables.rk_Ktraj[(j) + (run1)] += + acadoWorkspace.rk_delta[0]*acadoVariables.rk_diffKtraj[(tmp_index1) + (run1)] + acadoWorkspace.rk_delta[1]*acadoVariables.rk_diffKtraj[(tmp_index1 + 1) + (run1)] + acadoWorkspace.rk_delta[2]*acadoVariables.rk_diffKtraj[(tmp_index1 + 2) + (run1)] + acadoWorkspace.rk_delta[3]*acadoVariables.rk_diffKtraj[(tmp_index1 + 3) + (run1)] + acadoWorkspace.rk_delta[4]*acadoVariables.rk_diffKtraj[(tmp_index1 + 4) + (run1)] + acadoWorkspace.rk_delta[5]*acadoVariables.rk_diffKtraj[(tmp_index1 + 5) + (run1)] + acadoWorkspace.rk_delta[6]*acadoVariables.rk_diffKtraj[(tmp_index1 + 6) + (run1)] + acadoWorkspace.rk_delta[7]*acadoVariables.rk_diffKtraj[(tmp_index1 + 7) + (run1)] + acadoWorkspace.rk_delta[8]*acadoVariables.rk_diffKtraj[(tmp_index1 + 8) + (run1)];
}
}
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 7; ++j)
{
acadoWorkspace.rk_stageValues[(0) + ((0) + ((run1 * 7) + (j)))] = acadoWorkspace.rk_xxx_lin[j];
tmp_index1 = (k_index) + (j);
acadoWorkspace.rk_stageValues[(0) + ((0) + ((run1 * 7) + (j)))] += + Ah_mat[run1]*acadoVariables.rk_Ktraj[tmp_index1];
}
}
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 7; ++j)
{
acadoWorkspace.rk_xxx[j] = acadoWorkspace.rk_stageValues[(0) + ((0) + ((run1 * 7) + (j)))];
}
for (j = 0; j < 7; ++j)
{
tmp_index1 = (k_index) + (j);
acadoWorkspace.rk_xxx[j + 9] = acadoVariables.rk_Ktraj[(tmp_index1) + (run1)];
}
acado_acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 112 ]) );
for (j = 0; j < 7; ++j)
{
tmp_index1 = (run1 * 7) + (j);
acadoWorkspace.rk_A[tmp_index1 * 7] = + Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 112) + (j * 16)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 1] = + Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 112) + (j * 16 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 2] = + Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 112) + (j * 16 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 3] = + Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 112) + (j * 16 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 4] = + Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 112) + (j * 16 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 5] = + Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 112) + (j * 16 + 5)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 6] = + Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 112) + (j * 16 + 6)];
if( 0 == run1 ) {
acadoWorkspace.rk_A[tmp_index1 * 7] += acadoWorkspace.rk_diffsTemp2[(run1 * 112) + (j * 16 + 9)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 1] += acadoWorkspace.rk_diffsTemp2[(run1 * 112) + (j * 16 + 10)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 2] += acadoWorkspace.rk_diffsTemp2[(run1 * 112) + (j * 16 + 11)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 3] += acadoWorkspace.rk_diffsTemp2[(run1 * 112) + (j * 16 + 12)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 4] += acadoWorkspace.rk_diffsTemp2[(run1 * 112) + (j * 16 + 13)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 5] += acadoWorkspace.rk_diffsTemp2[(run1 * 112) + (j * 16 + 14)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 6] += acadoWorkspace.rk_diffsTemp2[(run1 * 112) + (j * 16 + 15)];
}
}
acado_acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 70] = - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 70 + 10] = - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 70 + 20] = - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 70 + 30] = - acadoWorkspace.rk_rhsTemp[3];
acadoWorkspace.rk_b[run1 * 70 + 40] = - acadoWorkspace.rk_rhsTemp[4];
acadoWorkspace.rk_b[run1 * 70 + 50] = - acadoWorkspace.rk_rhsTemp[5];
acadoWorkspace.rk_b[run1 * 70 + 60] = - acadoWorkspace.rk_rhsTemp[6];
}
det = acado_solve_dim7_system( acadoWorkspace.rk_A, acadoWorkspace.rk_dim7_perm );
if( run > 0 ) {
acadoWorkspace.rk_Xhat[run * 7] = acadoWorkspace.rk_Xhat[run * 7-7];
acadoWorkspace.rk_Xhat[run * 7 + 1] = acadoWorkspace.rk_Xhat[run * 7-6];
acadoWorkspace.rk_Xhat[run * 7 + 2] = acadoWorkspace.rk_Xhat[run * 7-5];
acadoWorkspace.rk_Xhat[run * 7 + 3] = acadoWorkspace.rk_Xhat[run * 7-4];
acadoWorkspace.rk_Xhat[run * 7 + 4] = acadoWorkspace.rk_Xhat[run * 7-3];
acadoWorkspace.rk_Xhat[run * 7 + 5] = acadoWorkspace.rk_Xhat[run * 7-2];
acadoWorkspace.rk_Xhat[run * 7 + 6] = acadoWorkspace.rk_Xhat[run * 7-1];
acadoWorkspace.rk_Xhat[run * 7] += + acadoWorkspace.rk_Khat[run * 7-7]*(real_t)2.5000000000000001e-02;
acadoWorkspace.rk_Xhat[run * 7 + 1] += + acadoWorkspace.rk_Khat[run * 7-6]*(real_t)2.5000000000000001e-02;
acadoWorkspace.rk_Xhat[run * 7 + 2] += + acadoWorkspace.rk_Khat[run * 7-5]*(real_t)2.5000000000000001e-02;
acadoWorkspace.rk_Xhat[run * 7 + 3] += + acadoWorkspace.rk_Khat[run * 7-4]*(real_t)2.5000000000000001e-02;
acadoWorkspace.rk_Xhat[run * 7 + 4] += + acadoWorkspace.rk_Khat[run * 7-3]*(real_t)2.5000000000000001e-02;
acadoWorkspace.rk_Xhat[run * 7 + 5] += + acadoWorkspace.rk_Khat[run * 7-2]*(real_t)2.5000000000000001e-02;
acadoWorkspace.rk_Xhat[run * 7 + 6] += + acadoWorkspace.rk_Khat[run * 7-1]*(real_t)2.5000000000000001e-02;
for (i = 0; i < 1; ++i)
{
for (j = 0; j < 7; ++j)
{
tmp_index1 = (i * 7) + (j);
acadoWorkspace.rk_b[tmp_index1 * 10] -= + acadoWorkspace.rk_diffsTemp2[(i * 112) + (j * 16)]*acadoWorkspace.rk_Xhat[run * 7] + acadoWorkspace.rk_diffsTemp2[(i * 112) + (j * 16 + 1)]*acadoWorkspace.rk_Xhat[run * 7 + 1] + acadoWorkspace.rk_diffsTemp2[(i * 112) + (j * 16 + 2)]*acadoWorkspace.rk_Xhat[run * 7 + 2] + acadoWorkspace.rk_diffsTemp2[(i * 112) + (j * 16 + 3)]*acadoWorkspace.rk_Xhat[run * 7 + 3] + acadoWorkspace.rk_diffsTemp2[(i * 112) + (j * 16 + 4)]*acadoWorkspace.rk_Xhat[run * 7 + 4] + acadoWorkspace.rk_diffsTemp2[(i * 112) + (j * 16 + 5)]*acadoWorkspace.rk_Xhat[run * 7 + 5] + acadoWorkspace.rk_diffsTemp2[(i * 112) + (j * 16 + 6)]*acadoWorkspace.rk_Xhat[run * 7 + 6];
}
}
}
else{
acadoWorkspace.rk_Xhat[0] = 0.0000000000000000e+00;
acadoWorkspace.rk_Xhat[1] = 0.0000000000000000e+00;
acadoWorkspace.rk_Xhat[2] = 0.0000000000000000e+00;
acadoWorkspace.rk_Xhat[3] = 0.0000000000000000e+00;
acadoWorkspace.rk_Xhat[4] = 0.0000000000000000e+00;
acadoWorkspace.rk_Xhat[5] = 0.0000000000000000e+00;
acadoWorkspace.rk_Xhat[6] = 0.0000000000000000e+00;
}
for (i = 0; i < 1; ++i)
{
for (j = 0; j < 7; ++j)
{
tmp_index1 = (i * 7) + (j);
for (run1 = 0; run1 < 7; ++run1)
{
tmp_index2 = (run1) + (j * 16);
acadoWorkspace.rk_b[(tmp_index1 * 10) + (run1 + 1)] = - acadoWorkspace.rk_diffsTemp2[(i * 112) + (tmp_index2)];
}
for (run1 = 0; run1 < 2; ++run1)
{
tmp_index2 = ((((run1) + (j * 16)) + (0)) + (7)) + (0);
acadoWorkspace.rk_b[(tmp_index1 * 10) + (run1 + 8)] = - acadoWorkspace.rk_diffsTemp2[(i * 112) + (tmp_index2)];
}
}
}
acado_solve_dim7_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim7_perm );
for (i = 0; i < 1; ++i)
{
for (j = 0; j < 7; ++j)
{
tmp_index1 = j * 9;
tmp_index3 = (i * 7) + (j);
for (run1 = 0; run1 < 7; ++run1)
{
tmp_index2 = (tmp_index1) + (run1);
acadoWorkspace.rk_diffKtraj_aux[(tmp_index2) + (i)] = acadoWorkspace.rk_b[(tmp_index3 * 10) + (run1 + 1)];
}
for (run1 = 0; run1 < 2; ++run1)
{
tmp_index2 = (tmp_index1 + 7) + (run1);
acadoWorkspace.rk_diffKtraj_aux[(tmp_index2) + (i)] = acadoWorkspace.rk_b[(tmp_index3 * 10) + (run1 + 8)];
}
}
}
for (i = 0; i < 1; ++i)
{
for (j = 0; j < 7; ++j)
{
tmp_index1 = ((k_index) + (j)) * (9);
tmp_index3 = j * 9;
for (run1 = 0; run1 < 7; ++run1)
{
tmp_index2 = (tmp_index1) + (run1);
acadoVariables.rk_diffKtraj[(tmp_index2) + (i)] = 0.0000000000000000e+00;
acadoVariables.rk_diffKtraj[(tmp_index2) + (i)] += + acadoWorkspace.rk_diffKtraj_aux[(tmp_index3) + (i)]*acadoWorkspace.rk_diffsPrev2[run1];
acadoVariables.rk_diffKtraj[(tmp_index2) + (i)] += + acadoWorkspace.rk_diffKtraj_aux[(tmp_index3 + 1) + (i)]*acadoWorkspace.rk_diffsPrev2[run1 + 9];
acadoVariables.rk_diffKtraj[(tmp_index2) + (i)] += + acadoWorkspace.rk_diffKtraj_aux[(tmp_index3 + 2) + (i)]*acadoWorkspace.rk_diffsPrev2[run1 + 18];
acadoVariables.rk_diffKtraj[(tmp_index2) + (i)] += + acadoWorkspace.rk_diffKtraj_aux[(tmp_index3 + 3) + (i)]*acadoWorkspace.rk_diffsPrev2[run1 + 27];
acadoVariables.rk_diffKtraj[(tmp_index2) + (i)] += + acadoWorkspace.rk_diffKtraj_aux[(tmp_index3 + 4) + (i)]*acadoWorkspace.rk_diffsPrev2[run1 + 36];
acadoVariables.rk_diffKtraj[(tmp_index2) + (i)] += + acadoWorkspace.rk_diffKtraj_aux[(tmp_index3 + 5) + (i)]*acadoWorkspace.rk_diffsPrev2[run1 + 45];
acadoVariables.rk_diffKtraj[(tmp_index2) + (i)] += + acadoWorkspace.rk_diffKtraj_aux[(tmp_index3 + 6) + (i)]*acadoWorkspace.rk_diffsPrev2[run1 + 54];
}
for (run1 = 0; run1 < 2; ++run1)
{
tmp_index2 = (tmp_index1 + 7) + (run1);
tmp_index3 = (j * 9 + 7) + (run1);
acadoVariables.rk_diffKtraj[(tmp_index2) + (i)] = acadoWorkspace.rk_diffKtraj_aux[(tmp_index3) + (i)];
tmp_index3 = j * 9;
acadoVariables.rk_diffKtraj[(tmp_index2) + (i)] += + acadoWorkspace.rk_diffKtraj_aux[(tmp_index3) + (i)]*acadoWorkspace.rk_diffsPrev2[run1 + 7];
acadoVariables.rk_diffKtraj[(tmp_index2) + (i)] += + acadoWorkspace.rk_diffKtraj_aux[(tmp_index3 + 1) + (i)]*acadoWorkspace.rk_diffsPrev2[run1 + 16];
acadoVariables.rk_diffKtraj[(tmp_index2) + (i)] += + acadoWorkspace.rk_diffKtraj_aux[(tmp_index3 + 2) + (i)]*acadoWorkspace.rk_diffsPrev2[run1 + 25];
acadoVariables.rk_diffKtraj[(tmp_index2) + (i)] += + acadoWorkspace.rk_diffKtraj_aux[(tmp_index3 + 3) + (i)]*acadoWorkspace.rk_diffsPrev2[run1 + 34];
acadoVariables.rk_diffKtraj[(tmp_index2) + (i)] += + acadoWorkspace.rk_diffKtraj_aux[(tmp_index3 + 4) + (i)]*acadoWorkspace.rk_diffsPrev2[run1 + 43];
acadoVariables.rk_diffKtraj[(tmp_index2) + (i)] += + acadoWorkspace.rk_diffKtraj_aux[(tmp_index3 + 5) + (i)]*acadoWorkspace.rk_diffsPrev2[run1 + 52];
acadoVariables.rk_diffKtraj[(tmp_index2) + (i)] += + acadoWorkspace.rk_diffKtraj_aux[(tmp_index3 + 6) + (i)]*acadoWorkspace.rk_diffsPrev2[run1 + 61];
}
}
}
for (i = 0; i < 7; ++i)
{
tmp_index1 = ((k_index) + (i)) * (9);
for (run1 = 0; run1 < 9; ++run1)
{
tmp_index2 = (tmp_index1) + (run1);
acadoWorkspace.rk_diffsNew2[(i * 9) + (run1)] = acadoWorkspace.rk_diffsPrev2[(i * 9) + (run1)];
acadoWorkspace.rk_diffsNew2[(i * 9) + (run1)] += + acadoVariables.rk_diffKtraj[tmp_index2]*(real_t)2.5000000000000001e-02;
}
}
acadoWorkspace.rk_xxx_lin[0] += + acadoVariables.rk_Ktraj[k_index]*(real_t)2.5000000000000001e-02;
acadoWorkspace.rk_xxx_lin[1] += + acadoVariables.rk_Ktraj[k_index + 1]*(real_t)2.5000000000000001e-02;
acadoWorkspace.rk_xxx_lin[2] += + acadoVariables.rk_Ktraj[k_index + 2]*(real_t)2.5000000000000001e-02;
acadoWorkspace.rk_xxx_lin[3] += + acadoVariables.rk_Ktraj[k_index + 3]*(real_t)2.5000000000000001e-02;
acadoWorkspace.rk_xxx_lin[4] += + acadoVariables.rk_Ktraj[k_index + 4]*(real_t)2.5000000000000001e-02;
acadoWorkspace.rk_xxx_lin[5] += + acadoVariables.rk_Ktraj[k_index + 5]*(real_t)2.5000000000000001e-02;
acadoWorkspace.rk_xxx_lin[6] += + acadoVariables.rk_Ktraj[k_index + 6]*(real_t)2.5000000000000001e-02;
for (j = 0; j < 1; ++j)
{
acadoVariables.rk_Ktraj[(k_index) + (j)] += acadoWorkspace.rk_b[j * 70];
acadoVariables.rk_Ktraj[(k_index + 1) + (j)] += acadoWorkspace.rk_b[j * 70 + 10];
acadoVariables.rk_Ktraj[(k_index + 2) + (j)] += acadoWorkspace.rk_b[j * 70 + 20];
acadoVariables.rk_Ktraj[(k_index + 3) + (j)] += acadoWorkspace.rk_b[j * 70 + 30];
acadoVariables.rk_Ktraj[(k_index + 4) + (j)] += acadoWorkspace.rk_b[j * 70 + 40];
acadoVariables.rk_Ktraj[(k_index + 5) + (j)] += acadoWorkspace.rk_b[j * 70 + 50];
acadoVariables.rk_Ktraj[(k_index + 6) + (j)] += acadoWorkspace.rk_b[j * 70 + 60];
}
acadoWorkspace.rk_Khat[run * 7] = acadoWorkspace.rk_b[0];
acadoWorkspace.rk_Khat[run * 7 + 1] = acadoWorkspace.rk_b[10];
acadoWorkspace.rk_Khat[run * 7 + 2] = acadoWorkspace.rk_b[20];
acadoWorkspace.rk_Khat[run * 7 + 3] = acadoWorkspace.rk_b[30];
acadoWorkspace.rk_Khat[run * 7 + 4] = acadoWorkspace.rk_b[40];
acadoWorkspace.rk_Khat[run * 7 + 5] = acadoWorkspace.rk_b[50];
acadoWorkspace.rk_Khat[run * 7 + 6] = acadoWorkspace.rk_b[60];
rk_eta[0] += + acadoVariables.rk_Ktraj[k_index]*(real_t)2.5000000000000001e-02;
rk_eta[1] += + acadoVariables.rk_Ktraj[k_index + 1]*(real_t)2.5000000000000001e-02;
rk_eta[2] += + acadoVariables.rk_Ktraj[k_index + 2]*(real_t)2.5000000000000001e-02;
rk_eta[3] += + acadoVariables.rk_Ktraj[k_index + 3]*(real_t)2.5000000000000001e-02;
rk_eta[4] += + acadoVariables.rk_Ktraj[k_index + 4]*(real_t)2.5000000000000001e-02;
rk_eta[5] += + acadoVariables.rk_Ktraj[k_index + 5]*(real_t)2.5000000000000001e-02;
rk_eta[6] += + acadoVariables.rk_Ktraj[k_index + 6]*(real_t)2.5000000000000001e-02;
for (i = 0; i < 7; ++i)
{
for (j = 0; j < 7; ++j)
{
tmp_index2 = (j) + (i * 7);
rk_eta[tmp_index2 + 7] = acadoWorkspace.rk_diffsNew2[(i * 9) + (j)];
}
for (j = 0; j < 2; ++j)
{
tmp_index2 = (j) + (i * 2);
rk_eta[tmp_index2 + 56] = acadoWorkspace.rk_diffsNew2[(i * 9) + (j + 7)];
}
}
acadoWorkspace.rk_ttt += 5.0000000000000000e-01;
}
error = 0;
return error;
}



