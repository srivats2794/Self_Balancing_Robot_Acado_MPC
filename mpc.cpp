/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 0){ 
      mexErrMsgTxt("This problem expects 0 right hand side argument(s) since you have defined 0 MexInput(s)");
    } 
 
    TIME autotime;
    DifferentialState x;
    DifferentialState y;
    DifferentialState Psi;
    DifferentialState v_l;
    DifferentialState v_r;
    DifferentialState theta;
    DifferentialState thetaDot;
    Control tau_l;
    Control tau_r;
    BMatrix acadodata_M1;
    acadodata_M1.read( "mpc_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "mpc_data_acadodata_M2.txt" );
    Function acadodata_f1;
    acadodata_f1 << x;
    acadodata_f1 << y;
    acadodata_f1 << Psi;
    acadodata_f1 << v_l;
    acadodata_f1 << v_r;
    acadodata_f1 << theta;
    acadodata_f1 << thetaDot;
    acadodata_f1 << tau_l;
    acadodata_f1 << tau_r;
    Function acadodata_f2;
    acadodata_f2 << x;
    acadodata_f2 << y;
    acadodata_f2 << Psi;
    acadodata_f2 << v_l;
    acadodata_f2 << v_r;
    acadodata_f2 << theta;
    acadodata_f2 << thetaDot;
    OCP ocp1(0, 5, 100);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f1);
    ocp1.minimizeLSQEndTerm(acadodata_M2, acadodata_f2);
    ocp1.subjectTo(2.50000000000000000000e-01 <= x <= 2.75000000000000000000e+00);
    ocp1.subjectTo(0.00000000000000000000e+00 <= y <= 4.25000000000000000000e+00);
    ocp1.subjectTo((-4.50000000000000000000e+00) <= v_l <= 4.50000000000000000000e+00);
    ocp1.subjectTo((-4.50000000000000000000e+00) <= v_r <= 4.50000000000000000000e+00);
    ocp1.subjectTo((-1.00000000000000005551e-01) <= theta <= 1.00000000000000005551e-01);
    ocp1.subjectTo((-1.00000000000000005551e-01) <= thetaDot <= 1.00000000000000005551e-01);
    ocp1.subjectTo((2.99999999999999988898e-01+2.99999999999999988898e-01-sqrt((pow((-1.14999999999999991118e+00+x),2.00000000000000000000e+00)+pow((-2.20000000000000017764e+00+y),2.00000000000000000000e+00)))) <= 0.00000000000000000000e+00);
    ocp1.subjectTo((2.99999999999999988898e-01+2.99999999999999988898e-01-sqrt((pow((-1.05000000000000004441e+00+x),2.00000000000000000000e+00)+pow((-3.20000000000000017764e+00+y),2.00000000000000000000e+00)))) <= 0.00000000000000000000e+00);
    DifferentialEquation acadodata_f3;
    acadodata_f3 << (-(v_l+v_r)/2.00000000000000000000e+00*cos(Psi)+dot(x)) == 0.00000000000000000000e+00;
    acadodata_f3 << dot(y) == (v_l+v_r)/2.00000000000000000000e+00*sin(Psi);
    acadodata_f3 << dot(Psi) == (v_l-v_r)/4.40000000000000002220e-01;
    acadodata_f3 << (-(-1.04738327677865084020e+00)*theta-(-4.54889351411010611770e-01)*tau_r-1.12110345893583795984e+00*tau_l+dot(v_l)) == 0.00000000000000000000e+00;
    acadodata_f3 << (-(-1.04738327677865084020e+00)*theta-(-4.54889351411010611770e-01)*tau_l-1.12110345893583795984e+00*tau_r+dot(v_r)) == 0.00000000000000000000e+00;
    acadodata_f3 << (dot(theta)-theta) == 0.00000000000000000000e+00;
    acadodata_f3 << (-(-5.03843431775340300227e-01)*tau_l-(-5.03843431775340300227e-01)*tau_r-1.64157566686435067993e+01*theta+dot(thetaDot)) == 0.00000000000000000000e+00;

    ocp1.setModel( acadodata_f3 );


    ocp1.setNU( 2 );
    ocp1.setNP( 0 );
    ocp1.setNOD( 0 );
    OCPexport ExportModule1( ocp1 );
    ExportModule1.set( GENERATE_MATLAB_INTERFACE, 1 );
    uint options_flag;
    options_flag = ExportModule1.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HESSIAN_APPROXIMATION");
    options_flag = ExportModule1.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: DISCRETIZATION_TYPE");
    options_flag = ExportModule1.set( SPARSE_QP_SOLUTION, FULL_CONDENSING_N2 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: SPARSE_QP_SOLUTION");
    options_flag = ExportModule1.set( IMPLICIT_INTEGRATOR_MODE, LIFTED );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: IMPLICIT_INTEGRATOR_MODE");
    options_flag = ExportModule1.set( INTEGRATOR_TYPE, INT_IRK_GL2 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: INTEGRATOR_TYPE");
    options_flag = ExportModule1.set( NUM_INTEGRATOR_STEPS, 200 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: NUM_INTEGRATOR_STEPS");
    options_flag = ExportModule1.set( QP_SOLVER, QP_QPOASES3 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: QP_SOLVER");
    options_flag = ExportModule1.set( GENERATE_SIMULINK_INTERFACE, YES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: GENERATE_SIMULINK_INTERFACE");
    options_flag = ExportModule1.set( LEVENBERG_MARQUARDT, 1.000000E-04 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: LEVENBERG_MARQUARDT");
    options_flag = ExportModule1.set( HOTSTART_QP, YES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HOTSTART_QP");
    options_flag = ExportModule1.set( MAX_NUM_QP_ITERATIONS, 100 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: MAX_NUM_QP_ITERATIONS");
    uint export_flag;
    export_flag = ExportModule1.exportCode( "export_MPC" );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");


    clearAllStaticCounters( ); 
 
} 

