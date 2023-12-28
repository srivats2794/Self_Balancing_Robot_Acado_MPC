   
    DifferentialState x y Psi v_l v_r theta thetaDot;
    Control tau_l tau_r;

    % % UNCOMMENT IF YOU WANT TO INTRODUCE DYNAMIC OBSTACLES
    % if (sim.obs_num>0)
    %     for i=1:sim.obs_num
    %         temp_var1 = strcat( 'x_obs',num2str(i) );
    %         temp_var2 = strcat( 'y_obs',num2str(i) );
    %         eval(sprintf('%s %s',"OnlineData",temp_var1));
    %         eval(sprintf('%s %s',"OnlineData",temp_var2));
    %     end
    %     clearvars temp_var2 temp_var1 i
    % end
    
    X_dot= pl.lin_sys.A*[v_l;v_r;theta;thetaDot]+pl.lin_sys.B*[tau_l;tau_r];

    %% Differential Equation
    f= [dot(x)-((v_l+v_r)/2)*cos(Psi) == 0; ...
        dot(y)== ((v_l+v_r)/2)*sin(Psi); ...
        dot(Psi)== ((v_l-v_r)/sys.w); ...
        dot(v_l)-X_dot(1)==0; ...
        dot(v_r)-X_dot(2)==0; ...
        dot(theta) - X_dot(3) == 0; ...
        dot(thetaDot) - X_dot(4) == 0;];

    h = [diffStates; controls];
    hN = diffStates;
    clearvars X_dot
    %% MPCexport
    acadoSet('problemname', 'mpc');
   
    ocp = acado.OCP( 0.0, pl.N*pl.Ts, pl.N );
    
    W = acado.BMatrix(eye(length(h)));
    WN= acado.BMatrix(eye(length(hN)));
    ocp.minimizeLSQ( W, h );
    ocp.minimizeLSQEndTerm( WN, hN );
    ocp.subjectTo( sim.x_min <= x <= sim.x_max );
    ocp.subjectTo( sim.y_min <= y <= sim.y_max );
    ocp.subjectTo( pl.v_min <= v_l <= pl.v_max );
    ocp.subjectTo( pl.v_min <= v_r <= pl.v_max );
    ocp.subjectTo( -0.1 <= theta <= 0.1 );
    ocp.subjectTo( -0.1 <= thetaDot <= 0.1);

    for i=1:sim.obs_num
        ocp.subjectTo(-sqrt((x-sim.obs_x(i))^2  + ...
                           (y-sim.obs_y(i))^2) + ...
                            pl.ego_safety_radius/2 + sim.obs_diam/2 <= 0);
    end
    clearvars i;
    % % IF YOU WANT DYNAMIC OBSTACLES
    % for i=1:sim.obs_num
    %      temp_var1 = strcat( 'x_obs',num2str(i) );
    %      temp_var2 = strcat( 'y_obs',num2str(i) );   
    %      ocp.subjectTo(-sqrt((x-eval(sprintf('%s',temp_var1)))^2  + ...
    %                          (y-eval(sprintf('%s',temp_varq)))^2) + ...
    %                     pl.ego_safety_radius/2 + sim.obs_diam/2 <= 0);
    % end
    ocp.setModel(f);
    mpc = acado.OCPexport( ocp );
    mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
    mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
    mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
    mpc.set( 'IMPLICIT_INTEGRATOR_MODE', 	'LIFTED' 			);
    mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL2'       );
    mpc.set( 'NUM_INTEGRATOR_STEPS',         2*pl.N             );
    mpc.set( 'QP_SOLVER',                   'QP_QPOASES3'    	);
    mpc.set( 'GENERATE_SIMULINK_INTERFACE', 'YES'               );
    mpc.set( 'LEVENBERG_MARQUARDT',          1e-4                );
    mpc.set( 'HOTSTART_QP',                 'YES'             	);
    mpc.set( 'MAX_NUM_QP_ITERATIONS',        100                 );


    if EXPORT
        mpc.exportCode( 'export_MPC' );
    end
    if COMPILE
        global ACADO_;
        copyfile([ACADO_.pwd '/../../external_packages/qpoases3'], 'export_MPC/qpoases3')
    
        cd export_MPC
        make_acado_solver_sfunction
        copyfile('acado_solver_sfun.mex*', '../')
        cd ..
    end
        
    save sim_data.mat ACADOdata ACADOinput ACADOoutput