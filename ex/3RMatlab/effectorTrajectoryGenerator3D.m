function [ qchd, qchd_d1, qchd_d2 ] = effectorTrajectoryGenerator3D( t, parameters)
%effectorTrajectoryGenerator
%generates prime function, first and second derivative for
%end effector trajectory

    w = 0.015 * 2 * pi;
    k = 0.5;
    dx = 2.3;
    dy = 0.1;
    dz = 0.0;
    
    qchd = [k*cos(w*t)+dx; k*sin(w*t)+dy; 0.1*k*cos(w*t)+dz];
    
    %caclulate first and second derivative based on the qchd vector
    qchd_d1 = [ ...
        -k*w*sin(w*t);
        k*w*cos(w*t);
        -0.1*k*w*sin(w*t)
    ];

    % second derivatives
    qchd_d2 = [ ...
        -k*w^2*cos(w*t);
        -k*w^2*sin(w*t);
        -0.1*k*w^2*cos(w*t)
    ];
    
end

