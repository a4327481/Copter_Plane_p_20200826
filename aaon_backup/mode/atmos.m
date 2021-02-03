function [mach,qbar,ps]=atmos( alt,  vt )
    rho0 = 2.377e-3;
    tfac =1 - .703e-5*(alt);
    temp = 519.0*tfac;
    if (alt >= 35000.0) 
       temp=390;
    end

    rho=rho0*pow(tfac,4.14);
    mach = (vt)/sqrt(1.4*1716.3*temp);
    qbar = .5*rho*pow(vt,2);
    ps   = 1715.0*rho*temp;

    if (ps == 0) 
      ps = 1715;
    end
end