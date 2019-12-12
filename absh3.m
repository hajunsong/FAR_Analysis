function [y, t_next ] = absh3( t_current, y, yp, stepsize_h )
% ABSH3  : constant step Adams Bashforth 3rd order formulation.

% written by Sung-Soo Kim
% Date: Oct. 19, 1998
% copyright reserved by Sung-Soo Kim

% input variables
% t_current: current time
% y : current state
% yp : current derivative of state
% stepsize_h: integration stepsize

% output variables
% t_next: next time 
% y : state at next time step

% STARTER:  upto 2h, i.e., derivatives are stored for the initial  time steps at 0, h, 2h, to form
%                  3rd order  Adams Bashforth formula

global  intcount  AT AT1
switch intcount
case 1
   % Forward Euler method with 0.25 stepsize_h for initial step
   % use derivative information at 0 step
   y = y + stepsize_h*yp/4.0;
   AT(:,2) = yp;
   AT1(:,2) = yp;
   intcount = intcount + 1;
   t_next = t_current + stepsize_h/4.0;
   
case 2
   % Adams Bashforth 2nd order method with 0.25 stepsize_h for 2nd step
   % use derivative inforamtion at 0, h/4
   y = y + stepsize_h * ( 3.0*yp - AT1(:,2))/8.0;
   AT1(:,1) = yp;
   intcount = intcount + 1;
   t_next = t_current + stepsize_h/4.0;
   
case 3
   % Adams Bashforth 3rd order method with 0.25 stepsize_h for 3rd step
   % use derivative information at 0, h/4, h/2
   y = y + stepsize_h * ( 23.0*yp - 16.0*AT1(:,1) + 5.0*AT1(:,2))/48.0;
   AT1(:,2) = AT1(:,1);
   AT1(:,1) = yp;
   intcount = intcount + 1;
   t_next = t_current + stepsize_h/4.0;
   
case 4
   % Adams Bashforth 3rd order method with 0.25 stepsize_h for 4th step
   % use derivative information at h/4, h/2, 3h/4
   y = y + stepsize_h * ( 23.0*yp - 16.0*AT1(:,1) + 5.0*AT1(:,2))/48.0;
   AT1(:,2) = AT(:,2);
   intcount = intcount + 1;
   t_next = t_current + stepsize_h/4.0;
   
case 5
   % Adams Bashforth 3rd order method with 0.5 stepsize_h for 5th step
   % use derivative information at 0, h/2, h
   y = y + stepsize_h * ( 23.0*yp - 16.0*AT1(:,1) + 5.0*AT1(:,2))/24.0;
   AT(:,1) = yp;
   AT1(:,2) = AT1(:,1);
   AT1(:,1) = yp;
   intcount = intcount + 1;
   t_next = t_current + stepsize_h/2.0;
   
case 6
   % Adams Bashforth 3rd order method with 0.5 stepsize_h for 6th step
   % use derivative information at h/2, h,  3h/2
   y = y + stepsize_h * ( 23.0*yp - 16.0*AT1(:,1) + 5.0*AT1(:,2))/24.0;
   AT1(:,2) = AT1(:,1);
   AT1(:,1) = yp;
   intcount = intcount + 1;
   t_next = t_current + stepsize_h/2.0;
   
case 7
   % Adams Bashforth 3rd order method with stepsize_h for 7th step
   % use derivative information at 0,  h,  2h
   y = y + stepsize_h * ( 23.0*yp - 16.0*AT(:,1) + 5.0*AT(:,2))/12.0;
   AT(:,2) = AT(:,1);
   AT(:,1) = yp;
   intcount = intcount + 1;
   t_next = t_current + stepsize_h;
   
otherwise
   % Adams Bashforth 3rd order method with stepsize_h for more than 8th step
   % use derivative information t_current-2h, t_current-h, t_current
   y = y + stepsize_h * ( 23.0*yp - 16.0*AT(:,1) + 5.0*AT(:,2))/12.0;
   AT(:,2) = AT(:,1);
   AT(:,1) = yp;
   intcount = intcount + 1;
   t_next = t_current + stepsize_h;
end