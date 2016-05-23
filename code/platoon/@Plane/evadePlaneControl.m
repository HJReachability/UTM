function [u_this, u_other, sval] = evadePlaneControl(obj, other, PE_BRS)
% [u_this, u_other, safety_val] = evadePlaneControl(obj, other, PE_BRS)
%   Computes the control for this Plane to evade the other Plane
%
% Inputs:
%   obj   -    this Plane object
%   other -  other Plane object
%   sval  - pursuit-evasion backward reachable set
%
% Outputs:
%   u_this     - optimal control for this Plane to avoid the other
%   u_other    - optimal control for the other Plane to cause a collision
%   safety_val - safety value of this Plane with respect to the other
%
% Mo Chen, 2016-05-22

% Safety value
xr = obj.getRelStates(other, 'pi');
sval = eval_u(PE_BRS.g, PE_BRS.data, xr);

% Gradient and determinants
p = eval_u(PE_BRS.g, PE_BRS.grad, xr);

det_v1 = -p(1);
det_v2 = p(1)*cos(xr(3)) + p(2)*sin(xr(3));
det_w1 = p(1)*xr(2) - p(2)*xr(1) - p(3);
det_w2 = p(3);

% Control
v1 = (det_v1>=0)*obj.vrange(2)   + (det_v1<0)*obj.vrange(1);
v2 = (det_v2>=0)*other.vrange(1) + (det_v2<0)*other.vrange(2);
w1 = (det_w1>=0)*obj.wMax        + (det_w1<0)*-obj.wMax;
w2 = (det_w2>=0)*-other.wMax     + (det_w2<0)*other.wMax;

u_this  = [v1; w1];
u_other = [v2; w2];
end