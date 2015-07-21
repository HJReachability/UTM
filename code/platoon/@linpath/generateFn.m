function fn = generateFn(obj)
% function fn = generateFn(z0, z1)
%
% Computes the function handle that describes a linear path
%
% Inputs:  z0, z1 - starting and ending points
%
% Output:  fn     - function handle for the highway
%             fn(s) specifies a point on the highway
%                fn(0) = z0, fn(1) = z1
%                fn(s), 0<s<1 picks a point between z0 and z1, with
%                             linear spacing
%
% Mo Chen, 2015-05-25
fn = @(s) [(1-s)*obj.z0(1) + s*obj.z1(1); (1-s)*obj.z0(2) + ...
  s*obj.z1(2)];
end