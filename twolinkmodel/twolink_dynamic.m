function [a, F] = twolink_dynamic(~, x, u, fext, model)
% x: state (base position (7), q (1), base velocity (6), qdot (1))
% u: torque at the joint
% fext: external forces
% model: the model

%create the gravity acceleration in the inertial frame
gravity = iDynTree.SpatialAcc();
gravity.zero();
gravity.setVal(2, -9.81); %z direction

%get the inputs (in the inertial frame)
x_pos = x(1:8);
assert(length(x_pos) == 8);
xdot = x(9:end);
assert(length(xdot) == 7);
%link 2 (foot) is also the base 
v2 = iDynTree.Twist(); %this is w.r.t the inertial frame
v2.fromMatlab(xdot(1:6));
qdot = xdot(end);

%Motion constraint matrix
S = zeros(6,1);
S(4) = 1; %rotation along x rotational-axis

% v2Mat = v2.toMatlab();
% Sdot = [skew(v2Mat(4:end)), skew(v2Mat(1:3));
%         zeros(3,3), skew(v2Mat(4:end))] * S;

f2_position = iDynTree.Position(x_pos(1), x_pos(2), x_pos(3));
f2_rotation = iDynTree.Rotation();
f2_rotation.fromMatlab(rotationFromQuaternion(x_pos(4:7)));

%transformations
inertial_X_f2 = iDynTree.Transform(f2_rotation, f2_position);
%model.foot.joint_X_frame < == > joint = f1, frame = f2
f2_X_f1 = iDynTree.Transform(iDynTree.Rotation.RotX(x(8)), -model.foot.joint_X_frame);
inertial_X_f1 = inertial_X_f2 * f2_X_f1;

v2 = inertial_X_f2.inverse() * v2; %now v2 is in the f2 frame
S = inertial_X_f2.inverse().asAdjointTransform().toMatlab() * S; %S expressed in 2

v2Mat = v2.toMatlab();
Sdot = [skew(v2Mat(4:end)), skew(v2Mat(1:3));
        zeros(3,3), skew(v2Mat(4:end))] * S;
% Sdot = inertial_X_f2.inverse().asAdjointTransform().toMatlab() * Sdot;
    
%wrench induced by the actuation
f_act = iDynTree.Wrench();
f_act.fromMatlab(S * u / norm(S));


Sqdot = iDynTree.Twist();
Sqdot.fromMatlab(S * qdot);
v1 = v2 + Sqdot; %v1 is in the f2 frame

%external force in the v2 frame
fc = inertial_X_f2.inverse() * fext;

%define inertias (in the v2 frame)
I1 = f2_X_f1 * model.leg.I;
I2 = model.foot.I;
I1mat = I1.asMatrix().toMatlab();
I2mat = I2.asMatrix().toMatlab();

%compute biases
v1xI1v1 = iDynTree.Wrench(v1.cross(I1.multiply(v1))); %expressed in 2
v2xI2v2 = iDynTree.Wrench(v2.cross(I2.multiply(v2))); %expressed in 2
b1 = v1xI1v1 - (I1 * (inertial_X_f2.inverse() * gravity)) - f_act; %expressed in 2
b2 = v2xI2v2 - (I2 * (inertial_X_f2.inverse() * gravity)) + f_act; %expressed in 2

SI = S / (S' * I1mat * S);
Q1 = I1mat * SI * S' * I1mat;
d = -Q1 * Sdot * qdot - I1mat * SI * S' * b1.toMatlab() + I1mat * Sdot * qdot + b1.toMatlab();

%compute acceleration of base frame (and foot)
a2 = (I1mat + I2mat - Q1) \ (fc.toMatlab() - b2.toMatlab() - d);
%compute acceleration of the joint
qddot = -inv(S'*I1mat*S) * (S' * I1mat * a2 + S' * I1mat * Sdot * qdot + S' * b1.toMatlab());
%acceleration of the leg is not used in the integration. Yet it can be
%computed by knowing the acceleration of the base the that of the joint
% a1 = a2 + S*qddot + Sdot * qdot;

%now transform the output back into inertial frame
a2Acc = iDynTree.SpatialAcc();
% following line is for a single rigid body
% a2Acc.fromMatlab(-inv(I2mat) * b2.toMatlab());
a2Acc.fromMatlab(a2);
a2_inertial = inertial_X_f2 * a2Acc;

% following line is for a single rigid body
% a = [a2_inertial.toMatlab(); 0];

a = [a2_inertial.toMatlab(); qddot];

F = [b1.toMatlab(); b2.toMatlab()];

end