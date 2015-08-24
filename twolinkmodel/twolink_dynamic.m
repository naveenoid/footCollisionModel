
function a = twolink_dynamic(~, x, fext, model)

gravity = iDynTree.SpatialAcc();
gravity.zero();
gravity.setVal(2, -9.81);

%input 
x_pos = x(1:8);
xdot = x(9:end);
%v2 = base
v2 = iDynTree.Twist();
v2.fromMatlab(xdot(1:6));
qdot = xdot(end);

%Constraint matrix
S = zeros(6,1);
S(3) = 1;
v2Mat = v2.toMatlab();
Sdot = [skew(v2Mat(1:3)), skew(v2Mat(4:end));
        skew(v2Mat(4:end)), zeros(3,3)] * S;

f2_position = iDynTree.Position(x_pos(1), x_pos(2), x_pos(3));
f2_rotation = iDynTree.Rotation();
f2_rotation.fromMatlab(rotationFromQuaternion(x_pos(4:7)));

%transformations
f2_X_inertial = iDynTree.Transform(f2_rotation, f2_position);
%model.foot.joint_X_frame < == > joint = f1, frame = f2
f2_X_f1 = iDynTree.Transform(iDynTree.Rotation.RotX(x(8)), -model.foot.joint_X_frame);
f1_X_inertial = f2_X_f1.inverse() * f2_X_inertial;

Sqdot = iDynTree.Twist();
Sqdot.fromMatlab(S * qdot);
v1 = v2 + Sqdot;

fc = f2_X_inertial * fext;

%define
I1 = f2_X_inertial * model.leg.I;
I2 = model.foot.I;
I1mat = I1.asMatrix().toMatlab();
I2mat = I2.asMatrix().toMatlab();

%compute biases
v1xI1v1 = iDynTree.Wrench(v1.cross(I1.multiply(v1)));
v2xI2v2 = iDynTree.Wrench(v2.cross(I2.multiply(v2)));
b1 = v1xI1v1 - (I1 * (f1_X_inertial * gravity));
b2 = v2xI2v2 - (I1 * (f2_X_inertial * gravity));

SI = S / (S' * I1mat * S);
Q1 = I1mat * SI * S' * I1mat;
q1 = -Q1 * Sdot * qdot  - I1mat * SI * S' * b1.toMatlab() + I1mat * Sdot * qdot + b1.toMatlab();



a2 = (I1mat + I2mat - Q1) \ (fc.toMatlab() - b2.toMatlab() - q1);
qddot = -inv(S'*I1mat*S) * (S' * I1mat * a2 + S' * I1mat * Sdot * qdot + S' * b1.toMatlab());
% a1 = a2 + S*qddot + Sdot * qdot;

a = [a2; qddot];

end