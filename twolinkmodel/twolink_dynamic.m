function [w_a, w_f_c] = twolink_dynamic(~, x, u, ~, constraints, model)
% x: state (base position (7), q (1), base velocity (6), qdot (1))
% u: torque at the joint
% fext: external wrench --- not used for now in our simulation
% constraints: either 'foot' or 'left_corner' or 'right_corner' (or empty)
% model: the model

%create the gravity acceleration in the inertial frame
w_gravity = iDynTree.SpatialAcc();
w_gravity.zero();
w_gravity.setVal(2, -9.81); %z direction index 2 in is C (i.e. z direction)

%get the inputs (in the inertial frame) splitting positions and velocitires
x_pos = x(1:8);
assert(length(x_pos) == 8);
xdot = x(9:end);
assert(length(xdot) == 7);
%link 2 (foot) is also the base 
w_v2 = iDynTree.Twist(); %this is w.r.t the inertial frame
w_v2.fromMatlab(xdot(1:6));
qdot = xdot(end);
q = x(8); % joint angle

%Motion constraint matrix
pj_f2_S = zeros(6,1); %written w.r.t origin in joint, orientation of f_2
pj_f2_S(4) = 1; %rotation along x rotational-axis

w_position_f2 = iDynTree.Position(x_pos(1), x_pos(2), x_pos(3));
w_R_f2 = iDynTree.Rotation();
w_R_f2.fromMatlab(rotationFromQuaternion(x_pos(4:7)));

%transformations
w_X_f2 = iDynTree.Transform(w_R_f2, w_position_f2);
%model.foot.joint_X_frame < == > joint = f1, frame = f2
f2_X_f1 = iDynTree.Transform(iDynTree.Rotation.RotX(q), -model.foot.joint_X_frame);
f2_X_pj_f2 = iDynTree.Transform(iDynTree.Rotation.Identity, -model.foot.joint_X_frame);
w_X_f1 = w_X_f2 * f2_X_f1; % for completeness sake but we do not integrate twist of link 1

f2_v2 = w_X_f2.inverse() * w_v2; %now v2 is in the f2 frame
% f2_S = w_X_f2.inverse().asAdjointTransform().toMatlab() * pj_f2_S; %S expressed in 2
f2_S = f2_X_pj_f2.asAdjointTransform().toMatlab() * pj_f2_S; %S expressed in 2, origin in 2

f2_v2Mat = f2_v2.toMatlab();
f2_v2Mat_linear = f2_v2Mat(1:3);
f2_v2Mat_angular = f2_v2Mat(4:end);
f2_Sdot = [skew(f2_v2Mat_angular),  skew(f2_v2Mat_linear);
           zeros(3,3),              skew(f2_v2Mat_angular)] * f2_S;
    
%wrench induced by the actuation
f2_f_act = iDynTree.Wrench();
f2_f_act.fromMatlab(f2_S * u / norm(f2_S));

f2_Sqdot = iDynTree.Twist();
f2_Sqdot.fromMatlab(f2_S * qdot);
f2_v1 = f2_v2 + f2_Sqdot; %v1 is in the f2 frame

%external force in the v2 frame
% w_X_f2.inverse() * fext; %to be modified

%define inertias (in the v2 frame)
f2_I1 = f2_X_f1 * model.leg.I;
f2_I2 = model.foot.I;
f2_I1mat = f2_I1.asMatrix().toMatlab();
f2_I2mat = f2_I2.asMatrix().toMatlab();

%compute biases
f2_v1xI1v1 = iDynTree.Wrench(f2_v1.cross(f2_I1.multiply(f2_v1))); %expressed in 2
f2_v2xI2v2 = iDynTree.Wrench(f2_v2.cross(f2_I2.multiply(f2_v2))); %expressed in 2
f2_b1 = f2_v1xI1v1 - (f2_I1 * (w_X_f2.inverse() * w_gravity)) - f2_f_act; %expressed in 2
f2_b2 = f2_v2xI2v2 - (f2_I2 * (w_X_f2.inverse() * w_gravity)) + f2_f_act; %expressed in 2

f2_ApparentIInv = f2_S / (f2_S' * f2_I1mat * f2_S) * f2_S'; %the constrainted appartent inverse inertia
f2_d = -f2_I1mat * f2_ApparentIInv * f2_I1mat * f2_Sdot * qdot ...
       -f2_I1mat * f2_ApparentIInv * f2_b1.toMatlab() + f2_I1mat * f2_Sdot * qdot + f2_b1.toMatlab();

%%Constraints
f2_T = [];
constraintsSize = 0;
if (~isempty(constraints))
    if (strcmpi(constraints, 'left_corner'))
        w_J = w_X_f2.asAdjointTransform().toMatlab();
        w_T = w_J';% [eye(3); zeros(3)];
        f2_T = w_X_f2.inverse().asAdjointTransform().toMatlab() * w_T;
        f2_T = f2_T(:,1:3);
        constraintsSize = 3;
    elseif (strcmpi(constraints, 'right_corner'))
        leftToRightTranslation_f2 = iDynTree.Position(0, model.foot.length, 0);
        identity = iDynTree.Rotation.Identity();
        tranlation_f2 = iDynTree.Transform(identity, leftToRightTranslation_f2);
        tranlation_f2 = w_X_f2 * tranlation_f2;
        w_J = tranlation_f2.asAdjointTransform().toMatlab();
        w_T = w_J';% [eye(3); zeros(3)];
        f2_T = w_X_f2.inverse().asAdjointTransform().toMatlab() * w_T;
        f2_T = f2_T(:,1:3);
        constraintsSize = 3;
    elseif (strcmpi(constraints, 'foot'))
%         w_J = [];
        w_T = eye(6);
        f2_T = w_X_f2.inverse().asAdjointTransform().toMatlab() * w_T;
        constraintsSize = 6;
    end
%     f2_J = w_X_f2.inverse().asAdjointTransform().toMatlab() * w_J;
    
end

%compute acceleration of base frame (and foot)
% f2_a2 = (f2_I1mat + f2_I2mat - f2_I1mat * f2_ApparentIInv * f2_I1mat) \ (f2_fc.toMatlab() - f2_b2.toMatlab() - f2_d);

%write system as A x = b
% f2_A = [(f2_I1mat + f2_I2mat - f2_I1mat * f2_ApparentIInv * f2_I1mat), - f2_T;
%         zeros(size(f2_J, 1)), f2_J  / (f2_I1mat + f2_I2mat - f2_I1mat * f2_ApparentIInv * f2_I1mat) * f2_T];
% f2_b = [- f2_b2.toMatlab() - f2_d; ];
f2_A = [(f2_I1mat + f2_I2mat - f2_I1mat * f2_ApparentIInv * f2_I1mat), f2_T;
          f2_T', zeros(constraintsSize)];
f2_b = [-f2_b2.toMatlab() - f2_d; zeros(constraintsSize, 1)];

f2_sol = f2_A \ f2_b;
f2_a2 = f2_sol(1:6);
w_f_c = [];
if (constraintsSize > 0)
    %output the constraint foce
    w_f_c = w_X_f2.asAdjointTransform().toMatlab() * f2_T * (-f2_sol(7:end));
end

%%Sanity checks
if (constraintsSize > 0)
    assert(any(abs(f2_T' * f2_v2Mat) < 1e-15),'Contraint equation: T^t v = 0');
    assert(any(abs(f2_T' * f2_a2) < 1e-15),'Acceleration contraint equation: T^t a = 0');
end

%compute acceleration of the joint
qddot = -(f2_S'*f2_I1mat*f2_S) \ (f2_S' * f2_I1mat * f2_a2 + f2_S' * f2_I1mat * f2_Sdot * qdot + f2_S' * f2_b1.toMatlab());
%acceleration of the leg is not used in the integration. Yet it can be
%computed by knowing the acceleration of the base the that of the joint
% a1 = a2 + S*qddot + Sdot * qdot;

%now transform the output back into inertial frame
f2_a2Acc = iDynTree.SpatialAcc();
% following line is for a single rigid body
% a2Acc.fromMatlab(-inv(I2mat) * b2.toMatlab());
f2_a2Acc.fromMatlab(f2_a2);
w_a2 = w_X_f2 * f2_a2Acc;

% following line is for a single rigid body
% a = [a2_inertial.toMatlab(); 0];

w_a = [w_a2.toMatlab(); qddot];

F = [f2_b1.toMatlab(); f2_b2.toMatlab()];

end
