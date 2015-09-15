function [betaStar] = optimalImpedance(phi,q,qDot,model)


    e3 = zeros(6,1); e3(3) = 1;
    e4 = zeros(6,1); e4(4) = 1;
    pj_f2_S = zeros(6,1); pj_f2_S(4) = 1;
    
    %1_X_2 = model.
    f2_X_f1M = iDynTree.Transform(iDynTree.Rotation.RotX(q), model.foot.joint_X_frame);
    f2_I1 = f2_X_f1M * model.leg.I;
    
    f2_I2 =  model.foot.I;
    
    I2 = f2_I2.asMatrix().toMatlab();
    I1 = f2_I1.asMatrix().toMatlab();
    f2_X_f1 = f2_X_f1M.asAdjointTransform().toMatlab();
    f1_X_f2 = f2_X_f1M.inverse().asAdjointTransform().toMatlab();
    f2_X_pj_f2 = iDynTree.Transform(iDynTree.Rotation.Identity, model.foot.joint_X_frame);
   
    
    S = f2_X_pj_f2.asAdjointTransform().toMatlab() * pj_f2_S;
    
    X = - [q;qDot]*S';

    Pd = model.foot.length / 2;

   % mTotal = model.foot.mass + model.leg.mass + model.upperbody.mass;
    ag = [rotx(phi*(180/pi))*[0;0;-9.8];[0;0;0]];
    Ibar = I1*S/(S'*I1*S)*S';
    
    alpha_of_q = (crossStar((eye(size(Ibar))...
        - Ibar)*S)*I1 * S * (qDot).^2) -...
        (I2 + I1 *f1_X_f2 - Ibar *I1* f1_X_f2)*ag;

    alpha4 = alpha_of_q(4);
    alpha3 = alpha_of_q(3);

    alphaP = alpha4 - Pd*alpha3;
    betaStar = - ((X*Ibar'*(e4 - Pd*e3)) * alphaP) / (norm(X*Ibar'*(e4 - Pd*e3)).^2);

    
    if(any(betaStar<0))
     h = X*Ibar'*(e4 - Pd*e3) ;
    
         fprintf('Negative impedance solution! Optimising on null space\n');
       % fprintf('old : \n');
        %disp(betaStar);
        
     betaStar = lsqnonneg(h', -alphaP);
    
        %fprintf('new : \n');
        %disp(betaStar);
    end
    
    
    %% verification
    cost = e4'*(Ibar*X'*betaStar + alpha_of_q) - Pd*e3'*(Ibar*X'*betaStar + alpha_of_q);
    fprintf('Cost Function : %2.2f\n',cost);
    
end