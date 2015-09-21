clear
close all;

plotFigBaseFolder = './plots/icra_final/';    
if(~exist(plotFigBaseFolder))
    mkdir(plotFigBaseFolder);
end


plotFigBaseName = strcat(plotFigBaseFolder,'damping_');

load('./data/multiTestResult_state.mat');

    model = struct();
    %note: numbers are random for now :D
    %foot: box
    model.foot.mass = 0.4; %kg
    model.foot.length = 0.2; %m
    model.foot.height = 0.05; %m
    %foot reference frame: bottom-left angle
    rotI = iDynTree.RotationalInertiaRaw();
    rotI.zero();
    rotI.setVal(1,1,1); rotI.setVal(2, 2,1);
    rotI.setVal(0, 0, model.foot.mass/3 * (model.foot.length^2 + model.foot.height^2));
    pos = iDynTree.Position(0, ...
        model.foot.length / 2, model.foot.height / 2);
    model.foot.I = iDynTree.SpatialInertia(model.foot.mass, pos, rotI);
    model.foot.joint_X_frame = iDynTree.Position(0, model.foot.length / 3, model.foot.height);

    %leg: rod with uniform mass + an additional mass attached to one end.
    %reference frame at the joint (one end of the rod)
    model.leg.length = 0.4;
    model.leg.mass = 10;
    model.upperbody.mass = 20;
    rotI.zero();
    rotI.setVal(1,1,0.1); rotI.setVal(2, 2,0.1);
    rotI.setVal(0, 0, model.leg.mass/3 * model.leg.length^2);
    legI = iDynTree.SpatialInertia(model.leg.mass, iDynTree.Position(0, 0, model.leg.length / 2), rotI);
    rotI.zero();
    rotI.setVal(1,1,0.1); rotI.setVal(2, 2,0.1);
    rotI.setVal(0,0, model.upperbody.mass * model.leg.length^2);
    upperBodyI = iDynTree.SpatialInertia(model.upperbody.mass, iDynTree.Position(0, 0, model.leg.length), rotI);
    model.leg.I = legI + upperBodyI;


Jtotal = (resultStore.CoPResult - model.foot.length/2*ones(size(resultStore.CoPResult))).^2;

%qCols = autumn(length(ranges.qRange));
%qDCols = winter(length(ranges.qDRange));
cols1 =  autumn(length(ranges.plane_inclinationRange));
cols2 =  winter(length(ranges.plane_inclinationRange));

stiffnessRange = zeros(length(ranges.plane_inclinationRange),length(ranges.qRange),length(ranges.qDRange));
dampingRange = zeros(length(ranges.plane_inclinationRange),length(ranges.qRange),length(ranges.qDRange));

status = zeros(length(ranges.plane_inclinationRange),length(ranges.qRange),length(ranges.qDRange));
 for inclCtr =  1:length(ranges.plane_inclinationRange)
     for qCtr = 1:length(ranges.qRange)
            for qDCtr =1:length(ranges.qDRange)
                
                inclination = ranges.plane_inclinationRange(inclCtr);
                if(inclination == 0)
                    inclination = inclination+0.01;
                end
                q = ranges.qRange(qCtr);
                qD = ranges.qDRange(qDCtr);
                %fprintf('\n----Next run, phi: %2.2f, q(tf): %2.2f, dq/dt(tf) :%2.2f----\n',inclination, q, qD);
                [betaStar] = optimalImpedance(inclination,q,qD,model,'silentMode');
                if(betaStar(1) ~= betaStar(2))
                    status(inclCtr,qCtr,qDCtr) = 1;
                end
                stiffness(inclCtr,qCtr,qDCtr) = betaStar(1); 
                damping(inclCtr,qCtr,qDCtr) = betaStar(2);
               % fprintf('Optimal Stiffness : %2.2f,  Damping %2.2f \n',stiffness,damping);
               % [multiTestResult(inclCtr,qCtr,qDCtr),CoPResult(inclCtr,qCtr,qDCtr),timeToCoPResult(inclCtr,qCtr,qDCtr)] = multiTest(inclination,stiffness,damping,model,rotI);
               
            end
     end
       % fprintf('inclination : %2.2f, status : %d\n',inclination,sum(sum(status(inclCtr,:,:))));
       % disp(squeeze(status(inclCtr,:,:)));
 end

 qdL = length(ranges.qDRange);

 
fH = figure;%('units','normalized','outerposition',[0 0 1 1]);
% axes1 = axes('Parent',fH,'FontSize',16);
% box(axes1,'on');
% hold(axes1,'all');
%chosen_inclCtr1 = [2, 6];
qChosenCtr1 = 2;
qDChosenCtr1 = 1;

qChosenCtr2 = 2;
qDChosenCtr2 = 2;

qChosenCtr3 = 2;
qDChosenCtr3 = 3;

qChosen1 = ranges.qRange(qChosenCtr1);
qChosen2 = ranges.qRange(qChosenCtr2);
qChosen3 = ranges.qRange(qChosenCtr3);
qDChosen1 = ranges.qDRange(qDChosenCtr1);
qDChosen2 = ranges.qDRange(qDChosenCtr2);
qDChosen3 = ranges.qDRange(qDChosenCtr3);

plot(ranges.plane_inclinationRange,stiffness(:,qChosenCtr1,qDChosenCtr1),'-ob','lineWidth',5.0);hold on;
plot(ranges.plane_inclinationRange,stiffness(:,qChosenCtr2,qDChosenCtr2),'-or','lineWidth',5.0);
plot(ranges.plane_inclinationRange,stiffness(:,qChosenCtr3,qDChosenCtr3),'-ok','lineWidth',5.0);
axis tight; grid on;
set(gca,'FontSize',36)

leg = legend(sprintf('q = %2.2f, qdot = %2.2f',qChosen1,qDChosen1),sprintf('q = %2.2f, qdot = %2.2f',qChosen2,qDChosen2),sprintf('q = %2.2f, qdot = %2.2f',qChosen3,qDChosen3));
set(leg,'Interpreter','latex','FontSize',48);

xlabel('Surface Orientation (\phi) [rad]','FontSize',48);
ylabel('Optimal Stiffness [Nm/rad]','FontSize',48);
set(gcf, 'Position', get(0,'Screensize'));

set(gcf, 'PaperPosition', [0 0 30 20]); %Position plot at left hand corner with width 5 and height 5.
set(gcf, 'PaperSize', [30 20]); %Set the paper to have width 5 and height 5.
saveas(gcf, 'stiff_over_inclination', 'pdf') %Save figure


fH = figure;%('units','normalized','outerposition',[0 0 1 1]);
plot(ranges.plane_inclinationRange,damping(:,qChosenCtr1,qDChosenCtr1),'-ob','lineWidth',5.0);hold on;
plot(ranges.plane_inclinationRange,damping(:,qChosenCtr2,qDChosenCtr2),'-or','lineWidth',5.0);
plot(ranges.plane_inclinationRange,damping(:,qChosenCtr3,qDChosenCtr3),'-ok','lineWidth',5.0);
axis tight; grid on;
set(gca,'FontSize',36)
%legend(sprintf('q = %2.2f, \dot{q} = %2.2f',qChosen1,qDChosen1),sprintf('q = %2.2f, qD = %2.2f',qChosen2,qDChosen2));
leg = legend(sprintf('q = %2.2f, qdot = %2.2f',qChosen1,qDChosen1),sprintf('q = %2.2f, qdot = %2.2f',qChosen2,qDChosen2),sprintf('q = %2.2f, qdot = %2.2f',qChosen3,qDChosen3));

set(leg,'Interpreter','latex','FontSize',48);
xlabel('Surface Orientation (\phi) [rad]','FontSize',48);
ylabel('Optimal Damping [Nm s/rad]','FontSize',48);
set(gcf, 'Position', get(0,'Screensize'));

set(gcf, 'PaperPosition', [0 0 30 20]); %Position plot at left hand corner with width 5 and height 5.
set(gcf, 'PaperSize', [30 20]); %Set the paper to have width 5 and height 5.
saveas(gcf, 'damp_over_inclination', 'pdf') %Save figure

% saveTightFigure(fH, 'damp_over_inclination.pdf');
% print(fH, '-deps', 'damp_over_inclination.eps');
% printImage(fH,'damp_over_inclination.pdf');

% %figure(1);
% for j = [1,4,7]
%      figure();
%     for i = 1: length(ranges.plane_inclinationRange)
%         %subplot(3,1,j);
%        
%         st1 = squeeze(stiffness(i,:,j));
%         plot(ranges.qRange,st1,'Color',cols1(i,:),'lineWidth',2.0); hold on;
%         ylabel('stiffness (k)');
%         xlabel('q');
%         grid on;
%         title(sprintf('Terminal joint velocity : %2.2f',ranges.qDRange(j)));
%         axis tight;
%     end
% end
% axis tight;
% 
% %figure(2); % damping vs orientation
% for j = [1,4,7]
%     figure();
%     for i = 1: length(ranges.plane_inclinationRange)
%         st1 = squeeze(stiffness(i,j,:));
%         plot(ranges.qDRange,st1,'Color',cols2(i,:),'lineWidth',2.0); hold on;
%         ylabel('stiffness (k)');
%         xlabel('qD');
%         grid on;
%         title(sprintf('Terminal joint position : %2.2f',ranges.qRange(j)));
%         axis tight;
%     end
% end
% axis tight;
