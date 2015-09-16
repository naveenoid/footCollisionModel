clear
close all;

plotFigBaseFolder = './plots/icra_final/';    
if(~exist(plotFigBaseFolder))
    mkdir(plotFigBaseFolder);
end

plotFigBaseName = strcat(plotFigBaseFolder,'damping_');

load('./data/multiTestResult.mat');
model.foot.length = 0.2;
Jtotal = (resultStore.CoPResult - model.foot.length/2*ones(size(resultStore.CoPResult))).^2;

if(length(ranges.dampRange) == 1)
    figure;
    surf(ranges.stiffnessRange,ranges.plane_inclinationRange,Jtotal );  
    xlabel('K'); ylabel('\phi'); zlabel('J');   
else
    %figure;       
    for i = 1:length(ranges.plane_inclinationRange)

        figure;
        contourf(ranges.stiffnessRange,ranges.dampRange,squeeze(Jtotal(i,:,:)) );  
        xlabel('K'); ylabel('b'); zlabel('J');   
        title(sprintf('Orientation = %2.2f',ranges.plane_inclinationRange(i)));
%optimalVals.stiffnessMean = stiffnessMean;
%    optimalVals.dampingMean = dampingMean;
    hold on;
    %    plot(optimalVals.stiffnessMean(i),optimalVals.dampingMean(i),'ko','LineWidth',2);
        set(gca,'FontSize',16);
        set(gcf,'Renderer','OpenGL');
        print('-dpng','-r400',strcat(plotFigBaseName,...
        sprintf('%d',round(10*ranges.plane_inclinationRange(i)))),'-opengl');

    end
end
   

figure;
    %for i = 1:length(ranges.plane_inclinationRange)

        %figure;
% contourslice(Jtotal,[],[],linspace(ranges.plane_inclinationRange(1),...
%     ranges.plane_inclinationRange(end),3),6);  
      %  xlabel('K'); ylabel('b'); zlabel('J');   
        %title(sprintf('Orientation = %2.2f',ranges.plane_inclinationRange(i)));
        
       % clabel(cs,h); hold on;
%optimalVals.stiffnessMean = stiffnessMean;
%    optimalVals.dampingMean = dampingMean;
   % hold on;
    %    plot(optimalVals.stiffnessMean(i),optimalVals.dampingMean(i),'ko','LineWidth',2);
       % set(gca,'FontSize',16);
       % set(gcf,'Renderer','OpenGL');
       % print('-dpng','-r400',strcat(plotFigBaseName,...
       % sprintf('%d',round(10*ranges.plane_inclinationRange(i)))),'-opengl');
    %end