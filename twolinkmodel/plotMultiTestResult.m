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
    for i = 1:length(ranges.dampRange)

        figure;
        contourf(ranges.stiffnessRange,ranges.plane_inclinationRange,Jtotal(:,:,i) );  
        xlabel('K'); ylabel('\phi'); zlabel('J');   
        title(sprintf('Damping = %2.2f',ranges.dampRange(i)));


        set(gca,'FontSize',16);
        set(gcf,'Renderer','OpenGL');
        print('-dpng','-r400',strcat(plotFigBaseName,...
        sprintf('%d',round(10*ranges.dampRange(i)))),'-opengl');

    end
end
   