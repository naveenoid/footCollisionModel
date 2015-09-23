function prepareGazebo(plotForPaper)

files = {'data/incl_m5_k5_d3.mat','data/incl_m5_k8_d3.mat', 'data/incl_m5_k10_d3.mat', ...
    'data/incl_m5_k15_d3.mat','data/incl_m5_k20_d3.mat', ...
    'data/incl_m5_k30_d3.mat'};%, 'data/incl_m5_k80_d3.mat'};

J_m5 = cleanData(files);

files = {'data/incl_m10_k5_d3.mat', 'data/incl_m10_k8_d3.mat', 'data/incl_m10_k10_d3.mat', ...
    'data/incl_m10_k15_d3.mat','data/incl_m10_k20_d3.mat', ...
    'data/incl_m10_k30_d3.mat'};%, 'data/incl_m10_k80_d3.mat'};

J_m10 = cleanData(files);

files = {'data/incl_m7_5_k5_d3.mat', 'data/incl_m7_5_k8_d3.mat', 'data/incl_m7_5_k10_d3.mat', ...
    'data/incl_m7_5_k15_d3.mat','data/incl_m7_5_k20_d3.mat', ...
    'data/incl_m7_5_k30_d3.mat'};%, 'data/incl_m10_k80_d3.mat'};

J_m7_5 = cleanData(files);

files = {'data/incl_5_k5_d3.mat','data/incl_5_k8_d3.mat', 'data/incl_5_k10_d3.mat', ...
    'data/incl_5_k15_d3.mat','data/incl_5_k20_d3.mat', ...
    'data/incl_5_k30_d3.mat'};%, 'data/incl_m5_k80_d3.mat'};

J_5 = cleanData(files);

files = {'data/incl_10_k5_d3.mat', 'data/incl_10_k8_d3.mat', 'data/incl_10_k10_d3.mat', ...
    'data/incl_10_k15_d3.mat','data/incl_10_k20_d3.mat', ...
    'data/incl_10_k30_d3.mat'};%, 'data/incl_m10_k80_d3.mat'};

J_10 = cleanData(files);

files = {'data/incl_7_k5_d3.mat', 'data/incl_7_k8_d3.mat', 'data/incl_7_k10_d3.mat', ...
    'data/incl_7_k15_d3.mat','data/incl_7_k20_d3.mat', ...
    'data/incl_7_k30_d3.mat'};

J_7_5 = cleanData(files);

stiff = [5, 8, 10, 15, 20, 30]';%, 80]';
stiff = stiff ./ 180 .* pi;

figure;
cols = autumn(6);
doPaper = 0;
if (exist('plotForPaper', 'var') && plotForPaper == 1)
    doPaper = 1;
end

if (doPaper)
    lineWidth = 5;
    labelsFontSize = 48;
    axisFontSize = 36;
else
    lineWidth = 1.0; %5;
    labelsFontSize = 12; %48
    axisFontSize = 12; %36
end

plot(stiff, J_m5, 'LineWidth', lineWidth);
hold on;
plot(stiff, J_m7_5, 'LineWidth', lineWidth);
plot(stiff, J_m10, 'LineWidth', lineWidth);
plot(stiff, J_5, 'LineWidth', lineWidth);
plot(stiff, J_7_5, 'LineWidth', lineWidth);
plot(stiff, J_10, 'LineWidth', lineWidth);

xlabel('Stiffness [Nm/rad]','FontSize',labelsFontSize)
ylabel('Cost','FontSize',labelsFontSize);
leg = legend('0.09 rad', '0.13 rad', '0.17 rad','-0.09 rad', '-0.13 rad', '-0.17 rad','Location','NorthWest');
set(leg,'Interpreter','latex','FontSize',labelsFontSize);
set(gcf, 'Position', get(0,'Screensize'));
set(gca,'FontSize',axisFontSize);
grid on;

if (doPaper)
    set(gcf, 'PaperPosition', [0 0 30 20]); %Position plot at left hand corner with width 5 and height 5.
    set(gcf, 'PaperSize', [30 20]); %Set the paper to have width 5 and height 5.
    saveas(gcf, 'gazebo_plots', 'pdf') %Save figure
end

Jtotal = zeros(6,6);

Jtotal(:,1) = J_m10;
Jtotal(:,2) = J_m7_5;
Jtotal(:,3) = J_m5;
Jtotal(:,4) = J_5;
Jtotal(:,5) = J_7_5;
Jtotal(:,6) = J_10;

disp(Jtotal);

disp('blah');

Orientation = [-10,-7.5,-5,5,7.5,10];
figure;
contourf(stiff,Orientation,Jtotal);
xlabel('Stiffness [Nm/rad]');
ylabel('Surface Orientation \phi [rad]');colorbar;colormap(winter)
    set(gcf, 'PaperPosition', [0 0 30 20]); %Position plot at left hand corner with width 5 and height 5.
    set(gcf, 'PaperSize', [30 20]); %Set the paper to have width 5 and height 5.
    saveas(gcf, 'gazebo_surf_plots', 'pdf') %Save figure
end


function J = cleanData(files)
    J = zeros(length(files), 1);
    for i = 1: length(files)
        data = load(files{i});

        %get the valid data
        conditionStartIndex = find(data.conditions.signals(1).values >= 1);
        conditionStartIndex = conditionStartIndex(1);
        croppedConditions = data.conditions.signals(1).values(conditionStartIndex:end);
        conditionEndIndex = find(croppedConditions <= 0);
        if isempty(conditionEndIndex)
            if (isempty(croppedConditions))
                fprintf('Index %d is empty\n', i);
                J(i) = inf;
                continue;
            else
                conditionEndIndex = length(data.conditions.signals(1).values); %just the end
            end
        else
            conditionEndIndex = conditionEndIndex(1) + conditionStartIndex - 2;
        end

        cop = data.cop_left.signals(1).values(conditionEndIndex);
        J(i) = 0.5 * cop^2;
    end
end
