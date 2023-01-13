%% PMS Assignment 1 - Job Meijer
%
% This script contains three parts:
% 1. Simulating with manual values
% 2. Finding optimal lengths
% 3. Plotting found data
%
clear all, close all;

tic             % Start simulation time 

% Constants
v1 = 5;                 % Truck longitudinal speed [km/h], constant of 5.
Aa0 = 0;                % Initial articulating angle [degrees]
Rmin = 20/2;            % Minimal radius average roundabout [m]
Rmax = 30/2;            % Maximal radius average roundabout [m]
% Parameters for manual simulation
Manual_SA = 0;          % Manual steering angle, will be automatically determined if 0 [degrees]
Manual_L  = 9;          % Truck wheelbase [m]
Manual_L1 = -1;          % Truck rear axle to kingpin distance [m]
Manual_L2 = 6;          % Trailer wheelbase [m]

%% Finding optimum lengths
Try_L = [8:0.1:10];                       % Array with different values for L, max is 10.
Try_L1 = [-1.5:0.1:-0.5];                      % Array with different values for L1.
Try_L2 = [5:0.1:7];                      % Array with different values for L2, max is 10.
max_total_length = 16.75;               % Maximum length total vehicle

Arraylength_L = length(Try_L);          % Determine number of lengths L
Arraylength_L1 = length(Try_L1);        % Determine number of lengths L1
Arraylength_L2 = length(Try_L2);        % Determine number of lengths L2
Possible_Lengths = [0 0 0 0];           % Create a matrix for all possible solutions
Possible_SA_min = 0;                    % Set to zero  
Possible_SA_max = 0;                    % Set to zero

% Generate all possible combinations:
% based on total vehicle length, L1 < L and L2 > L1.
for a=1:Arraylength_L
    L=Try_L(a);
    
    for b=1:Arraylength_L1
        L1=Try_L1(b);
                     
        for c=1:Arraylength_L2
            L2=Try_L2(c);      
            
            % Calculate total length of vehicle
            Total_Length = L-L1+L2;
            
            % Filter solutions for max length and L1 smaller than L
            if (Total_Length <= max_total_length) && (L1 < L) && (L2 > L1)
                Possible_L(a) = L;
                Possible_L1(b) = L1;
                Possible_L2(c) = L2; 
                
                z1 = [L L1 L2 Total_Length];   % Add the found values to an array
                
                % Merge found values with already found values
                Possible_Lengths = [Possible_Lengths; z1]; 
            end
        end
    end   
end 
    
% Calculate steering angle and swept path for all possible solutions
Total_Possible_Lengths = size(Possible_Lengths, 1)
Possible_Solutions = [0 0 0 0 0 0 0 0];

% Trying all values
for d = 2:Total_Possible_Lengths
    
    Progress_Percentage = (d/Total_Possible_Lengths)*100    % Show progress in command window
    
    % Get lengths from matrix
    L = Possible_Lengths(d,1);              % Possibilities for L
    L1 = Possible_Lengths(d,2);             % Possibilities for L1
    L2 = Possible_Lengths(d,3);             % Possibilities for L2
    Total_Length = Possible_Lengths(d,4);   % Total length of possibilities
   
    % Determine steering angle (if possible)
    Try_SA2 = [0:1:40];                     % Try simulating with different steering angles
    Arraylength_SA2 = length(Try_SA2);      % Determine the length of the Try_SA array.
    Possible_SA_min = 0;                    % Reset 
    Possible_SA_max = 0;                    % Reset
    %    
    for i=1:Arraylength_SA2
    SA=Try_SA2(i); 

    sim Assignment1JM2015a.slx
    
    % Calculating middle of roundabout for finding radius
    maxxFA = max(DATA0(:,2));
    minxFA = min(DATA0(:,2));   
    middle_x = (maxxFA+minxFA)/2;
    maxyFA = max(DATA0(:,3));
    minyFA = min(DATA0(:,3));
    middle_y = (maxyFA+minyFA)/2;
      
    % Generate vectors for distance FA, RA & TA from middle roundabout
    LengthFA = length(DATA0(:,2));
    X_FA(1:LengthFA) = (DATA0(:,2)-middle_x);
    Y_FA(1:LengthFA) = (DATA0(:,3)-middle_y);
    X_FA = X_FA(1:size(DATA0));
    Y_FA = Y_FA(1:size(DATA0));
    X_FA = abs(X_FA);                       % Get absolute values for X front axle
    Y_FA = abs(Y_FA);                       % Get absolute values for Y front axle
    R_FA = sqrt((X_FA).^2+(Y_FA).^2);       % Calculate vector for front axle
    min_R_FA = min(R_FA);                   % Determine minimal vector length FA
    max_R_FA = max(R_FA);                   % Determine maximal vector length FA
    %
    LengthRA = length(DATA1(:,2));
    X_RA(1:LengthRA) = (DATA1(:,2)-middle_x);
    Y_RA(1:LengthRA) = (DATA1(:,3)-middle_y);
    X_RA = X_RA(1:size(DATA1));
    Y_RA = Y_RA(1:size(DATA1));
    X_RA = abs(X_RA);                       % Get absolute values for X rear axle
    Y_RA = abs(Y_RA);                       % Get absolute values for Y rear axle
    R_RA = sqrt((X_RA).^2+(Y_RA).^2);       % Calculate vector for rear axle
    min_R_RA = min(R_RA);                   % Determine minimal vector length RA
    max_R_RA = max(R_RA);                   % Determine maximal vector length RA
    %
    LengthTA = length(DATA2(:,2));
    X_TA(1:LengthTA) = (DATA2(:,2)-middle_x);
    Y_TA(1:LengthTA) = (DATA2(:,3)-middle_y);
    X_TA = X_TA(1:size(DATA2));
    Y_TA = Y_TA(1:size(DATA2));
    X_TA = abs(X_TA);                       % Get absolute values for X trailer axle
    Y_TA = abs(Y_TA);                       % Get absolute values for Y trailer axle
    R_TA = sqrt((X_TA).^2+(Y_TA).^2);       % Calculate vector for trailer axle    
    min_R_TA = min(R_TA);                   % Determine minimal vector length TA
    max_R_TA = max(R_TA);                   % Determine maximal vector length TA

    % Determine if steering angle is possible
        if (max_R_FA <= 15) && (min_R_FA >= 10)   % FA vector must be or within 10 and 15 meters
            if  min_R_TA >= 10              % Minimal TA vector must be equal or larger than 10 meters
           
                % Save first (and smallest) steering angle
                if Possible_SA_min == 0;
                    Possible_SA_min = SA;
                    SP_SA_min = (min_R_RA - min_R_TA);
                end
            
                % Save largest possible steering angle
                if Possible_SA_min ~= 0;
                    Possible_SA_max = SA;
                    SP_SA_max = (min_R_RA - min_R_TA);
                end
            end            
        end
        
        % Save all values with a possible steering angle to a matrix
        if (Possible_SA_min ~= 0)
            % Values to save: L, L1, L2, total length, steering angle, swept path.
            z2 = [L L1 L2 Total_Length Possible_SA_min Possible_SA_max SP_SA_min SP_SA_max];
            % Add found values to the other found values matrix
            Possible_Solutions = [Possible_Solutions; z2];
        end
    end 
end

%% Manual simulation

% Determining possible steering angle for manually entered lengths
Try_SA = [0:1:40];                  % Try simulating with different steering angles
Arraylength_SA = length(Try_SA);    % Determine the length of the Try_SA array.
Possible_SA_min = 0;                % Reset 
Possible_SA_max = 0;                % Reset

% Manual length values
L = Manual_L;
L1 = Manual_L1;
L2 = Manual_L2;

for i=1:Arraylength_SA
    SA=Try_SA(i); 
   
    sim Assignment1JM2015a.slx
   
    % Calculating middle of roundabout for finding radius
    maxxFA = max(DATA0(:,2));
    minxFA = min(DATA0(:,2));   
    middle_x = (maxxFA+minxFA)/2;
    maxyFA = max(DATA0(:,3));
    minyFA = min(DATA0(:,3));
    middle_y = (maxyFA+minyFA)/2;
      
    % Generate vectors for distance FA & TA from middle roundabout
    LengthFA = length(DATA0(:,2));
    X_FA(1:LengthFA) = (DATA0(:,2)-middle_x);
    Y_FA(1:LengthFA) = (DATA0(:,3)-middle_y);
    X_FA = X_FA(1:size(DATA0));
    Y_FA = Y_FA(1:size(DATA0));
    X_FA = abs(X_FA);                   % Get absolute values for X front axle
    Y_FA = abs(Y_FA);                   % Get absolute values for Y front axle
    R_FA = sqrt((X_FA).^2+(Y_FA).^2);   % Calculate vector for front axle
    min_R_FA = min(R_FA);               % Determine minimal vector length FA
    max_R_FA = max(R_FA);               % Determine maximal vector length FA
    %
    LengthRA = length(DATA1(:,2));
    X_RA(1:LengthRA) = (DATA1(:,2)-middle_x);
    Y_RA(1:LengthRA) = (DATA1(:,3)-middle_y);
    X_RA = X_RA(1:size(DATA1));
    Y_RA = Y_RA(1:size(DATA1));
    X_RA = abs(X_RA);                   % Get absolute values for X rear axle
    Y_RA = abs(Y_RA);                   % Get absolute values for Y rear axle
    R_RA = sqrt((X_RA).^2+(Y_RA).^2);   % Calculate vector for rear axle
    min_R_RA = min(R_RA);               % Determine minimal vector length RA
    max_R_RA = max(R_RA);               % Determine maximal vector length RA
    %
    LengthTA = length(DATA2(:,2));
    X_TA(1:LengthTA) = (DATA2(:,2)-middle_x);
    Y_TA(1:LengthTA) = (DATA2(:,3)-middle_y);
    X_TA = X_TA(1:size(DATA2));
    Y_TA = Y_TA(1:size(DATA2));
    X_TA = abs(X_TA);                   % Get absolute values for X trailer axle
    Y_TA = abs(Y_TA);                   % Get absolute values for Y trailer axle
    R_TA = sqrt((X_TA).^2+(Y_TA).^2);   % Calculate vector for trailer axle    
    min_R_TA = min(R_TA);               % Determine minimal vector length TA
    max_R_TA = max(R_TA);               % Determine maximal vector length TA

    % Determine if steering angle is possible
    if (max_R_FA <= 15) && (min_R_FA >= 10)   % FA vector must be or within 10 and 15 meters
        if  min_R_TA >= 10              % Minimal TA vector must be equal or larger than 10 meters
           
            % Save first (and smallest) steering angle
            if Possible_SA_min == 0;
                Possible_SA_min = SA;
            end
            
            % Save largest possible steering angle
            if Possible_SA_min ~= 0;
                Possible_SA_max = SA;
            end
        end            
    end
end


%% Simulate with found steering angle and generate plots

% Use average of smallest and largest possible steering angle and simulate
if Possible_SA_min == 0
   errordlg('No possible steering angle found!')
else
    SA = (Possible_SA_min+Possible_SA_max)/2;
end    
    
if (Manual_SA ~= 0)
    if (Manual_SA <= 40) && (Manual_SA >= 0)
        SA = Manual_SA          
    else
        errordlg('Manual steering angle should be greater than 0 and smaller or equal to 40')
    end
end


% Calculate average of possible steering angles for simulation
sim Assignment1JM2015a.slx

    % Calculating middle of roundabout for plotting
    maxxFA = max(DATA0(:,2));
    minxFA = min(DATA0(:,2));   
    middle_x = (maxxFA+minxFA)/2;
    maxyFA = max(DATA0(:,3));
    minyFA = min(DATA0(:,3));
    middle_y = (maxyFA+minyFA)/2;

% Determine swept path for plotting
% Generate vectors for distance FA & TA from middle roundabout
    LengthFA = length(DATA0(:,2));
    X0 = DATA0(:,2);                    % Save X0 values
    Y0 = DATA0(:,3);                    % Save Y0 values
    X_FA(1:LengthFA) = (DATA0(:,2)-middle_x);
    Y_FA(1:LengthFA) = (DATA0(:,3)-middle_y);
    X_FA = abs(X_FA);                   % Get absolute values for X front axle
    Y_FA = abs(Y_FA);                   % Get absolute values for Y front axle
    R_FA = sqrt((X_FA).^2+(Y_FA).^2);   % Calculate vector for front axle
    min_R_FA = min(R_FA);               % Determine minimal vector length FA
    max_R_FA = max(R_FA);               % Determine maximal vector length FA
    %
    LengthRA = length(DATA1(:,2));
    X1 = DATA1(:,2);                    % Save X1 values
    Y1 = DATA1(:,3);                    % Save Y1 values
    X_RA(1:LengthRA) = (DATA1(:,2)-middle_x);
    Y_RA(1:LengthRA) = (DATA1(:,3)-middle_y);
    X_RA = abs(X_RA);                   % Get absolute values for X rear axle
    Y_RA = abs(Y_RA);                   % Get absolute values for Y rear axle
    R_RA = sqrt((X_RA).^2+(Y_RA).^2);   % Calculate vector for rear axle
    min_R_RA = min(R_RA);               % Determine minimal vector length RA
    max_R_RA = max(R_RA);               % Determine maximal vector length RA
    %
    LengthTA = length(DATA2(:,2));
    X2 = DATA2(:,2);                    % Save X2 values
    Y2 = DATA2(:,3);                    % Save Y2 values
    X_TA(1:LengthTA) = (DATA2(:,2)-middle_x);
    Y_TA(1:LengthTA) = (DATA2(:,3)-middle_y);
    X_TA = abs(X_TA);                   % Get absolute values for X trailer axle
    Y_TA = abs(Y_TA);                   % Get absolute values for Y trailer axle
    R_TA = sqrt((X_TA).^2+(Y_TA).^2);   % Calculate vector for trailer axle    
    min_R_TA = min(R_TA);               % Determine minimal vector length TA
    max_R_TA = max(R_TA);               % Determine maximal vector length TA


% Plotting all figures
Fig = figure(1);                        % Select  firuge 1
set(Fig, 'Position', [0 35 960 960]);   % Set position for figure 1

subplot 221;
hold on
% plotting axle positions on a roundabout
plot(DATA0(:,2),DATA0(:,3),'g','LineWidth',2);
plot(DATA1(:,2),DATA1(:,3),'b','LineWidth',2);
plot(DATA2(:,2),DATA2(:,3),'r','LineWidth',2);
plot(middle_x, middle_y, '*black', 'LineWidth',2);
% Generating and plotting roundabout
th = 0:pi/50:2*pi;
xmin = Rmin*cos(th)+middle_x;
ymin = Rmin*sin(th)+middle_y;
roundmin = plot(xmin, ymin, '--black','LineWidth', 2);
xmax = Rmax*cos(th)+middle_x;
ymax = Rmax*sin(th)+middle_y;
roundmax = plot(xmax, ymax, '--black','Linewidth', 2);
hold off
legend('Front axle position','Rear axle position','Trailer axle position');
grid on
xlabel('Horizontal movement [m]');
ylabel('Vertical movement [m]');
title('All axle positions on roundabout');

% Plotting Psi1, Psi2, Aa and SA
subplot 222;
hold on 
plot(DATA3(:,1), DATA3(:,2), 'g', 'LineWidth', 2);
plot(DATA3(:,1), DATA3(:,3), 'b', 'LineWidth', 2);
plot(DATA3(:,1), DATA3(:,8), 'r', 'LineWidth', 2);
plot(DATA3(:,1), DATA3(:,9), 'black', 'LineWidth', 2);
hold off
grid on
xlabel('Time [s]');
ylabel('Angle [Rad]');
legend('Psi1 [Rad]','Psi2 [Rad]', 'Aa [Rad]', 'SA [Rad]');
title('Simulation parameters over time');

% Plotting trailer turn radius compared to lengths
subplot 223;
hold on
Plot_L = Possible_Solutions(2:length(Possible_Solutions),1);
Plot_L1 = Possible_Solutions(2:length(Possible_Solutions),2);
Plot_L2 = Possible_Solutions(2:length(Possible_Solutions),3);
Plot_SP = Possible_Solutions(2:length(Possible_Solutions),7);
cla 
scatter3(Plot_L2, Plot_L, Plot_L1, 10, Plot_SP, 'filled')  % Pointsize=10
ax = gca;
ax.XDir = 'reverse';
view(205,5)                 % Set viewpoint
xlabel('L2')
ylabel('L')
zlabel('L1')
cb = colorbar;
cb.Label.String = 'Swept path at minimal possible steering angle';
hold off
grid on
title('Swept path at smallest possible steering angle');

% Plotting only combinations with total length >= 16 and L1 between -1&1 
Max_Length_Combi = [0 0 0 0 0 0 0 0];
for e=1:length(Possible_Solutions(:,1))
    if (Possible_Solutions(e,4) >= 16) && (Possible_Solutions(e,2) >= -1) && (Possible_Solutions(e,2) <= 1)
        Max_Length_Combi = [Max_Length_Combi; Possible_Solutions(e,:)];
    end
end
Max_Length_Combi(1,:) = [];

subplot 224;
hold on
Plot_L = Max_Length_Combi(1:length(Max_Length_Combi),1);
Plot_L1 = Max_Length_Combi(1:length(Max_Length_Combi),2);
Plot_L2 = Max_Length_Combi(1:length(Max_Length_Combi),3);
Plot_SP = Max_Length_Combi(1:length(Max_Length_Combi),7);
cla 
scatter3(Plot_L2, Plot_L, Plot_L1, 10, Plot_SP, 'filled')  % Pointsize=10
ax = gca;
ax.XDir = 'reverse';
view(205,5)                 % Set viewpoint
xlabel('L2')
ylabel('L')
zlabel('L1')
cb = colorbar;
cb.Label.String = 'Swept path at minimal possible steering angle';
hold off
grid on
title('Total length >= 16m and -1<L1<1 compared to swept path');

% Finding combination that combines largest total length with smallest swept path
for f=1:length(Max_Length_Combi(:,1))
    if (Max_Length_Combi(f,4) >= Optimum_Combi(:,4)) && (Max_Length_Combi(f,7) <= Optimum_Combi(:,7))
        Optimum_Combi(1,:) = Max_Length_Combi(f,:);
    end
end

% Optimum lengths are
Optimum_L = Optimum_Combi(1,1)
Optimum_L1 = Optimum_Combi(1,2)
Optimum_L2 = Optimum_Combi(1,3)
Optimum_Totallength = Optimum_Combi(1,4)
Optimum_Sweptpath = Optimum_Combi(1,7)


%% Stop simulation time
toc             
