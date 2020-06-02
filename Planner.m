%% Planner
initialLocation = [-10,1; 8,1; 4,8; 8,8]; %representing the initial location of each robot
goalLocation = [15,1; 2,2; 19,9; 17,6];% teh position of tasks
M = zeros(4,4);
for i = 1:4
    for j = 1:4
        M(i,j) = norm(initialLocation(i,:) - goalLocation(j,:));
    end
end
[A,C]= munkres(M); % A is the optimal assignment matrix, C is the total minimum cost
pi = A.*M; % calculate pi*
%% plots of trajectories
% This step is to form four linear trajectories for the four robots, which has already been well calculated above.  Because of the feature of HUngarian Algorithm, the four trajectories should not intersect each other.

%In MATLAB document, waypointTrajectory can help us build a trajectory objectory, howecer, now I just focus on the primitive linea lines.
%labels = cellstr(num2str([1:4]')) % create the order and lables
a = zeros(1,4);
b = zeros(1,4);
for i = 1:4
    for j = 1:4
        if A(i,j) == 1
            figure(1)
            axis([-20,20,-20,20])
            plot([initialLocation(i,1), goalLocation(j,1)], [initialLocation(i,2),goalLocation(j,2)])
            coefficients = polyfit([initialLocation(i,1),goalLocation(j,1)],[initialLocation(i,2),goalLocation(j,2)],1);
            a(i) = coefficients(1);
            b(i) = coefficients(2);
            OrderOfRobot = num2str(i);
            OrderOfGoal = num2str(j);
            text(initialLocation(i,1), initialLocation(i,2),OrderOfRobot,'FontSize',12)
            text(initialLocation(i,1), initialLocation(i,2),'o','color','g')
            text(goalLocation(j,1), goalLocation(j,2),OrderOfGoal,'FontSize',15)
            text(goalLocation(j,1), goalLocation(j,2),'*','color','r')
            axis manual
            hold on
        end
    end
end
% The primitive tradjectory is done, the next step is to construct the collision avoidance function into planner, then refine tradjectory.

% Problem: when the shortest oath is verticle, the line parameters of line function fails to represent it.

% In this stage, I do not want too many things about differential cars, that may left to the next few steps.
%% functions of trajectories

% In this section, i will plot those trajectories in a new form for further utilization.
yTrajectory = cell(1,4);
xTrajectory = cell(1,4);

for i = 1:4
    for j = 1:4
        if A(i,j) == 1
            xTrajectory{i} = linspace(initialLocation(i,1),goalLocation(j,1),100);
            yTrajectory{i} = a(i) * xTrajectory{i} + b(i);
        end
    end
end
%% robot animination

% In this section, the robot will be modelled as a point and move along the trajectory

% Velocity havn't been sepcified, and waypointTrajectory havn't been used.

% In this program, there must be four object moving along four trajectory.

ax = gca; % gca: Get handle to the current axis. Return the handle to the current axis in the current figure.
h1 = hgtransform('Parent', ax);
h2 = hgtransform('Parent', ax);
h3 = hgtransform('Parent', ax);
h4 = hgtransform('Parent', ax);
hold on
%for i = 1:length(xTrajectory)    
    plot(xTrajectory{1}(1),yTrajectory{1}(1),'ro','Parent',h1); % In this line, I deleted some parameters such as 'VerticalAlignment', 'top' and so on, i may should add them on later.
    plot(xTrajectory{2}(1),yTrajectory{2}(1),'ro','Parent',h2);
    plot(xTrajectory{3}(1),yTrajectory{3}(1),'ro','Parent',h3);
    plot(xTrajectory{4}(1),yTrajectory{4}(1),'ro','Parent',h4);
    hold off
    t1 = text(xTrajectory{1}(1),yTrajectory{1}(1),num2str(yTrajectory{1}(1)),'Parent',h1, 'VerticalAlignment','top','FontSize',14);
    t2 = text(xTrajectory{2}(1),yTrajectory{2}(1),num2str(yTrajectory{2}(1)),'Parent',h2, 'VerticalAlignment','top','FontSize',14);
    t3 = text(xTrajectory{3}(1),yTrajectory{3}(1),num2str(yTrajectory{3}(1)),'Parent',h3, 'VerticalAlignment','top','FontSize',14);
    t4 = text(xTrajectory{4}(1),yTrajectory{4}(1),num2str(yTrajectory{4}(1)),'Parent',h4, 'VerticalAlignment','top','FontSize',14);    
    for k = 2:100 % 99 iterations
        m1 = makehgtform('translate', xTrajectory{1}(k)-xTrajectory{1}(1),yTrajectory{1}(k) - yTrajectory{1}(1), 0);
        m2 = makehgtform('translate', xTrajectory{2}(k)-xTrajectory{2}(1),yTrajectory{2}(k) - yTrajectory{2}(1), 0);
        m3 = makehgtform('translate', xTrajectory{3}(k)-xTrajectory{3}(1),yTrajectory{3}(k) - yTrajectory{3}(1), 0);
        m4 = makehgtform('translate', xTrajectory{4}(k)-xTrajectory{4}(1),yTrajectory{4}(k) - yTrajectory{4}(1), 0);
        h1.Matrix = m1;
        h2.Matrix = m2;
        h3.Matrix = m3;
        h4.Matrix = m4;
        t1.String = num2str(xTrajectory{1}(k)) + "," + num2str(yTrajectory{1}(k));
        t2.String = num2str(xTrajectory{2}(k)) + "," + num2str(yTrajectory{2}(k));
        t3.String = num2str(xTrajectory{3}(k)) + "," + num2str(yTrajectory{3}(k));
        t4.String = num2str(xTrajectory{4}(k)) + "," + num2str(yTrajectory{4}(k));
        drawnow
    end
    hold off
    % Yeah, I know this program is very tedius and looks as a rigmorale. I will fix it when i finish all the essential procedures.
    
%end

% So, now the question is, how to let four robots(points) moving somutaneously on the four trajjectories?
%%  Clearance Function theta






















%% Hungarian Algorithm
function [assignment,cost] = munkres(costMat)
% MUNKRES   Munkres Assign Algorithm 
%
% [ASSIGN,COST] = munkres(COSTMAT) returns the optimal assignment in ASSIGN
% with the minimum COST based on the assignment problem represented by the
% COSTMAT, where the (i,j)th element represents the cost to assign the jth
% job to the ith worker.
%
% This is vectorized implementation of the algorithm. It is the fastest
% among all Matlab implementations of the algorithm.
% Examples
% Example 1: a 5 x 5 example
%{
[assignment,cost] = munkres(magic(5));
[assignedrows,dum]=find(assignment);
disp(assignedrows'); % 3 2 1 5 4
disp(cost); %15
%}
% Example 2: 400 x 400 random data
%{
n=400;
A=rand(n);
tic
[a,b]=munkres(A);
toc                 % about 6 seconds 
%}
% Reference:
% "Munkres' Assignment Algorithm, Modified for Rectangular Matrices", 
% http://csclab.murraystate.edu/bob.pilgrim/445/munkres.html
% version 1.0 by Yi Cao at Cranfield University on 17th June 2008
assignment = false(size(costMat));
cost = 0;
costMat(costMat~=costMat)=Inf;
validMat = costMat<Inf;
validCol = any(validMat);
validRow = any(validMat,2);
nRows = sum(validRow);
nCols = sum(validCol);
n = max(nRows,nCols);
if ~n
    return
end
    
dMat = zeros(n);
dMat(1:nRows,1:nCols) = costMat(validRow,validCol);
%*************************************************
% Munkres' Assignment Algorithm starts here
%*************************************************
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   STEP 1: Subtract the row minimum from each row.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 dMat = bsxfun(@minus, dMat, min(dMat,[],2));
%**************************************************************************  
%   STEP 2: Find a zero of dMat. If there are no starred zeros in its
%           column or row start the zero. Repeat for each zero
%**************************************************************************
zP = ~dMat;
starZ = false(n);
while any(zP(:))
    [r,c]=find(zP,1);
    starZ(r,c)=true;
    zP(r,:)=false;
    zP(:,c)=false;
end
while 1
%**************************************************************************
%   STEP 3: Cover each column with a starred zero. If all the columns are
%           covered then the matching is maximum
%**************************************************************************
    primeZ = false(n);
    coverColumn = any(starZ);
    if ~any(~coverColumn)
        break
    end
    coverRow = false(n,1);
    while 1
        %**************************************************************************
        %   STEP 4: Find a noncovered zero and prime it.  If there is no starred
        %           zero in the row containing this primed zero, Go to Step 5.  
        %           Otherwise, cover this row and uncover the column containing 
        %           the starred zero. Continue in this manner until there are no 
        %           uncovered zeros left. Save the smallest uncovered value and 
        %           Go to Step 6.
        %**************************************************************************
        zP(:) = false;
        zP(~coverRow,~coverColumn) = ~dMat(~coverRow,~coverColumn);
        Step = 6;
        while any(any(zP(~coverRow,~coverColumn)))
            [uZr,uZc] = find(zP,1);
            primeZ(uZr,uZc) = true;
            stz = starZ(uZr,:);
            if ~any(stz)
                Step = 5;
                break;
            end
            coverRow(uZr) = true;
            coverColumn(stz) = false;
            zP(uZr,:) = false;
            zP(~coverRow,stz) = ~dMat(~coverRow,stz);
        end
        if Step == 6
            % *************************************************************************
            % STEP 6: Add the minimum uncovered value to every element of each covered
            %         row, and subtract it from every element of each uncovered column.
            %         Return to Step 4 without altering any stars, primes, or covered lines.
            %**************************************************************************
            M=dMat(~coverRow,~coverColumn);
            minval=min(min(M));
            if minval==inf
                return
            end
            dMat(coverRow,coverColumn)=dMat(coverRow,coverColumn)+minval;
            dMat(~coverRow,~coverColumn)=M-minval;
        else
            break
        end
    end
    %**************************************************************************
    % STEP 5:
    %  Construct a series of alternating primed and starred zeros as
    %  follows:
    %  Let Z0 represent the uncovered primed zero found in Step 4.
    %  Let Z1 denote the starred zero in the column of Z0 (if any).
    %  Let Z2 denote the primed zero in the row of Z1 (there will always
    %  be one).  Continue until the series terminates at a primed zero
    %  that has no starred zero in its column.  Unstar each starred
    %  zero of the series, star each primed zero of the series, erase
    %  all primes and uncover every line in the matrix.  Return to Step 3.
    %**************************************************************************
    rowZ1 = starZ(:,uZc);
    starZ(uZr,uZc)=true;
    while any(rowZ1)
        starZ(rowZ1,uZc)=false;
        uZc = primeZ(rowZ1,:);
        uZr = rowZ1;
        rowZ1 = starZ(:,uZc);
        starZ(uZr,uZc)=true;
    end
end
% Cost of assignment
assignment(validRow,validCol) = starZ(1:nRows,1:nCols);
cost = sum(costMat(assignment));
end