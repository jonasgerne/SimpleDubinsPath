function [] = Dubin(sPoint, gPoint)
%Dubin: finds the shortest path among the sequences RLR, LRL, LSL, LSR, RSL
%and RSR for a Dubins Car
%
%   PARAMETERS
%   -----------
%   sPoint:     array, shape [1, 3]
%               Starting configuration [x; y; orientation in deg]
%   gPoint:     array, shape [1, 3]
%               Goal configuration [x; y; orientation in deg]
%               by setting the orientation of the goal configuration to -1
%               all possible orientations are tested at an interval of STEP
%               (default value 1) and the shortest one is returned.
%               
% Jonas Gerstner, 13.04.2018

sequences = {'LSR', 'RSL', 'RSR', 'LSL', 'LRL', 'RLR'};
S = [sPoint(1:2); 0];
G = [gPoint(1:2); 0];
sAngle = sPoint(3);
sVec = [cosd(sAngle); sind(sAngle); 0];
STEP = 1;
    
    % goal orientation free
    if gPoint(3) == -1 
        best_l = inf;
        for j = 1:STEP:360
            % generate the goal vector for the current j value
            gVec = [cosd(j); sind(j); 0];
            % retrieve values for all sequences
            paths = [LSR(S, G, sVec, gVec); RSL(S, G, sVec, gVec); RSR(S, G, sVec, gVec); LSL(S, G, sVec, gVec); LRL(S, G, sVec, gVec); RLR(S, G, sVec, gVec)];
            % get the path lengths
            l = [paths.length];
            % set length to inf if sequence returned -1
            l(l<0) = inf;
            % get minimum
            [m, idx] = min(l);
            % compare to previous best
            if m < best_l
                best_l = m;
                best_path = paths(idx);
                best_idx = idx;
                best_angle = j;
            end
        end
        plotPath(best_path, S, G, best_idx, sequences);
        fprintf("The shortest path is %s at an angle of %f degrees with a length of %f.\n", sequences{best_idx}, best_angle, best_l);
    
    % goal orientation set
    else      
        gAngle = gPoint(3);
        gVec = [cosd(gAngle); sind(gAngle); 0];
        % get the paths
        paths = [LSR(S, G, sVec, gVec); RSL(S, G, sVec, gVec); RSR(S, G, sVec, gVec); LSL(S, G, sVec, gVec); LRL(S, G, sVec, gVec); RLR(S, G, sVec, gVec)];
        % retrieve length values
        l = [paths.length];
        % set length to inf if sequence returned -1
        l(l<0) = inf;
        % get minimum
        [m, idx] = min(l);
        % print result to console
        printResults(sequences, paths);
        fprintf("%s gives the minimum path length.\n", sequences{idx});
        % plot the shortest path
        % TODO: add option to plot all feasible paths
        plotPath(paths(idx), S, G, idx, sequences);
    end
end


function [] = plotPath(path, S, G, idx, sequences)
    % this function plots the given path
    figure(1) 
    
    % get the name of the choosen shortest path to set it as the title
    name = sequences(idx);
    % depending on the index, i.e. the sequence, the draw arc and the plot function is called
    if idx == 1
        drawArc(path.C1, S, path.s1, 'l');
        hold on;
        plot([path.T1(1) path.T2(1)],[path.T1(2) path.T2(2)]); 
        drawArc(path.C2, path.T2, path.s2, 'r');
    elseif idx == 2
        drawArc(path.C1, S, path.s1, 'r');
        hold on;
        plot([path.T1(1) path.T2(1)],[path.T1(2) path.T2(2)]); 
        drawArc(path.C2, path.T2, path.s2, 'l');
    elseif idx == 3
        drawArc(path.C1, S, path.s1, 'r');
        hold on;
        plot([path.T1(1) path.T2(1)],[path.T1(2) path.T2(2)]); 
        drawArc(path.C2, path.T2, path.s2, 'r');
    elseif idx == 4
        drawArc(path.C1, S, path.s1, 'l');
        hold on;
        plot([path.T1(1) path.T2(1)],[path.T1(2) path.T2(2)]); 
        drawArc(path.C2, path.T2, path.s2, 'l');
    elseif idx == 5
        drawArc(path.C1, S, path.s1, 'l');
        hold on;
        drawArc(path.C3, path.T1, path.s2, 'r'); 
        drawArc(path.C2, path.T2, path.s3, 'l');
    else
        drawArc(path.C1, S, path.s1, 'r');
        hold on;
        drawArc(path.C3, path.T1, path.s2, 'l'); 
        drawArc(path.C2, path.T2, path.s3, 'r');
    end
    % plot start and end point
    plot(S(1),S(2),'ro');
    plot(G(1), G(2), 'bx');
    grid on;
    axis equal
    title(name);
    hold off;
    % TODO: fit canvas to plot, i.e. add margin around the plotted path
end

function [] = printResults(sequences, paths) 
% prints the sequences to the console
    for i = 1:6
        if paths(i).length == -1
            fprintf("%s not feasible.\n", sequences{i});
        else
            fprintf("%s:\nlength: %f\nT1= (%f, %f)\nT2= (%f, %f)\n", sequences{i}, paths(i).length, paths(i).T1(1), paths(i).T1(2), paths(i).T2(1), paths(i).T2(2));
        end 
    end
end

function [centerPoint] = center(point, vec, dir)
% gives center of turning circle of radius 1
    if dir == 'r'
        centerPoint = point + cross(vec, [0; 0; 1]);
    else
        centerPoint = point - cross(vec, [0; 0; 1]);
    end
end

function [T] = Tangentpoint(P, dir, a)
% this function returns the tangent point for a straight line and a circle
% of radius 1 depending on the turning direction of the circle
    alpha = acosd(2/sqrt(sum(a.^2)));
    if dir == 'l'
        T = P + cosd(alpha)*(a/norm(a)) - sind(alpha)*cross([0; 0; 1], a/norm(a));
    else
        T = P + cosd(alpha)*(a/norm(a)) + sind(alpha)*cross([0; 0; 1], a/norm(a));
    end
end

function s = circleLength(A, B, vec)
% return the length of a circle segment
    A_new = round(A, 5);            % avoid unwanted effects with very small numbers around 0
    B_new = round(B, 5);
    w = B_new - A_new;
    beta = acos(dot(vec,w)/(norm(vec)*norm(w)));
    if isnan(beta)                  % avoid NaN when angle is zero
        beta = 0;
    end
    s = 2*beta;    
end

function path = LRL(sPoint, gPoint, sVec, gVec)
    path = DubinsPath;
    path.C1 = center(sPoint, sVec, 'l');
    path.C2 = center(gPoint, gVec, 'l');
    a = path.C2 - path.C1;
    if (norm(a)-2) > 2
          return;
    else
        % A is center between centerpoints of L circles
        A = path.C1 + 0.5*a;
        if norm(a) ~= 0
            w = sqrt(4 - 0.25*sqrt(sum(a.^2))^2)*cross([0; 0; 1], (a/sqrt(sum(a.^2))));
        else
            w = [0; 0; 0];
        end
        path.C3 = A + w;
        path.T1 = path.C1 + 0.5*(path.C3 - path.C1);
        path.T2 = path.C2 + 0.5*(path.C3 - path.C2);
        path.s1 = circleLength(sPoint, path.T1, sVec);
        path.s2 = circleLength(path.T1, path.T2, cross([0; 0; 1],(path.C3-path.C1)/norm(path.C3-path.C1)));
        path.s3 = circleLength(path.T2, gPoint, cross((path.C2 - path.C3)/norm(path.C2 - path.C3), [0; 0; 1]));
        path.length = path.s1 + path.s2 + path.s3;
    end
end

function path = RLR(sPoint, gPoint, sVec, gVec)
    path = DubinsPath;
    path.C1 = center(sPoint, sVec, 'r');
    path.C2 = center(gPoint, gVec, 'r');
    a = path.C2 - path.C1;
    if (norm(a)-2) > 2
        return;
    else
        A = path.C1 + 0.5*a;
        if norm(a) ~= 0
            w = sqrt(4 - 0.25*sqrt(sum(a.^2))^2)*cross([0; 0; 1], (a/sqrt(sum(a.^2))));
        else
            w = [0; 0; 0];
        end
        path.C3 = A + w;
        path.T1 = path.C1 + 0.5*(path.C3 - path.C1);
        path.T2 = path.C2 + 0.5*(path.C3 - path.C2);
        path.s1 = circleLength(sPoint, path.T1, sVec);
        path.s2 = circleLength(path.T1, path.T2, cross((path.C3-path.C1)/norm(path.C3-path.C1),[0; 0; 1]));
        path.s3 = circleLength(path.T2, gPoint, cross([0; 0; 1], (path.C2 - path.C3)/norm(path.C2 - path.C3)));
        path.length = path.s1 + path.s2 + path.s3; 
    end
   
end

function path = LSL(sPoint, gPoint, sVec, gVec)
    path = DubinsPath;
    path.C1 = center(sPoint, sVec, 'l');
    path.C2 = center(gPoint, gVec, 'l');
    a = path.C2 - path.C1;
    if norm(a) ~= 0
        e_a = a/norm(a);
    else
        % case that norm(a) == 0
        e_a = (gPoint - sPoint)/norm(gPoint - sPoint);
    end
    path.T1 = path.C1 + cross(e_a, [0; 0; 1]);
    path.T2 = path.T1 + a;
    path.s1 = circleLength(sPoint, path.T1, sVec);
    path.s2 = circleLength(path.T2, gPoint, e_a);
    path.length = norm(a) + path.s1 + path.s2;
end

function path = LSR(sPoint, gPoint, sVec, gVec)
    path = DubinsPath;
    path.C1 = center(sPoint, sVec, 'l');
    path.C2 = center(gPoint, gVec, 'r');
    a = path.C2 - path.C1;
    if norm(a) < 2
        return;
    else
        A = path.C1 + 0.5*a;
        path.T1 = Tangentpoint(path.C1, 'l', a);
        LSR = 2*(A - path.T1);
        path.T2 = path.T1 + LSR;
        path.s1 = circleLength(sPoint, path.T1, sVec);
        % case that starting point and path.T2 coincide
        if norm(a) ~= 2   
            path.s2 = circleLength(path.T2, gPoint, (LSR/(norm(LSR))));
        else
            path.s2 = circleLength(path.T2, gPoint, sVec/norm(sVec));
        end
        path.length = norm(LSR) + path.s1 + path.s2;
    end
end

function path = RSL(sPoint, gPoint, sVec, gVec)
   path = DubinsPath;
   path.C1 = center(sPoint, sVec, 'r');
   path.C2 = center(gPoint, gVec, 'l');
   a = path.C2 - path.C1;
   if norm(a) < 2
       return;
   else
       A = path.C1 + 0.5*a;
       path.T1 = Tangentpoint(path.C1, 'r', a);
       RSL = 2*(A - path.T1);
       path.T2 = path.T1 + RSL;
       path.s1 = circleLength(sPoint, path.T1, sVec);
       if norm(a) ~= 2
           path.s2 = circleLength(path.T2, gPoint, (RSL/(norm(RSL))));
       else
           path.s2 = circleLength(path.T2, gPoint, sVec/(norm(sVec)));
       end
       path.length = norm(RSL) +  path.s1 + path.s2;
       
   end
end

function path = RSR(sPoint, gPoint, sVec, gVec)
    path = DubinsPath;
    path.C1 = center(sPoint, sVec, 'r');
    path.C2 = center(gPoint, gVec, 'r');
    a = path.C2 - path.C1;
    if norm(a) ~= 0
        e_a = a/norm(a);
    else
        % case that norm(a) == 0
        e_a = (gPoint - sPoint)/norm(gPoint - sPoint);
    end
    path.T1 = path.C1 + cross([0; 0; 1], e_a);
    path.T2 = path.T1 + a;
    path.s1 = circleLength(sPoint, path.T1, sVec);
    path.s2 = circleLength(path.T2, gPoint, e_a);
    path.length = sqrt(sum(a.^2)) + path.s1 + path.s2;
end