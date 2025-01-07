function [t1,t2] = inverse_kinematics(r1,r2,x,y)
%t1,t2 is the angle, x,y is the position
% to return the desired reararm angle(t1)[deg] and forearm angle(t2)[deg] for PID to control motors
% input:
% x and y are desired position
% r1 is reararm lenth[mm]
% r2 is forearm lenth[mm]

% ensure -78<=x<=78 and 0<=y<=156, whcih is the size of the workspace
if y < 0
    error('Errors happen');
elseif y > 156
    error('Errors happen');
elseif x < -78
    error('Errors happen');
elseif x > 78
    error('Errors happen');
end

% calculate the angle[deg] of forearm
D = (x^2 + y^2 - r1^2 - r2^2) / (2 * r1 * r2); 
if abs(D) > 1
    error('Errors happen');
end
t2 = acosd(D); 
t2 = max(-180,min(t2,180)); % ensure t2 is between -180 and 180

% judge the posture of arm
if x >= 0
    t2 = -abs(t2); % if desired position is in the first quartile, use minus value of t2
    if (-90 <= t2) && (t2 <=0)
        % calculate the angle[deg] of reararm
        if x == 0
            x = x+0.0000001; % avoidi x == 0 when y/x
        else
            % pass
        end
        k1 = y/x;
        k2 = abs(r2*sind(t2))/(r1+abs(r2*cosd(t2)));
        t1 = atand(k1)+atand(k2);
        t1 = max(0,min(t1,180)); % ensure t1 is between 0 and 180
    else
        k1 = y/x;
        k2 = abs(r2*sind(t2))/(r1-abs(r2*cosd(t2)));
        t1 = atand(k1)+atand(k2);
        t1 = max(0,min(t1,180)); % ensure t1 is between 0 and 180
    end
else
    t2 = abs(t2); % if desired position is in the second quartile, use plus value of t2
    if (0 <= t2) && (t2 <= 90)
        % calculate the angle[deg] of reararm
        k1 = y/x;
        k2 = abs(r2*sind(t2))/(r1+abs(r2*cosd(t2)));
        t1 = atand(k1)+180-atand(k2); 
        t1 = max(0,min(t1,180)); % ensure t1 is between 0 and 180
    else
        % calculate the angle[deg] of reararm
        k1 = y/x;
        k2 = abs(r2*sind(t2))/(r1-abs(r2*cosd(t2)));
        t1 = atand(k1)+180-atand(k2); 
        t1 = max(0,min(t1,180)); % ensure t1 is between 0 and 180
    end
end
