%% Create a multi-robot environment
clear all;
n = 7;
p = 13;
env = MultiRobotEnv(n);
env.robotRadius = 0.4;
env.robotColors = [0 0 1];
%%  Define Target
m = 13;
target = randi([15 35],m,4); % 3 is the banchor and 4 is detected
target(1,1) = 16;
target(1,2) = 18;
count = 1;
while count <= m
    numberTarget = randi([1 2]);
    if count == m-1
        numberTarget = 1;
    end
    i = 1;
    while i<= numberTarget
        distanceMax = randi([4 5]);
        angle = randi([0 360]);
        if angle <= 90
            target(count+1,1) = target(count,1) + distanceMax*sin(angle);
            target(count+1,2) = target(count,2) + distanceMax*cos(angle);
        elseif angle >90 && angle <= 180
            angle = 180-angle;
            target(count+1,1) = target(count,1) + distanceMax*sin(angle);
            target(count+1,2) = target(count,2) - distanceMax*cos(angle);
        elseif angle >180 && angle <= 270
            target(count+1,1) = target(count,1) - distanceMax*sin(angle);
            target(count+1,2) = target(count,2) - distanceMax*cos(angle);
        else
            angle = 360 - angle;
            target(count+1,1) = target(count,1) - distanceMax*sin(angle);
            target(count+1,2) = target(count,2) + distanceMax*cos(angle);
        end
        bool_ = 0;
        for preTarget = 1:count
            target(preTarget,1:2);
            target(count+1,1:2);
            distanceMin = norm(target(preTarget,1:2)-target(count+1,1:2));
            if distanceMin < 4 || target(count+1,1) > 40 || target(count+1,2) > 40 || target(count+1,1) < 20 || target(count+1,2) < 20
                bool_ = 1;
            end
        end
        if bool_ == 0
            count = count + 1;
            i = i + 1;
        end
    end
    if count == m
        count = m+1;
    end
end
target(:,3:4) = 0;
%%  Define of the multi-robot system
A = randi([2 9],n,4); % Positive
for i = 1:1:n
    A(i,1) = randi([1 12]);
    A(i,2) = randi([1 12]);
    for j = 1:1:n
        g = norm(A(i,1:2) - A(j,1:2))
        if i ~= j && g <= 0.7
            j = n+1;
            i = i-1;
        end
    end
end
Va = zeros(n,3);    % velocity alignment of Robots
V = zeros(n,3);     % velocity of Robots
Vmax = 1;
Ra = 0.5;    % Radius Sa:  the obstacle avoidance area
Rn = 5;      % Radius Sn:  the noncritical area
Rc = 6;      % Radius Sc:  the critical area
Rdetect = 5; % Radius Sensor:  the detect target
distanceRvT = 99;
posTarget = 0;
diChuyen = 0;
point = randi([10 20],1,2);
distanceObs = zeros(n,p);
dem = 0;
stt = zeros(n,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Motion of Robots
env.Poses = transpose(A);
while 1
   %% Detect distance min of Robots
       env([1:n],transpose(A));
       axis([0 40 0 40]);
       hold on
       a = plot(target(:,1),target(:,2),'m.');
       b = plot(50,50,'bo');
       %legend([a b],{'target','robot'})
       %plot(B(:,1),B(:,2),'g*');
       hold off
       %% check Robot and Target
       A(:,3:4) = 0;
       for i = 1:1:n
           for j= 1:1:m
               if target(j,3) == 1
                   for k= 1:1:n
                       if norm(target(j,1:2) - A(k,1:2)) <= 0.05
                           A(k,3) = 1;
                           A(k,4) = 1;
                           target(j,3) = 1;
                           diChuyen = 1;
                       end
                   end
               end
               if target(j,3) == 0
                   if norm(target(j,1:2) - A(i,1:2)) <= 0.05
                       A(i,3) = 1;
                       A(i,4) = 1;
                       target(j,3) = 1;
                       diChuyen = 1;
                       dem = dem + 1
                       stt(dem) = i
                   end
                   if norm(target(j,1:2) - A(i,1:2)) <= 5.4
                       if target(j,3) == 0
                           target(j,4) = 1;
                           A(i,4) = 1;
                           diChuyen = 1;
                       end
                   end
               end
           end
       end
       temp = 6;
       temp = dem -temp;
       if temp > 7
           temp = dem - 2*temp
       end
       if temp > 0 && temp < 7
           u = stt(temp);
           A(u,3) = 0;
           A(u,4) = 0;
           for p = 1:1:m
               if target(p,3) == 0
                   if norm(target(p,1:2) - A(u,1:2)) <= 0.05
                       A(u,3) = 1;
                       A(u,4) = 1;
                       target(p,3) = 1;
                   end
                   if norm(target(p,1:2) - A(u,1:2)) <= 5.4
                       if target(p,3) == 0
                           target(p,4) = 1;
                           A(u,4) = 1;
                       end
                   end
               end
           end
       end
       if diChuyen == 0
           temp1 = [40 40];
           for i = 1:1:n
               Va(i,1:2) = (temp1 - A(i,1:2));
           end
       elseif diChuyen == 1
           for i = 1:1:n
               if A(i,3) == 0
                    %% Execute Free Robot
                    if A(i,4) == 0
                        for j = 1:1:n
                            if A(j,3) == 1
                                for k = 1:1:m
                                    if target(k,3) == 0
                                        disNagevative = norm(target(k,1:2) - A(j,1:2));
                                        if disNagevative <= 5.2
                                            if A(j,1) < target(k,1)
                                                point(1) = A(j,1) + 0.5;
                                            elseif A(j,1) >= target(k,1)
                                                point(1) = A(j,1) - 0.5;
                                            end
                                            if A(j,2) < target(k,2)
                                                point(2) = A(j,2) + 0.5;
                                            elseif A(j,1) >= target(k,1)
                                                point(2) = A(j,2) - 0.5;
                                            end
                                            distanceRvR = norm(A(j,1:2) - A(i,1:2));
                                            Va(i,1:2) = (A(j,1:2) - A(i,1:2))/distanceRvR;
                                            if distanceRvR < 1.5
                                                distanceRvR = norm(point(1:2) - A(i,1:2));
                                                Va(i,1:2) = (point(1:2) - A(i,1:2))/distanceRvR;
                                            end
                                        end
                                    end
                                end
                            end
                        end
                    %% Execute Robot detected
                    elseif A(i,4) == 1
                        disMin = 99;
                        for j= 1:1:m
                            if target(j,3) == 0 
                                disRvT = norm(target(j,1:2) - A(i,1:2));
                                if disRvT <= 5.4 && disRvT < disMin
                                    disMin = disRvT;
                                    check = j;
                                end
                            end
                        end
                        Va(i,1:2) = (target(check,1:2) - A(i,1:2))/disMin;
                    end
               end
           end
       end
       Vs = zeros(n,3); % Matrix velocity separation of Robots
       Vc = zeros(n,3); % Matrix velocity cohesion of Robots
       for i=1:n
            for j=1:n
                if A(j,3) == 0
                    r(i,j) = norm(A(i,1:2)-A(j,1:2))-2*0.35;
                    %%%%%%%%% calculate vector Vs %%%%%%%%%%%
                    if r(i,j) <= Ra
                        Vs(i,1:2) = Vs(i,1:2)-exp(-2*(r(i,j)-Ra))*(A(j,1:2)-A(i,1:2))/r(i,j);
                    elseif r(i,j) > Ra && r(i,j) < Rn
                        Vs(i,1:2) = Vs(i,1:2);
                        Vc(i,1:2) = Vc(i,1:2);
                    %%%%%%%%% calculate vector Vc %%%%%%%%%%%
                    elseif  r(i,j) > Rn && A(j,3) == 0%&& r(i,j) < Rc
                        Vc(i,1:2) = Vc(i,1:2) + A(j,1:2) - A(i,1:2); 
                    end
                elseif A(j,3) == 1
                    r(i,j) = norm(A(i,1:2)-A(j,1:2))-2*0.35;
                    %%%%%%%%% calculate vector Vs %%%%%%%%%%%
                    if r(i,j) <= Ra
                        Vs(i,1:2) = Vs(i,1:2)-exp(-2*(r(i,j)-Ra))*(A(j,1:2)-A(i,1:2))/r(i,j);
                    elseif r(i,j) > Ra && r(i,j) < Rn
                        Vs(i,1:2) = Vs(i,1:2);
                    end
                    Vc(i,1:2) = Vc(i,1:2);
                end
            end
            %%%%%%%%% calculate vector Vs - Obs %%%%%%%%%%%
            %for m=1:p
            %    distanceObs(i,m) = norm(A(i,1:2)-B(m,1:2))-0.35;
             %   if distanceObs(i,m) <= Ra
             %       Vs(i,1:2) = Vs(i,1:2)-exp(-50*(distanceObs(i,m)-Ra))*(B(m,1:2)-A(i,1:2))/distanceObs(i,m);
              %      Vs(i,1:2) = Vs(i,1:2) - (B(m,1:2) - A(i,1:2));
              %  elseif r(i,j) > Ra
              %      Vs(i,1:2) = Vs(i,1:2);
               % end
           % end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       end
       for v = 1:1:n
           for b= 1:1:m
               if norm(target(b,1:2) - A(v,1:2)) <= 0.5
                   Vs(v,1:2) = 0;
               end 
           end
           if A(v,4) == 1
               Vc(v,1:2) = 0;
           end
       end
       for k=1:n
            if A(k,3) == 0
                V = 3*Va(k,1:2) + Vc(k,1:2) + Vs(k,1:2);
                if norm(V) > Vmax
                    V = (V/norm(V)) * Vmax;
                end
                A(k,1:2)  = A(k,1:2) + V*0.07;
            end
       end
end