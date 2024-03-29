%%%%%% Script %%%%%%

%% Movement parameters %%

TimeAcc = 0.25; %time for acceleration
MaxAcc = 140; %Maximum acceleration
MaxVel = TimeAcc * MaxAcc;

%% Start and End Positions %%

StartSer1 = 55;
StartSer2 = 70;
StartSer3 = 110;
CurrentPos = [StartSer1, StartSer2, StartSer3];

Goal = 1; %1 for "home", 2 for "extended", 3 for "to user" 

if Goal == 1
            GoalPos = [45, 90, 135];
elseif Goal == 2
            GoalPos = [0, 45, 90];
elseif Goal == 3
            GoalPos = [135, 90, 45];
else
        GoalPos = 'GoalPos has to be 1, 2 or 3';
end

%disp(GoalPos); %Check goal position

%% Angle restrictions%%

%Bottom and top angle limits for each servo%
Restrict1 = [45,315];   %servo1
Restrict2 = [1,100];     %servo2
Restrict3 = [1,270];     %servo3

%prevents from execution if a servo is NOT within the allowed range
if (CurrentPos(1) < Restrict1(1)) || (CurrentPos(1) > Restrict1(2))
    disp('warning1');
    return
elseif (CurrentPos(2) < Restrict2(1)) || (CurrentPos(2) > Restrict2(2))
   disp('warning2');
    return
elseif (CurrentPos(3) < Restrict3(1)) || (CurrentPos(3) > Restrict3(2))
    disp('warning3');
    return
end
%% Program%%

%Calculates the biggest movement 
PrimeJointDelta = DominantJointDelta(CurrentPos, GoalPos);

%Calculates the total time for all movements
Tf = timeTotal(PrimeJointDelta, TimeAcc, MaxAcc);

%Calculates the time for keeping constant velocity for all movements
VelTime = TimeConstantVel(Tf, TimeAcc);

%Acceleration calculation for each joint
[acc1,acc2,acc3] = Acc(CurrentPos, GoalPos, Tf, TimeAcc);
Accelerations = [acc1,acc2,acc3];

%Calculates the max angular velocity of each servo
[vel1,vel2,vel3] = omegavector(Accelerations, TimeAcc);
MaxVel = [vel1,vel2,vel3];

%Position at the end of acceleration part for each joint
[acb1, acb2, acb3] = AccelerationBlendEnd(Accelerations, TimeAcc, CurrentPos);
AccEndPos = [acb1, acb2, acb3];

%Position at the beginning of deceleration part for each joint
[dcb1, dcb2, dcb3] = DecelerationBlendStart(Accelerations, TimeAcc, AccEndPos, VelTime);
DecStartPos = [dcb1, dcb2, dcb3];

SampleTime =0.8;

%Get the position of the three servos at the above specified sampling time
[poss1,poss2,poss3] = SamplePos(Tf, Accelerations, TimeAcc, CurrentPos, VelTime, SampleTime, MaxVel);
SamplePosition = [poss1,poss2,poss3];

%Get the velocity of the three servos at the above specified sampling time
[vels1,vels2,vels3] = SampleVel(Tf, Accelerations, TimeAcc, VelTime, SampleTime);
SampleVelocity = [vels1,vels2,vels3];

%Get the acceleration of the three servos at the above specified sampling time
[accs1,accs2,accs3] = SampleAcc( Accelerations, TimeAcc, VelTime, SampleTime);
SampleAcceleration = [accs1,accs2,accs3];

%Plot the movements, the sampling position, velocity and acceleration and
%the borders between the parabolic blends and the linear segment

close all;
PlotVelocity(Accelerations, Tf, TimeAcc, SampleTime, SampleVelocity);
PlotPosition (Accelerations, Tf, TimeAcc, CurrentPos, MaxVel, GoalPos, SampleTime, SamplePosition,AccEndPos, DecStartPos);
PlotAcceleration(Accelerations, Tf, TimeAcc, SampleTime, SampleAcceleration);


%% Functions %%

%Determines the longest angular distance from all jojnts
function DominantDelta = DominantJointDelta(CurrentPos, GoalPos)
    for n = 1:3
   DeltaTheta(n) = abs(GoalPos(n) - CurrentPos(n));
    end
   DominantDelta = max(DeltaTheta);
end

%Determines the total time for all movements
function TotalTime = timeTotal(deltaTheta, timeAcc, maxAcc)
    if ((deltaTheta + maxAcc*timeAcc*timeAcc)/(maxAcc*timeAcc) > 2*timeAcc)
    TotalTime = (deltaTheta + maxAcc*timeAcc^2)/(maxAcc*timeAcc);
    else
    TotalTime = 2*timeAcc;    
    end
end

%Calculates the time for constant velocity
function Tvmax = TimeConstantVel(totalTime, timeAcc)
    if totalTime == 2*timeAcc
        Tvmax = 0;
    else 
        Tvmax = totalTime - 2*timeAcc;
    end  
end

%Calculates the accelerations of the servos and puts them into an array
function [x,y,z] = Acc(CurrentPos, GoalPos, Tf, TimeAcc)
Acceleration = [0,0,0];
     for m = 1:3
        DeltaTheta(m) = abs(GoalPos(m) - CurrentPos(m));    
        if Tf == 2*TimeAcc
            Acceleration(m) =  DeltaTheta(m)/(TimeAcc^2);
        else 
            Acceleration(m) =  DeltaTheta(m)/(TimeAcc*(Tf - TimeAcc));
        end
        if GoalPos(m) < CurrentPos(m)
            Acceleration(m) = -Acceleration(m);
        end 
     end 
    x = Acceleration(1);
    y = Acceleration(2);
    z = Acceleration(3);

end

%Puts the three velocities into an array
function [x,y,z] = omegavector(Accelerations, TimeAcc)
 for n = 1:3
     vector(n) = Accelerations(n)*TimeAcc;
 end
 x = vector(1);
 y = vector(2);
 z = vector(3);
 
end
%Calculates position at the end of acceleration period
function [x,y,z] = AccelerationBlendEnd(Accelerations, TimeAcc, CurrentPos)
    for n = 1:3
        Displacement(n) = abs(1/2*Accelerations(n)*TimeAcc^2);
        
        if Accelerations(n) > 0
            Position(n) = CurrentPos(n) + Displacement(n);
        else 
            Position(n) = CurrentPos(n) - Displacement(n);
        end
    end
    x = Position(1);
    y = Position(2);
    z = Position(3);
end
 
%Calculates position at the beginning of deceleration part
function [x,y,z] = DecelerationBlendStart(Accelerations, TimeAcc, AccEndPos, VelTime)
    for n = 1:3
        Displacement(n) = abs(Accelerations(n)*TimeAcc*VelTime);
        
        if Accelerations(n) > 0
            Position(n) = AccEndPos(n) + Displacement(n);
        else 
            Position(n) = AccEndPos(n) - Displacement(n);
        end
    end
    x = Position(1);
    y = Position(2);
    z = Position(3);
end

%Calculate and plot acceleration as a function of time, as well as sample acceleration
function PlotAcceleration (Accelerations, Tf, AccTime, SampleTime, SampleAcceleration)
syms Acc1(t);
syms Acc2(t);
syms Acc3(t);

Acc1(t) = piecewise(t<0, 0, 0<= t<AccTime, Accelerations(1), AccTime<=t<(Tf - AccTime), 0, (Tf - AccTime)<=t<Tf, -Accelerations(1), t>=Tf , 0);
Acc2(t) = piecewise(t<0, 0, 0<= t<AccTime, Accelerations(2), AccTime<=t<(Tf - AccTime), 0, (Tf - AccTime)<=t<Tf, -Accelerations(2), t>=Tf , 0);
Acc3(t) = piecewise(t<0, 0, 0<= t<AccTime, Accelerations(3), AccTime<=t<(Tf - AccTime), 0, (Tf - AccTime)<=t<Tf, -Accelerations(3), t>=Tf , 0);

figure(3)
fplot(Acc1); hold on;
fplot(Acc2); hold on;
fplot(Acc3); hold on;

plot(SampleTime, SampleAcceleration(1),'b*'); hold on;
plot(SampleTime, SampleAcceleration(2),'b*'); hold on;
plot(SampleTime, SampleAcceleration(3),'b*'); 

xlabel('Time, s')
hl = ylabel('Acceleration $\ddot{\theta}$, $deg/s^2$')
set(hl, 'Interpreter', 'latex');
axis([0 Tf -150 150])
end

%Calculate and plot velocity as a function of time, as well as sample velocity

function PlotVelocity(Accelerations, Tf, AccTime,SampleTime, SampleVelocity)

syms vel1(t);
syms vel2(t);
syms vel3(t);
    vel1(t) = piecewise(t<0, 0, 0<= t<AccTime, Accelerations(1)*t ,AccTime<=t<(Tf - AccTime), Accelerations(1)*AccTime, (Tf - AccTime)<=t<Tf, AccTime*Accelerations(1) - Accelerations(1)*(t-Tf+AccTime) ,t>=Tf , 0);
    vel2(t) = piecewise(t<0, 0, 0<= t<AccTime, Accelerations(2)*t ,AccTime<=t<(Tf - AccTime), Accelerations(2)*AccTime, (Tf - AccTime)<=t<Tf, AccTime*Accelerations(2) - Accelerations(2)*(t-Tf+AccTime) ,t>=Tf , 0);
    vel3(t) = piecewise(t<0, 0, 0<= t<AccTime, Accelerations(3)*t ,AccTime<=t<(Tf - AccTime), Accelerations(3)*AccTime, (Tf - AccTime)<=t<Tf, AccTime*Accelerations(3) - Accelerations(3)*(t-Tf+AccTime) ,t>=Tf , 0);

figure(1)
fplot(vel1); hold on;
fplot(vel2); hold on;
fplot(vel3); hold on;
plot(SampleTime, SampleVelocity(1),'b*'); hold on;
plot(SampleTime, SampleVelocity(2),'b*'); hold on;
plot(SampleTime, SampleVelocity(3),'b*');
xlabel('Time, s');
hl = ylabel('Angular velocity $\dot{\theta}$, deg / s');
set(hl, 'Interpreter', 'latex');
axis([0 Tf -60 60])
end

%%Calculate and plot position as a function of time, as well as sample position and linear segment borders
function PlotPosition (Accelerations, Tf, AccTime, CurrentPos, MaxVel, GoalPos, SampleTime, SamplePosition, AccEndBlend, DecStartBlend)

syms theta1(t);
syms theta2(t);
syms theta3(t);

theta1(t) = piecewise( t<0, CurrentPos(1), 0<= t<AccTime, CurrentPos(1) + 1/2*Accelerations(1)*t*t,AccTime<=t<(Tf - AccTime),CurrentPos(1) + 1/2*Accelerations(1)*(AccTime)^2 + MaxVel(1)*(t-AccTime),(Tf - AccTime)<=t<Tf, CurrentPos(1) + 1/2*Accelerations(1)*(AccTime)^2 + MaxVel(1)*(Tf-2*AccTime) + MaxVel(1)*(t-Tf+AccTime) - 1/2*Accelerations(1)*(t-Tf+AccTime)^2, t>=Tf , GoalPos(1));
theta2(t) = piecewise( t<0, CurrentPos(2), 0<= t<AccTime, CurrentPos(2) + 1/2*Accelerations(2)*t*t,AccTime<=t<(Tf - AccTime),CurrentPos(2) + 1/2*Accelerations(2)*(AccTime)^2 + MaxVel(2)*(t-AccTime),(Tf - AccTime)<=t<Tf, CurrentPos(2) + 1/2*Accelerations(2)*(AccTime)^2 + MaxVel(2)*(Tf-2*AccTime) + MaxVel(2)*(t-Tf+AccTime) - 1/2*Accelerations(2)*(t-Tf+AccTime)^2, t>=Tf , GoalPos(2));
theta3(t) = piecewise( t<0, CurrentPos(3), 0<= t<AccTime, CurrentPos(3) + 1/2*Accelerations(3)*t*t,AccTime<=t<(Tf - AccTime),CurrentPos(3) + 1/2*Accelerations(3)*(AccTime)^2 + MaxVel(3)*(t-AccTime),(Tf - AccTime)<=t<Tf, CurrentPos(3) + 1/2*Accelerations(3)*(AccTime)^2 + MaxVel(3)*(Tf-2*AccTime) + MaxVel(3)*(t-Tf+AccTime) - 1/2*Accelerations(3)*(t-Tf+AccTime)^2, t>=Tf , GoalPos(3));

figure(2)
fplot(theta1); hold on;
fplot(theta2); hold on;
fplot(theta3); hold on;
plot(SampleTime, SamplePosition(1),'b*'); hold on;
plot(SampleTime, SamplePosition(2),'b*'); hold on;
plot(SampleTime, SamplePosition(3),'b*'); hold on;

plot(AccTime, AccEndBlend(1),'r*'); hold on;
plot(AccTime, AccEndBlend(2),'r*'); hold on;
plot(AccTime, AccEndBlend(3),'r*'); hold on;

plot(Tf - AccTime, DecStartBlend(1),'r*'); hold on;
plot(Tf - AccTime, DecStartBlend(2),'r*'); hold on;
plot(Tf - AccTime, DecStartBlend(3),'r*'); 
xlabel('Time, s')
ylabel('Position \theta, deg')
axis([0 Tf 0 135])
end

%Calculate the position at a specific sample time
function [x,y,z] = SamplePos(Tf, Accelerations, TimeAcc, CurrentPos, VelTime, SampleTime, MaxVel)
   AccS =[0,0,0];
   VelS =[0,0,0];
   DecS =[0,0,0];
   sampos = [0,0,0];
       
    for n = 1:3
       if SampleTime <= TimeAcc
          AccS(n) =1/2*Accelerations(n)*SampleTime*SampleTime;
       else 
          AccS(n) = 1/2*Accelerations(n)*TimeAcc*TimeAcc;
       end
       
       if (VelTime >0) && (SampleTime <= VelTime + TimeAcc) && (SampleTime > TimeAcc)
           VelS(n) = Accelerations(n)*TimeAcc*(SampleTime - TimeAcc);
       elseif (VelTime > 0)&&(SampleTime > VelTime+TimeAcc)
           VelS(n) = Accelerations(n)*TimeAcc*VelTime;
       else
            VelS(n) = 0;
       end
          
          
       if SampleTime > Tf - TimeAcc
            DecS(n) = MaxVel(n)*(SampleTime-(Tf - TimeAcc))-1/2*(Accelerations(n))*(SampleTime - (Tf - TimeAcc))*(SampleTime - (Tf - TimeAcc));
       else 
           DecS(n) = 0;
       end
    
        sampos(n)= CurrentPos(n) + AccS(n) + VelS(n) + DecS(n);
    end

x = sampos(1);
y = sampos(2);
z = sampos(3);
end

%Calculate the velocity at a specific sample time
  function [x,y,z] = SampleVel(Tf, Accelerations, TimeAcc, VelTime, SampleTime)    
  vel = [0,0,0];
  for n = 1:3
     if SampleTime <= TimeAcc
         vel(n) = Accelerations(n)*SampleTime;
     elseif (VelTime > 0) && (SampleTime <= VelTime + TimeAcc) && (SampleTime > TimeAcc)
         vel(n) = Accelerations(n)*TimeAcc;
     else
         vel(n) = Accelerations(n)*TimeAcc - Accelerations(n)*(SampleTime - Tf + TimeAcc); 
     end
  end
  x =  vel(1);
  y =  vel(2);
  z =  vel(3);
  end
  
  %Calculate the acceleration at a specific sample time
  function [x,y,z] = SampleAcc(Accelerations, TimeAcc, VelTime, SampleTime)    
  acc = [0,0,0];
  for n = 1:3
     if SampleTime <= TimeAcc
         acc(n) = Accelerations(n);
     elseif (VelTime > 0) && (SampleTime <= VelTime + TimeAcc) && (SampleTime > TimeAcc)
         acc(n) = 0;
     else
         acc(n) = - Accelerations(n); 
     end
  end
  x =  acc(1);
  y =  acc(2);
  z =  acc(3);
   end
  
