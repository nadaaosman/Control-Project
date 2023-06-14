M1 = 100;  
M2 = 100;   
k1 = 5;    
k2 = 50;    
k3 = 5;    
f1 = 100;  
f2 = 100;   
u = 1;

B1 = tf(1, [M1 0 0]);
B2 = tf(k2, 1);
B3 = tf(1, [M2 0 0]);
B4 = tf(k2, 1);
B5 = tf(k2, 1);
B6 = tf(k1, 1);
B7 = tf([f1 0], 1);
B8 = tf(k3, 1);
B9 = tf(k2, 1);
B10 = tf([f2 0], 1);

BlockMat = append(B1, B2, B3, B4, B5, B6, B7, B8, B9, B10);
connect_map = [    1, -5, -6, -7,4;    2, 1, 0, 0, 0;    3, 2, -8, -9, -10;    4, 3, 0, 0, 0;    5, 1, 0, 0, 0;    6, 1, 0, 0, 0;    7, 1, 0, 0, 0;    8, 3, 0, 0, 0;    9, 3, 0, 0, 0;    10, 3, 0, 0, 0;];


%for requriment2

%for transfer function-->(x1/u)
system1 = connect(BlockMat, connect_map, 1, 1);
p0= stepplot(system1);
pzmap (system1);
[wn1,z1]=damp(system1);
X1_U = system1(1);
[num_1, den_1] = tfdata(X1_U, 'v');
fprintf('The value of X1/U transfer function is: \n');
fprintf('numerator: %s \n', mat2str(num_1));
fprintf('denominator: %s \n', mat2str(den_1));

%for requriment3

%check stability for transfer function-->(x1/u)
poles_X1_U = pole(X1_U);
if real(poles_X1_U) < 0
    disp('The system is (stable)')
else
    disp('The system is (unstable)')
end

%for requirment4

% Simulate the system response for X1 due to u=1
t = 0:0.01:110;
X1 = lsim(system1, u*ones(size(t)), t);
% Plot the response for X1
figure();
plot(t, X1);
xlabel('Time (s)');
ylabel('X1');
title('Response of X1 to u=1N');

%for transfer function-->(x2/u)
system2 = connect(BlockMat, connect_map, 1, 3);
figure();

p1= stepplot(system2);
pzmap(system2);
[wn2,z2]=damp(system2);
X2_U = system2(1);
[num_2, den_2] = tfdata(X2_U, 'v');
fprintf('The value of X2/U transfer function is: \n');
fprintf('numerator: %s \n', mat2str(num_2));
fprintf('denominator: %s \n', mat2str(den_2));

%check stability for transfer function-->(x2/u)
poles_X2_U = pole(X2_U);
if real(poles_X2_U) < 0
    disp('The system is (stable)')
else
    disp('The system is (unstable)')
end

% Simulate the response system for X2 due to u=1
X2 = lsim(system2, u*ones(size(t)), t);
figure();

plot(t, X2);
xlabel('Time (s)');
ylabel('X2');
title('Response of X2 u=1N');

%requriment 4

%Calculate the steady state values-->for x1,x2
X1_Steady.State = mean(X1(end-100:end));
X2_Steady.State = mean(X2(end-100:end));
fprintf('X1_steadystate is %f m\n', X1_Steady.State);
fprintf('X2_steadystate is %f m\n', X2_Steady.State);

%requriment_5
%requirment_6
%requirment_7

closedloop = feedback(system2, 1);

% Simulate the system for X1 for desired displacment=2m
t = 0:0.01:100;
Xd = ones(size(t))*2 ;
X2 = lsim(closedloop, Xd, t);

% Plot the response for x1
figure();

plot(t, X2);
xlabel('Time (s)');
ylabel('Displacement (m)');
title('Response of x2 to input xd=2m');
legend('X2', 'Xd');

% Calculate the rise time, peak time, maximum peak, settling time, and ess
disp('before using proportional controller')
step_info =stepinfo(closedloop*2);
risetime = step_info.RiseTime;
peaktime = step_info.PeakTime;
maxpeak = step_info.Peak;
settlingtime = step_info.SettlingTime;
ess = Xd(end) - X2(end);
%printing
fprintf('Rise Time: %.2f s\n', risetime);
fprintf('Peak Time: %.2f s\n', peaktime);
fprintf('Maximum Peak: %.2f\n', maxpeak);
fprintf('Settling Time: %.2f s\n', settlingtime);
fprintf('Ess: %.2f\n', ess);

%requriment8

%kp=1;
kp = 1;
% Define the controller-->transfer function
controller_block1 = tf(kp, 1);
closedloop0 = feedback(system2*controller_block1, 1);
figure();
p2= stepplot(closedloop0);
pzmap (closedloop0);
[wn3,z3]=damp(closedloop0);
tf1 = closedloop0(1);
% Simulate the system for X
X2 = lsim(closedloop0, Xd, t);
% Plot the response for x2
figure();

plot(t, X2);
xlabel('Time (s)');
ylabel('Displacement (m)');
title('Response of x2 to xd=2 at Kp=1');
legend('X2', 'Xd');
% Calculate the rise time, peak time, maximum peak, settling time, and ess
step_info =stepinfo(closedloop0*2);
rise_time = step_info.RiseTime;
peak_time = step_info.PeakTime;
max_peak = step_info.Peak;
settling_time = step_info.SettlingTime;
disp('kp=1')
fprintf('Rise Time: %.2f s\n', rise_time);
fprintf('Peak Time: %.2f s\n', peak_time);
fprintf('Maximum Peak: %.2f\n', max_peak);
fprintf('Settling Time: %.2f s\n', settling_time);
%calculate ess
ess = Xd(end) - X2(end);
fprintf('Ess: %.2f\n', ess);
%check stability
poles1= pole(tf1);
if real(poles1) < 0
    disp('The system is (stable)')
else
    disp('The system is (unstable)')
end

% at kp=10
kp=10;
controller_block2 = tf(kp, 1);
closedloop1 = feedback(system2*controller_block2, 1);
figure();

p3= stepplot(closedloop1);
pzmap (closedloop1);
[wn4,z4]=damp(closedloop1);
tf2 = closedloop1(1);
% Simulate the system for X2
X2 = lsim(closedloop1, Xd, t);
% Plot the responses
figure();
plot(t, X2);
xlabel('Time (s)');
ylabel('Displacement (m)');
title('Response of the X2 to xd=2 at Kp=10');
legend('X2', 'Xd');
% Calculate the rise time, peak time, maximum peak, settling time, and ess
step_info =stepinfo(closedloop1*2);
rise_time = step_info.RiseTime;
peak_time = step_info.PeakTime;
max_peak = step_info.Peak;
settling_time = step_info.SettlingTime;
disp('at kp=10)')
fprintf('Rise Time: %.2f s\n', rise_time);
fprintf('Peak Time: %.2f s\n', peak_time);
fprintf('Maximum Peak: %.2f\n', max_peak);
fprintf('Settling Time: %.2f s\n', settling_time);
%calculate ess
ess = Xd(end) - X2(end);
fprintf('Ess: %.2f\n', ess);
%check stability
poles2 = pole(tf2);
if real(poles2) < 0
    disp('The system is (stable)')
else
    disp('The system is (unstable)')
end


%at kp=100
kp=100;
controller_block3 = tf(kp, 1);
closedloop2 = feedback(system2*controller_block3, 1);
figure();
p4= stepplot(closedloop2);
pzmap (closedloop2);
[wn5 ,z5]=damp(closedloop2);
tf3 = closedloop2(1);
% Simulate the system
X3= lsim(closedloop2, Xd, t);
% Plot the responses
figure();
plot(t, X3);
xlabel('Time (s)');
ylabel('Displacement (m)');
title('the Response to xd=2m at Kp=100');
legend('X_3', 'Xd');
% Calculate the rise time, peak time, maximum peak, settling time, and ess
step_info =stepinfo(closedloop2*2);
rise_time = step_info.RiseTime;
peak_time = step_info.PeakTime;
max_peak = step_info.Peak;
settling_time = step_info.SettlingTime;
disp('at kp=100')
fprintf('Rise Time: %.2f s\n', rise_time);
fprintf('Peak Time: %.2f s\n', peak_time);
fprintf('Maximum Peak: %.2f\n', max_peak);
fprintf('Settling Time: %.2f s\n', settling_time);
%calculate ess
ess = Xd(end) - X2(end);
fprintf('Ess: %.2f\n', ess);
%check stability
poles3 = pole(tf3);
if real(poles3) < 0
    disp('The system is (stable)')
else
    disp('The system is (unstable)')
end

%at kp=1000
kp=1000;
controller_block3= tf(kp, 1);
closedloop3 = feedback(system2*controller_block3, 1);
figure();
p5= stepplot(closedloop3);
pzmap (closedloop3);
[wn6,z6]=damp(closedloop3);
tf4 = closedloop3(1);
% Simulate the system for X
X4 = lsim(closedloop3, Xd, t);
% Plot the responses
figure();
plot(t, X4);
xlabel('Time (s)');
ylabel('Displacement (m)');
title('the Response to xd=2m at Kp=1000');
legend('X4', 'Xd');
% Calculate the rise time, peak time, maximum peak, settling time, and ess
step_info =stepinfo(closedloop3*2);
rise_time = step_info.RiseTime;
peak_time = step_info.PeakTime;
max_peak = step_info.Peak;
settling_time = step_info.SettlingTime;
disp('at kp=1000')
fprintf('Rise Time: %.2f s\n', rise_time);
fprintf('Peak Time: %.2f s\n', peak_time);
fprintf('Maximum Peak: %.2f\n', max_peak);
fprintf('Settling Time: %.2f s\n', settling_time);
%calculate ess
ess = Xd(end) - X2(end);
fprintf('Ess: %.2f\n', ess);
%check stability
poles4 = pole(tf4);
if real(poles4) < 0
    disp('The system is (stable)')
else
    disp('The system is (unstable)')
end


%requriment9
% Simulate the system for X2
Xd2 = ones(size(t))*4; 
%after calculations 
kp = 4200;
controller_block4 = tf(kp, 1);
closedloop4 = feedback(system2*controller_block4, 1);
figure();
p6= stepplot(closedloop4);
pzmap (closedloop4);
[wn7,z7]=damp(closedloop4);
tf5 = closedloop4(1);
%check stability
poles5= pole(tf5);
disp('at kp=4200')
if real(poles5) < 0
    disp('The system is (stable)')
else
    disp('The system is (unstable)')
end
% Simulate the system for X
X5 = lsim(closedloop4, Xd2, t);
%calculate ess
ess= Xd2(end) - X5(end);
fprintf('Ess: %.2f\n', ess);
% Plot the responses
figure();
plot(t, X5);
xlabel('Time (s)');
ylabel('Displacement (m)');
title('the Response to xd=4m at Kp=400');
legend('X5', 'Xd');

%requriment 10

ki = 4;
kp=100;
%PI controller
controller_block5 = tf([kp ki], [1 0]);

% Connect the controller to the system
sys_connected_to_controller_5 = series(controller_block5, system2);

% Define the closed-loop transfer function
closedloop5 = feedback(sys_connected_to_controller_5, 1);
%check stability
figure();
p7= stepplot(closedloop5);
pzmap (closedloop5);
[wn8,z8]=damp(closedloop5);
tf6 = closedloop5(1);
%check stability
poles6= pole(tf6);
disp('at kp=4,ki=100')
if real(poles6) < 0
    disp('The system is (stable)')
else
    disp('The system is (unstable)')
end
% Simulate the system for X1
X6 = lsim(closedloop5, Xd2, t);
%calculate ess
ess = Xd2(end) - X6(end);
fprintf('Ess: %.2f\n', ess);
% Plot the responses
figure();
plot(t, X6);
xlabel('Time (s)');
ylabel('Displacement (m)');
title('the Response xd=4m at Kp=100 and ki=4');
legend('X5', 'Xd');


