% close all;
clear;
close all
% clc;

%% set environment
addpath('../trajectories/');
addpath('./controllers/');

params = struct();
params.JQ  =  diag([0.0483333 , 0.0483333 , 0.0833333]);
params.mQ = 2;
sys = QrotorHardwareSimulation(params);

sys.controller = SO3VariationMPC();
sys.controller.mQ = params.mQ;
sys.controller.updateInertia(params.JQ);

sys.controller.trajectory = @setpoint;
% sys.controller.trajectory = @circular_trajectory;
sys.controller.flat2state = @qflat2state;

%% simulation paramters

t0 = 0;
tf = 5;


%%
% initial condition
xQ0 = 1*[0.5; 0.5; 0.5];
vQ0 = zeros(3,1);
R0 = RPYtoRot_ZXY(20*pi/180, 0*pi/180, 0*pi/180);
% R0 = RPYtoRot_ZXY(0*pi/180, 0*pi/180, 0*pi/180);
Om0 = zeros(3,1);

x0 = [xQ0; vQ0; reshape(R0,9,1); Om0];

% 
sys.include_noise = 0;

% noise covariances
sys.xVar = diag([0.25e-9, 0.25e-9, 0.25e-9]);
sys.vVar = diag([0.25e-6, 0.25e-6, 0.25e-6]);
sys.eVar = diag([0.25e-6, 0.25e-6, 0.25e-6]);
sys.bVar = diag([0.000012341280121e-6, 0.000013413255965e-6, 0.000002635158647e-6]);

% R = blkdiag([xVar, vVar, eVar, bVar]);


%% simulation

tic
[log] = sys.discrete_simulate([t0, tf], x0, @ode15s);
%  [log] = sys.continuous_simulate([t0, tf], x0, @ode15s);
toc

%% animation

opts.data.t = log.tTOTAL';
opts.data.x = log.xLog.actual';
opts.data.td = log.tTOTAL';
opts.data.xd = log.xLog.ideal';

% animateQuadrotor(opts);

        
%% plots
% close all
ind = round(linspace(1, length(log.tTOTAL), round(1*length(log.tTOTAL)))) ;

close all

figure('name', 'inputs');
subplot(2,2,1);
hold on;
stairs(log.tTOTAL(ind), log.uLog.ideal(1,ind),'-r','linewidth',2);
stairs(log.tTOTAL(ind), log.uLog.actual(1,ind),'-b','linewidth',1);
l=plot(log.tTOTAL(ind), sys.fmin*ones(1,length(ind)),'k','linewidth',2);l.Color(4)=0.4;
l=plot(log.tTOTAL(ind), sys.fmax*ones(1,length(ind),1),'k','linewidth',2);l.Color(4)=0.4;
latex_title('$$f$$');
latex_legend({'ideal','actual'});
grid on;

subplot(2,2,2);
hold on;
stairs(log.tTOTAL(ind), log.uLog.ideal(2,ind),'-r','linewidth',2);
stairs(log.tTOTAL(ind), log.uLog.actual(2,ind),'-b','linewidth',1);
l = stairs(log.tTOTAL(ind), sys.Mmin(1)*ones(1,length(ind)),'k','linewidth',2); l.Color(4)=0.4;
l = stairs(log.tTOTAL(ind), sys.Mmax(1)*ones(1,length(ind)),'k','linewidth',2); l.Color(4)=0.4;
latex_title('$$M_x$$');
latex_legend({'ideal','actual'});
grid on;
subplot(2,2,3);
hold on;
stairs(log.tTOTAL(ind),log.uLog.ideal(3,ind),'-r','linewidth',2);
stairs(log.tTOTAL(ind),log.uLog.actual(3,ind),'-b','linewidth',1);
l = plot(log.tTOTAL(ind), sys.Mmin(2)*ones(1,length(ind)),'k','linewidth',2); l.Color(4)=0.4;
l = plot(log.tTOTAL(ind), sys.Mmax(2)*ones(1,length(ind)),'k','linewidth',2); l.Color(4)=0.4;
latex_title('$$M_y$$');
latex_legend({'ideal','actual'});
grid on;
subplot(2,2,4);
hold on;
stairs(log.tTOTAL(ind),log.uLog.ideal(4,ind),'-r','linewidth',2);
stairs(log.tTOTAL(ind),log.uLog.actual(4,ind),'-b','linewidth',1);
l = plot(log.tTOTAL(ind), sys.Mmin(3)*ones(1,length(ind)),'k','linewidth',2); l.Color(4)=0.4;
l = plot(log.tTOTAL(ind), sys.Mmax(3)*ones(1,length(ind)),'k','linewidth',2); l.Color(4)=0.4;
latex_title('$$M_z$$');
latex_legend({'ideal','actual'});
grid on;


figure; 
subplot(3,1,1);
hold on;
plot(log.tTOTAL(ind), log.xLog.ideal(1,ind),'r','linewidth',2);
plot(log.tTOTAL(ind), log.xLog.actual(1,ind),'b','linewidth',1);
latex_title('x');
latex_legend({'ideal','actual'});
grid on;
subplot(3,1,2);
hold on;
plot(log.tTOTAL(ind), log.xLog.ideal(2,ind),'r','linewidth',2);
plot(log.tTOTAL(ind), log.xLog.actual(2,ind),'b','linewidth',1);
latex_title('y');
latex_legend({'ideal','actual'});
grid on;
subplot(3,1,3);
hold on;
plot(log.tTOTAL(ind), log.xLog.ideal(3,ind),'r','linewidth',2);
plot(log.tTOTAL(ind), log.xLog.actual(3,ind),'b','linewidth',1);
latex_title('z');
latex_legend({'ideal','actual'});
grid on;


figure;
subplot(3,1,1);
hold on;
plot(log.tTOTAL(ind),log.eulerLog.ideal(1,ind)*180/pi,'-r','linewidth',2);
plot(log.tTOTAL(ind),log.eulerLog.actual(1,ind)*180/pi,'-b','linewidth',1);
latex_title('roll');
latex_legend({'ideal','actual'});
grid on;

subplot(3,1,2);
hold on;
plot(log.tTOTAL(ind),log.eulerLog.ideal(2,ind)*180/pi,'-r','linewidth',2);
plot(log.tTOTAL(ind),log.eulerLog.actual(2,ind)*180/pi,'-b','linewidth',1);
latex_title('pitch');
latex_legend({'ideal','actual'});
grid on;
subplot(3,1,3);
hold on;
plot(log.tTOTAL(ind),log.eulerLog.ideal(3,ind)*180/pi,'-r','linewidth',2);
plot(log.tTOTAL(ind),log.eulerLog.actual(3,ind)*180/pi,'-b','linewidth',1);
latex_title('yaw');
latex_ylabel('[deg]');
latex_legend({'ideal','actual'});
grid on;

