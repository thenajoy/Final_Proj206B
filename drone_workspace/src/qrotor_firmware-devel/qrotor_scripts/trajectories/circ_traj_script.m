% CIRC_TRAJ_SCRIPT   Generate static library circ_traj from circ_traj.
% 
% Script generated from project 'circ_traj.prj' on 27-Apr-2021.
% 
% See also CODER, CODER.CONFIG, CODER.TYPEOF, CODEGEN.

%% Create configuration object of class 'coder.EmbeddedCodeConfig'.
cfg = coder.config('lib','ecoder',true );
cfg.GenerateReport = true;
cfg.ReportPotentialDifferences = false;
cfg.GenCodeOnly = true;
cfg.TargetLang = 'C++';
% coder.config(numeric_conversion_type)

%% Define argument types for entry-point 'circ_traj'.
ARGS = cell(1,1);
ARGS{1} = cell(5,1);
ARGS{1}{1} = coder.typeof(0);
ARGS{1}{2} = coder.typeof(0,[3 1]);
ARGS{1}{3} = coder.typeof(0,[3 1]);
ARGS{1}{4} = coder.typeof(0,[3 1]);
ARGS{1}{5} = coder.typeof(0,[3 1]);

%% Invoke MATLAB Coder.
cd('/home/kotaru/Workspace/catkin_ws/qrotor_ws/src/qrotor_firmware/qrotor_firmware/scripts/trajectories/autogen');
codegen -config cfg circ_traj -args ARGS{1}

