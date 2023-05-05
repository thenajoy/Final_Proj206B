function animate(quads)

    RATE = 25 * 1;
    %==========================================
    % initialize the animation figure and axes
    %==========================================
    figure_x_limits = [-5 5];
    figure_y_limits = [-5 5];
    figure_z_limits = [-5 5] ;
    fig1 = figure;

    set(0,'Units','pixels')
    scnsize = get(0,'ScreenSize');

    screen_width = scnsize(3);
    screen_height = scnsize(4);

    % find the minimum scaling factor
    figure_x_size = figure_x_limits(2) - figure_x_limits(1);
    figure_y_size = figure_y_limits(2) - figure_y_limits(1);

    xfactor = screen_width/figure_x_size;
    yfactor = screen_height/figure_y_size;

    if (xfactor < yfactor)
      screen_factor = 0.5*xfactor;
    else
      screen_factor = 0.5*yfactor;
    end

    % calculate screen offsets
    screen_x_offset = (screen_width - screen_factor*figure_x_size)/2;
    screen_y_offset = (screen_height - screen_factor*figure_y_size)/2;

    % draw figure and axes
    set(fig1,'Position', [screen_x_offset screen_y_offset screen_factor*figure_x_size screen_factor*figure_y_size]);
    set(fig1,'MenuBar', 'none');
    axes1 = axes;
    set(axes1,'XLim',figure_x_limits,'YLim',figure_y_limits);
    %     set(axes1,'Position',[0 0 1 1]);
    %     set(axes1,'Color','w');
    %set(axes1,'TickDir','out');
    axis equal ;
    box on;
    hist = 250 ;
    
    %% post-process data
    
    
    
    %% animate

    for i=1:length(t)
        % delete current figure        
        tem = get(axes1, Children');
        delete(tem);
        % draw quadrotors
        
        
        % draw payloads
        
        drawQuadrotor(axes1, x(i,:)');
        plot3(x(max(1,i-hist):i, 1), x(max(1,i-hist):i, 2), x(max(1,i-hist):i, 3), 'b') ;
        plot3(xLd(max(1,i-hist):i, 1), xLd(max(1,i-hist):i, 2), xLd(max(1,i-hist):i, 3), 'r') ;
        %         plot3(xorig(:, 1), xorig(:, 2), xorig(:, 3), 'b') ;
        %         plot3(xLdorig(:, 1), xLdorig(:, 2), xLdorig(:, 3), 'r') ;
        %         s = sprintf('Running\n t = %1.2fs \n 1/%d realtime speed',t(i), RATE/25);
        %         text(-1.3,2.4,s,'FontAngle','italic','FontWeight','bold');
        drawnow;
        set(axes1,'XLim',figure_x_limits,'YLim',figure_y_limits,'ZLim',figure_z_limits);
    end
end



function drawQuadrotor(xQ, R,varargin)
    s.L = 0.1; %length of quadrotor boom
    s.R = 0.065; %radius of propeller prop

    BRW = R' ;
    point1 = BRW'*[s.L,0,0]';
    point2 = BRW'*[0,s.L,0]';
    point3 = BRW'*[-s.L,0,0]';
    point4 = BRW'*[0,-s.L,0]';

    p1 = BRW'*[s.L,0, 0.05]';
    p2 = BRW'*[0,s.L, 0.05]';
    p3 = BRW'*[-s.L,0, 0.05]';
    p4 = BRW'*[0,-s.L, 0.05]';

    nprop = 40;
    propangs = linspace(0,2*pi,nprop);
    proppts = s.R*BRW'*[cos(propangs);sin(propangs);zeros(1,nprop)];

    wp = xQ ;
    wp1 = [wp + point1, wp + p1]  ;
    wp2 = [wp + point2, wp + p2];
    wp3 = [wp + point3, wp + p3];
    wp4 = [wp + point4, wp + p4];


    prop1 = proppts + wp1(:,end)*ones(1,nprop);
    prop2 = proppts + wp2(:,end)*ones(1,nprop);
    prop3 = proppts + wp3(:,end)*ones(1,nprop);
    prop4 = proppts + wp4(:,end)*ones(1,nprop);

    lwp = 2 ;
    lwq = 1 ;
    lwc = 2 ;
    lwl = 2 ;

    s.qhandle1 = line([wp1(1,end), wp1(1,1), wp3(1,1), wp3(1,end)],...
                        [wp1(2,end), wp1(2,1), wp3(2,1), wp3(2,end)],...
                        [wp1(3,end), wp1(3,1), wp3(3,1), wp3(3,end)]); hold on ;
    s.qhandle2 = line([wp2(1,end), wp2(1,1), wp4(1,1), wp4(1,end)],...
                        [wp2(2,end), wp2(2,1), wp4(2,1), wp4(2,end)],...
                        [wp2(3,end), wp2(3,1), wp4(3,1), wp4(3,end)]); hold on ;
    set(s.qhandle1,'Color','k', 'LineWidth',lwq);
    set(s.qhandle2,'Color','k', 'LineWidth',lwq);
    scatter3([wp1(1,end), wp2(1,end), wp3(1,end), wp4(1,end)],...
                [wp1(2,end), wp2(2,end), wp3(2,end), wp4(2,end)],...
                [wp1(3,end), wp2(3,end), wp3(3,end), wp4(3,end)],5,'k','filled');

    s.hprop1 = fill3(prop1(1,:),prop1(2,:),prop1(3,:),'r','FaceAlpha', 0.6);
    s.hprop2 = fill3(prop2(1,:),prop2(2,:),prop2(3,:),'b','FaceAlpha', 0.6);
    s.hprop3 = fill3(prop3(1,:),prop3(2,:),prop3(3,:),'b','FaceAlpha', 0.6);
    s.hprop4 = fill3(prop4(1,:),prop4(2,:),prop4(3,:),'b','FaceAlpha', 0.6);

end




function [Et, Ex] = even_sample(t, x, Fs, type)
if nargin < 4, type = 'linear'; end

dt = diff(t);
dt = dt + (dt==0)*1e-5;
t = [0;cumsum(dt)];

% Obtain the process related parameters
N = size(x, 2);    % number of signals to be interpolated
M = size(t, 1);    % Number of samples provided
t0 = t(1,1);       % Initial time
tf = t(M,1);       % Final time
EM = (tf-t0)*Fs;   % Number of samples in the evenly sampled case with
                   % the specified sampling frequency
Et = linspace(t0, tf, round(EM))';
% Using linear interpolation (used to be cubic spline interpolation)
% and re-sample each signal to obtain the evenly sampled forms
    for s = 1:N
      Ex(:,s) = interp1(t(:,1), x(:,s), Et(:,1),type); 
    end
end    




    