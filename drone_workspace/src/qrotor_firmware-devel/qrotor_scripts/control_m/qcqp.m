function [xOpt,fval,eflag,output,lambda] = qcqp(Q, f, c, H, k, d, lb, ub, x0)
%% Quadtractically constrained quadratic programming
% cost: structure with fields Q, f, c
% const: structure with fields H, k, d 
% 
%%
options = optimoptions(@fmincon,'Algorithm','sqp',...
                        'Display', 'off',...
                        'SpecifyObjectiveGradient',true,...
                        'SpecifyConstraintGradient',true);

fun = @(x)quadobj(x, Q, f, c);
nonlconstr = @(x)quadconstr(x, H, k, d);

[xOpt,fval,eflag,output,lambda] = fmincon(fun,x0,...
        [], [], [], [], lb, ub, nonlconstr, options);

end  

%%
function [y,grady] = quadobj(x,Q,f,c)
    y = 1/2*x'*Q*x + f'*x + c;
    if nargout > 1
        grady = Q*x + f;
    end
end
function [y,yeq,grady,gradyeq] = quadconstr(x,H,k,d)
    
y = 1/2*x'*H*x + k'*x + d;
yeq = [];

if nargout > 2
    grady = H*x + k;
    gradyeq = [];
end
end
function hess = quadhess(x, lambda, Q, H)
    hess = Q;
	hess = hess + lambda.ineqnonlin(1)*H;
end



