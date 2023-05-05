function [exp_mat] = matrix_exp(mat)
    % matrix exponential
    I = eye(size(mat));

    % upto second order
    exp_mat =  I + mat + 0.5*mat*mat;
end
