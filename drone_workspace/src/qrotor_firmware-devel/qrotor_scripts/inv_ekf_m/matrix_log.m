function [log_mat] = matrix_log(mat)
    % matrix logarithm
    I = eye(size(mat));

    % upto second order
    log_mat = (mat-I)-0.5*(mat-I)*(mat-I);
end