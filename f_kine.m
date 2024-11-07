
function [T,O] = f_kine(n,J,alpi_1,ai_1,d,q,th)

% Forward kinematics by using Modified DH parameters according to the book
% of craig
Tmp_mat = eye(4);   % Temporary matrix 
T =([]);
O = ([]);
for i = 1:n
    th_i(i)=J(i)*th(i)+(1-J(i))*q(i);
    d_i(i)=(1-J(i))*d(i)+J(i)*q(i);
    T_ii_1  =  [cos(th_i(i))                      -sin(th_i(i))                  0                   ai_1(i);
                sin(th_i(i))*cos(alpi_1(i))   cos(th_i(i))*cos(alpi_1(i))   -sin(alpi_1(i))     -d_i(i)*sin(alpi_1(i));
                sin(th_i(i))*sin(alpi_1(i))   cos(th_i(i))*sin(alpi_1(i))    cos(alpi_1(i))      d_i(i)*cos(alpi_1(i));
                0                                  0                            0                       1]; 
    T(:,:,i) = Tmp_mat*T_ii_1;
    Tmp_mat = T(:,:,i);
    O(:,i) = T((1:3),4,i);
end
