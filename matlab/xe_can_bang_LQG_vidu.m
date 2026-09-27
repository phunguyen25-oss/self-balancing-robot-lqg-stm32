%% VI DU DAY DU: LQG cho xe can bang 2 banh (theo 10 buoc trong lo trinh)
% Mo hinh con lac nguoc tren xe (cart-pole), anh xa sang xe 2 banh:
%   - "xe" tinh tien  <->  truc 2 banh (dong bo, khong xet yaw)
%   - "con lac"       <->  than xe (khung + pin + mach)
%   - u (luc ngang)   <->  tong luc keo tuong duong tu 2 dong co
%                          (u = tong mo-men banh / ban kinh banh)
% CANH BAO: cac thong so vat ly ben duoi la SO MINH HOA, phai thay bang
% so do that tren xe cua ban truoc khi tin tuong ket qua.
% (Da sua loi dau o A(2,3): he so cua theta trong phuong trinh xddot
%  phai la SO AM, kiem lai bang cach nghich dao ma tran khoi luong.)
clear; clc; close all;

%% ---- Thong so vat ly (thay bang so do thuc te) ----
M = 0.20;    % kg, khoi luong quy doi cua banh xe + truyen dong
m = 0.80;    % kg, khoi luong than xe (khung+pin+mach), KHONG tinh banh
l = 0.10;    % m,  khoang cach truc banh -> trong tam than
I = 0.0050;  % kg*m^2, mo-men quan tinh than quanh trong tam cua no
b = 0.02;    % N*s/m, ma sat nhot tai truc banh (quy doi qua ban kinh)
g = 9.81;    % m/s^2

%% ===== GIAI DOAN 1 — Buoc 1-2: ODE tuyen tinh hoa + State-Space =====
% State: x = [vi tri x; van toc x_dot; goc nghieng theta; toc do goc theta_dot]
p = I*(M+m) + M*m*l^2;

A = [0            1                    0                  0;
     0   -b*(I+m*l^2)/p     -(g*l^2*m^2)/p                0;
     0            0                    0                  1;
     0      b*l*m/p          g*l*m*(M+m)/p                0];

B = [0; (I+m*l^2)/p; 0; -l*m/p];

disp('A ='); disp(A);
disp('B ='); disp(B);
disp('eig(A) vong ho (khong dieu khien):'); disp(eig(A).');
% -> co 1 cuc dung tai 0 (trang thai x la tich phan thuan tuy, khong
%    xuat hien o bat ky phuong trinh dao ham nao khac) + 1 cap cuc that
%    +-omega (theta khong on dinh, dung nhu ban chat con lac nguoc)

%% ===== GIAI DOAN 2 — Buoc 3: Kiem tra dieu khien duoc =====
Co = ctrb(A, B);
fprintf('rank(Ctrb) = %d / 4\n', rank(Co));   % phai = 4

%% ===== Buoc 4: Kiem tra quan sat duoc =====
% Thu chi do theta (IMU) truoc:
C_theta = [0 0 1 0];
fprintf('rank(Obsv, chi theta) = %d / 4  <-- KHONG DU HANG!\n', rank(obsv(A, C_theta)));
% Ly do vat ly: x khong xuat hien trong bat ky phuong trinh dong luc nao
% (chi x_dot va theta moi anh huong lan nhau) -> chi do goc thi KHONG
% THE suy ra vi tri tuyet doi x. Day chinh la ly do BOM cua ban co
% 2x AS5600 (encoder banh): goc banh * ban kinh banh = x truc tiep.

C = [1 0 0 0;      % x     do tu encoder AS5600 (tich phan/quy doi tu goc banh)
     0 0 1 0];     % theta do tu IMU MPU6050 (fusion accel+gyro)
D = [0; 0];
fprintf('rank(Obsv, x+theta) = %d / 4  <-- du hang\n', rank(obsv(A, C)));

%% ===== GIAI DOAN 3 — Buoc 5: Thiet ke LQR (quy tac Bryson) =====
% Buoc 1: chon gia tri toi da chap nhan duoc cho tung bien:
x_max     = 0.20;   % m,    sai so vi tri toi da chap nhan
xdot_max  = 0.50;   % m/s
theta_max = 0.10;   % rad (~5.7 deg), gioi han de xap xi tuyen tinh con dung
thetadot_max = 1.0; % rad/s
u_max     = 3.0;    % N,   luc keo toi da cho phep (gioi han dong co)

Q = diag([1/x_max^2, 1/xdot_max^2, 1/theta_max^2, 1/thetadot_max^2]);
R = 1/u_max^2;

[K, S, E_lqr] = lqr(A, B, Q, R);
disp('K ='); disp(K);

%% ===== Buoc 6: Kiem chung K =====
Acl = A - B*K;
disp('eig(A-BK) ='); disp(eig(Acl).');   % tat ca phai co phan thuc am
sys_cl = ss(Acl, zeros(4,1), eye(4), zeros(4,1));
figure; initial(sys_cl, [0;0;0.15;0], 2); grid on;
title('Buoc 6: Dap ung tu do (theta_0=0.15 rad), phan hoi trang thai day du');

%% ===== Buoc 7: Thiet ke Kalman (L) =====
% Qn: nhieu qua trinh (do mo hinh khong hoan hao), Rn: nhieu do (datasheet)
Qn = diag([1e-6, 1e-4, 1e-6, 1e-3]) * 100;   % da tinh chinh de thoa buoc "vang"
Rn = diag([0.001^2, 0.02^2]);   % encoder ~1mm, goc IMU sau fusion ~0.02 rad

[L, P, E_obs] = lqe(A, eye(4), C, Qn, Rn);
disp('L ='); disp(L);
disp('eig(A-LC) ='); disp(eig(A-L*C).');

slowest_ctrl = max(real(E_lqr));
slowest_obs  = max(real(E_obs));
fprintf('Ti le cuc quan sat / cuc dieu khien = %.2f (muc tieu 3-5 lan)\n', ...
        slowest_obs/slowest_ctrl);

%% ===== Buoc 8: Ghep LQG (nguyen ly tach biet) =====
% x_hat_dot = A*x_hat + B*u + L*(y - C*x_hat) ,  u = -K*x_hat
% Doi bien e = x - x_hat (sai so uoc luong), thay vao he kin gop:
%   d/dt [x; e] = [A-BK   BK ] [x]
%                 [ 0    A-LC] [e]
% Ma tran khoi tam giac tren => 8 tri rieng = eig(A-BK) hop eig(A-LC),
% tach doc lap hoan toan (chung minh truc tiep nguyen ly tach biet):
Aaug = [Acl, B*K; zeros(4), A-L*C];
disp('8 cuc cua he LQG gop:'); disp(eig(Aaug).');
disp('so voi hop cua eig(A-BK) va eig(A-LC):');
disp(sort([eig(Acl); eig(A-L*C)]).');

%% ===== GIAI DOAN 4 — Buoc 9: Roi rac hoa (ZOH, Ts=5ms) =====
Ts = 0.005;  % s
sysd = c2d(ss(A, B, C, D), Ts, 'zoh');
[Ad, Bd, Cd, Dd] = ssdata(sysd);
disp('Ad ='); disp(Ad);
disp('Bd ='); disp(Bd);

% Thiet ke lai K, L truc tiep trong mien roi rac (chinh xac hon la quy
% doi K,L lien tuc sang roi rac khi Ts khong qua nho so voi dong hoc xe):
Kd = dlqr(Ad, Bd, Q, R);
Ld = dlqe(Ad, eye(4), Cd, Qn, Rn);

fprintf('|eig(Ad-Bd*Kd)| (phai < 1): '); disp(abs(eig(Ad-Bd*Kd)).');
fprintf('|eig(Ad-Ld*Cd)| (phai < 1): '); disp(abs(eig(Ad-Ld*Cd)).');

%% ===== Buoc 10: Vong lap ngat Timer 5ms tren STM32 (gia code) =====
disp(' ');
disp('Trong ham ngat Timer (chay dung moi 5 ms):');
disp('  1) Doc IMU (MPU6050) + 2x encoder (AS5600) -> y = [x_enc; theta_imu]');
disp('  2) x_hat = Ad*x_hat + Bd*u_prev + Ld*(y - Cd*x_hat)   % uoc luong Kalman');
disp('  3) u = -Kd * x_hat                                     % luat dieu khien');
disp('  4) u = clamp(u, -u_max, u_max); xuat PWM/step cho DRV8825');
