clc;clear;close all;
gamma=0.5;
 	 
           

x(1) = 1; y(1) = 1; 
d = .1;  xf = 2; yf = 2;
x(1) = 0; y(1) = 0; 
xf = 20; yf = 10;
%t1 =48.1;
t1 =3;
t1 =200;

% x(1) = 29; y(1) = 1265; 
% xf = 49; yf = 861;
tetha(1) =0*pi/2; tetha(1)=atan((yf-y(1))/(xf-x(1)));
zd1(1) = x(1); zd2(1) = (xf-x(1))/t1; q = 1; q2 = 1;
Q = zeros(7,7); 
Q(1) = q; Q(4) = -q; Q(19) = -q; Q(22) = q;
Q(15) = q2; Q(18) = -q2; Q(33) = -q2; Q(36) = q2;Q=eye(7);
Q=[1 0 0 -1 0 0 0 0 0;0 1 0 0 0 -1 0 0 0;0 0 1 0 0 0 0 -1 0]'*eye(3)*[1 0 0 -1 0 0 0 0 0;0 1 0 0 0 -1 0 0 0;0 0 1 0 0 0 0 -1 0];
R=.01*eye(2);
% R=[.1 0;0 .1];

m = (yf-y(1))/(xf-x(1)); %m=10000*exp(15)
n = y(1)-m*x(1);
% wd1(1) = m/sqrt(m^2+1);
wd1(1) =atan((yf-y(1))/(xf-x(1)));
yd1(1) = m*x(1)+n; yd2(1) = m*(xf-x(1))/t1;
Tt=[];
J = [];
v(1)=0;
V_sm=30;
 V_sm=.12;

for i = 1:t1/d
    if v(i) == 0; Ls(i) = 1; aa=22; else, 
        Ls(i) = sat(v(i),V_sm)/v(i); end
    
    A = [0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0;0 0 0 0 1 0 0 0;
        0 0 0 0 0 0 0 0;0 0 0 0 0 0 1 0;0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0];
    Ba1 = [cos(tetha(i)) sin(tetha(i)) 0 0 0 0 0 0]';
    
    Aa2 = [A Ba1*Ls(i); zeros(1,9)];
    B = [0 0 0 0 0 0 0 0 1;0 0 1 0 0 0 0 0 0]';
    
    K = lqr(Aa2-gamma*eye(9),B,Q,R);
    X=[x(i) y(i) tetha(i) zd1(i) zd2(i) yd1(i) yd2(i) wd1(i) v(i)];
    Ud(i) = sat(v(i),V_sm);
    u = -K*X';
    jad(i)=X*Q*X';
    jad2(i)=u'*R*u;
    JJ(i) = exp(-2*gamma*d*(i-1))*(X*Q*X'+u'*R*u); 

    %u = -K*[x(i) y(i) tetha(i) zd1(i) zd2(i) yd1(i) yd2(i) wd1(i)]';
    %JJ(i) = exp(-2*gamma*Tt(i+tend))*(X(i,:)*Q*X(i,:)'+u(i)'*R*u(i)); 
    
    
    v_til(i) = u(1); w(i) = u(2);
    %u(1)=min(u(1),1)
    x(i+1) = x(i)+d*(Ls(i)*v(i)*cos(tetha(i)));
    y(i+1) = y(i)+d*(Ls(i)*v(i)*sin(tetha(i)));
    tetha(i+1) = tetha(i)+d*(w(i));
    v(i+1) = v(i)+d*(v_til(i));

    zd1(i+1) = zd1(i)+d*(zd2(i));
    zd2(i+1) = zd2(i)+d*(0);
%     wd1(i+1) = wd1(i)+d*(0);
    yd1(i+1) = yd1(i)+d*(yd2(i));
    yd2(i+1) = yd2(i)+d*(0);
    wd1(i+1) = wd1(i)+d*(0);
    
    %Tt = [Tt,t1'+d*(i-1)];
    Tt = [Tt,d*(i-1)];

end
    %uu=[uu u];
    J = [J JJ];
for i = 2:length(Tt)
    V(i) = trapz(Tt(1:i),J(1:i));
end
cost=norm(V,'inf');

plot(x,y)
hold on
plot(zd1,yd1,'-.')
figure
plot(Tt,V,'LineWidth',2)
title('Cost'); xlabel('Time(s)'); grid on

figure
t = [0:d:t1-d];
plot(t,Ud,'LineWidth',2);grid on
