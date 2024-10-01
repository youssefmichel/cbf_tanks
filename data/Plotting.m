%% 
clear all
P=load('Pow_rob_vid_PTS_ord2_c1.txt') ;
E=load('Tank_rob_vid_PTS_ord2_c1.txt') ;

P=load('Pow_rob_vid_firstcbf_2.txt') ;
E=load('Tank_rob_vid_firstcbf_2.txt') ;


P_a=P(:,2) ;
P_d=P(:,1) ;

s_cbf=E(:,1) ;
s_cbf_dot=E(:,2) ;
alpha=E(:,2) ;


figure
plot(alpha)

% plot(P(:,4))
% hold on



figure
plot(P_a) 
hold on
plot(P_d,'g') 
hold on
plot(-s_cbf_dot,'r') 

figure
plot(s_cbf) 

figure
plot(alpha)



x_d=P(:,3) ;
x_d_dot=P(:,5) ;
x_d_act=P(:,4) ;



figure
plot(x_d_dot) 
hold on
plot(x_d_dot.*alpha)