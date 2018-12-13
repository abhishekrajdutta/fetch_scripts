l0 = 1;
l1 = 1;
l2 = 1;
l3 = 1;
l4=1;
l5=1;
l6=1;



A=[0,-1.57,0,1.57,0,0,0]';
g1=SE3(rotz(A(1)*180/pi),[0 0 0]');
g2=SE3(roty(A(2)*180/pi),[l0 0 0]');
g3=SE3(rotx(A(3)*180/pi),[l1 0 0]');
g4=SE3(roty(A(4)*180/pi),[l2 0 0]');
g5=SE3(rotx(A(5)*180/pi),[l3 0 0]');
g6=SE3(roty(A(6)*180/pi),[l4 0 0]');
g7=SE3(rotx(A(7)*180/pi),[l5 0 0]');
g8=SE3(eye(3),[l6 0 0]');
disp('gstart')
g_start=g1*g2*g3*g4*g5*g6*g7*g8

si0=[0 0 0 0 0 1]';
si1=[0 0 1 0 1 0]';
si2=[0 0 0 1 0 0]';
si3=[0 0 3 0 1 0]';
si4=[0 0 0 1 0 0]';
si5=[0 0 5 0 1 0]';
si6=[0 0 0 1 0 0]';

j=[si0 si1 si2 si3 si4 si5];

goal1 = [3,2,1]';
goal2 = [3,1,1]';
goal3 = [3,1,-1]';
goal4 = [3,2,-1]';
goal5 = [3,2,1]';

start = getTranslation(g_start);
diff = goal1 - start;

N = 200;


figure


for i = 1:N
    g1=SE3(rotz(A(1)*180/pi),[0 0 0]');
    g2=SE3(roty(A(2)*180/pi),[l0 0 0]');
    g3=SE3(rotx(A(3)*180/pi),[l1 0 0]');
    g4=SE3(roty(A(4)*180/pi),[l2 0 0]');
    g5=SE3(rotx(A(5)*180/pi),[l3 0 0]');
    g6=SE3(roty(A(6)*180/pi),[l4 0 0]');
    g7=SE3(rotx(A(7)*180/pi),[l5 0 0]');
    g8=SE3(eye(3),[l6 0 0]');
    g_curr = g1*g2*g3*g4*g5*g6*g7*g8;
    
    pose = start + (i/N)*diff;
    curr = getTranslation(g_curr);
    vdot = [pose(1) - curr(1),pose(2) - curr(2),pose(3) - curr(3),0,0,0]';
    
%     calculate the jacobian here
    T = g2*g3*g4*g5*g6*g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J1 = inv(AdT)*si0;
    
    T = g3*g4*g5*g6*g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J2 = inv(AdT)*si1;
    
    T = g4*g5*g6*g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];;
    J3 = inv(AdT)*si2;
    
    T = g5*g6*g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J4 = inv(AdT)*si3;
    
    T = g6*g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J5 = inv(AdT)*si4;

    T = g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J6 = inv(AdT)*si5;

    T = g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J7 = inv(AdT)*si6;
    
    J = [J1 J2 J3 J4 J5 J6 J7];
    
%     Adot = pinv(J)*vdot;
%     W=[1 0 0 0 0 0 0; 0 1 0 0 0 0 0; 0 0 1 0 0 0 0;0 0 0 1 0 0 0;0 0 0 0 1 0 0;0 0 0 0 0 1 0;0 0 0 0 0 0 1 ];
    Adot = J'*inv(J*J' + 0.05*eye(6))*vdot;
    A = A+Adot;
    
    


    
    g1=SE3(rotz(A(1)*180/pi),[0 0 0]');
    g2=SE3(roty(A(2)*180/pi),[l0 0 0]');
    g3=SE3(rotx(A(3)*180/pi),[l1 0 0]');
    g4=SE3(roty(A(4)*180/pi),[l2 0 0]');
    g5=SE3(rotx(A(5)*180/pi),[l3 0 0]');
    g6=SE3(roty(A(6)*180/pi),[l4 0 0]');
    g7=SE3(rotx(A(7)*180/pi),[l5 0 0]');
    g8=SE3(eye(3),[l6 0 0]');
    g_check = getTranslation(g1*g2*g3*g4*g5*g6*g7*g8);   
    
    X(i) = g_check(1);
    Y(i) = g_check(2);
    Z(i) = g_check(3);
    
end



scatter3(X,Y,Z)

hold;


start =g_check;
diff = goal2 - start;

for i = 1:N
    g1=SE3(rotz(A(1)*180/pi),[0 0 0]');
    g2=SE3(roty(A(2)*180/pi),[l0 0 0]');
    g3=SE3(rotx(A(3)*180/pi),[l1 0 0]');
    g4=SE3(roty(A(4)*180/pi),[l2 0 0]');
    g5=SE3(rotx(A(5)*180/pi),[l3 0 0]');
    g6=SE3(roty(A(6)*180/pi),[l4 0 0]');
    g7=SE3(rotx(A(7)*180/pi),[l5 0 0]');
    g8=SE3(eye(3),[l6 0 0]');
    g_curr = g1*g2*g3*g4*g5*g6*g7*g8;
    
    pose = start + (i/N)*diff;
    curr = getTranslation(g_curr);
    vdot = [pose(1) - curr(1),pose(2) - curr(2),pose(3) - curr(3),0,0,0]';
    
%     calculate the jacobian here
    T = g2*g3*g4*g5*g6*g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J1 = inv(AdT)*si0;
    
    T = g3*g4*g5*g6*g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J2 = inv(AdT)*si1;
    
    T = g4*g5*g6*g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];;
    J3 = inv(AdT)*si2;
    
    T = g5*g6*g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J4 = inv(AdT)*si3;
    
    T = g6*g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J5 = inv(AdT)*si4;

    T = g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J6 = inv(AdT)*si5;

    T = g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J7 = inv(AdT)*si6;
    
    J = [J1 J2 J3 J4 J5 J6 J7];
    
%     Adot = pinv(J)*vdot;
%     W=[1 0 0 0 0 0 0; 0 1 0 0 0 0 0; 0 0 1 0 0 0 0;0 0 0 1 0 0 0;0 0 0 0 1 0 0;0 0 0 0 0 1 0;0 0 0 0 0 0 1 ];
    Adot = J'*inv(J*J' + 0.05*eye(6))*vdot;
    A = A+Adot;
    
    


    
    g1=SE3(rotz(A(1)*180/pi),[0 0 0]');
    g2=SE3(roty(A(2)*180/pi),[l0 0 0]');
    g3=SE3(rotx(A(3)*180/pi),[l1 0 0]');
    g4=SE3(roty(A(4)*180/pi),[l2 0 0]');
    g5=SE3(rotx(A(5)*180/pi),[l3 0 0]');
    g6=SE3(roty(A(6)*180/pi),[l4 0 0]');
    g7=SE3(rotx(A(7)*180/pi),[l5 0 0]');
    g8=SE3(eye(3),[l6 0 0]');
    g_check = getTranslation(g1*g2*g3*g4*g5*g6*g7*g8);   
    
    X(i) = g_check(1);
    Y(i) = g_check(2);
    Z(i) = g_check(3);
    
end

scatter3(X,Y,Z)



start = g_check;
diff = goal3 - start;

for i = 1:N
    g1=SE3(rotz(A(1)*180/pi),[0 0 0]');
    g2=SE3(roty(A(2)*180/pi),[l0 0 0]');
    g3=SE3(rotx(A(3)*180/pi),[l1 0 0]');
    g4=SE3(roty(A(4)*180/pi),[l2 0 0]');
    g5=SE3(rotx(A(5)*180/pi),[l3 0 0]');
    g6=SE3(roty(A(6)*180/pi),[l4 0 0]');
    g7=SE3(rotx(A(7)*180/pi),[l5 0 0]');
    g8=SE3(eye(3),[l6 0 0]');
    g_curr = g1*g2*g3*g4*g5*g6*g7*g8;
    
    pose = start + (i/N)*diff;
    curr = getTranslation(g_curr);
    vdot = [pose(1) - curr(1),pose(2) - curr(2),pose(3) - curr(3),0,0,0]';
    
%     calculate the jacobian here
    T = g2*g3*g4*g5*g6*g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J1 = inv(AdT)*si0;
    
    T = g3*g4*g5*g6*g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J2 = inv(AdT)*si1;
    
    T = g4*g5*g6*g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];;
    J3 = inv(AdT)*si2;
    
    T = g5*g6*g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J4 = inv(AdT)*si3;
    
    T = g6*g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J5 = inv(AdT)*si4;

    T = g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J6 = inv(AdT)*si5;

    T = g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J7 = inv(AdT)*si6;
    
    J = [J1 J2 J3 J4 J5 J6 J7];
    
%     Adot = pinv(J)*vdot;
%     W=[1 0 0 0 0 0 0; 0 1 0 0 0 0 0; 0 0 1 0 0 0 0;0 0 0 1 0 0 0;0 0 0 0 1 0 0;0 0 0 0 0 1 0;0 0 0 0 0 0 1 ];
    Adot = J'*inv(J*J' + 0.05*eye(6))*vdot;
    A = A+Adot;
    
    


    
    g1=SE3(rotz(A(1)*180/pi),[0 0 0]');
    g2=SE3(roty(A(2)*180/pi),[l0 0 0]');
    g3=SE3(rotx(A(3)*180/pi),[l1 0 0]');
    g4=SE3(roty(A(4)*180/pi),[l2 0 0]');
    g5=SE3(rotx(A(5)*180/pi),[l3 0 0]');
    g6=SE3(roty(A(6)*180/pi),[l4 0 0]');
    g7=SE3(rotx(A(7)*180/pi),[l5 0 0]');
    g8=SE3(eye(3),[l6 0 0]');
    g_check = getTranslation(g1*g2*g3*g4*g5*g6*g7*g8);   
    
    X(i) = g_check(1);
    Y(i) = g_check(2);
    Z(i) = g_check(3);
    
end

scatter3(X,Y,Z)


start = g_check;
diff = goal4 - start;

for i = 1:N
    g1=SE3(rotz(A(1)*180/pi),[0 0 0]');
    g2=SE3(roty(A(2)*180/pi),[l0 0 0]');
    g3=SE3(rotx(A(3)*180/pi),[l1 0 0]');
    g4=SE3(roty(A(4)*180/pi),[l2 0 0]');
    g5=SE3(rotx(A(5)*180/pi),[l3 0 0]');
    g6=SE3(roty(A(6)*180/pi),[l4 0 0]');
    g7=SE3(rotx(A(7)*180/pi),[l5 0 0]');
    g8=SE3(eye(3),[l6 0 0]');
    g_curr = g1*g2*g3*g4*g5*g6*g7*g8;
    
    pose = start + (i/N)*diff;
    curr = getTranslation(g_curr);
    vdot = [pose(1) - curr(1),pose(2) - curr(2),pose(3) - curr(3),0,0,0]';
    
%     calculate the jacobian here
    T = g2*g3*g4*g5*g6*g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J1 = inv(AdT)*si0;
    
    T = g3*g4*g5*g6*g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J2 = inv(AdT)*si1;
    
    T = g4*g5*g6*g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];;
    J3 = inv(AdT)*si2;
    
    T = g5*g6*g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J4 = inv(AdT)*si3;
    
    T = g6*g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J5 = inv(AdT)*si4;

    T = g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J6 = inv(AdT)*si5;

    T = g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J7 = inv(AdT)*si6;
    
    J = [J1 J2 J3 J4 J5 J6 J7];
    
%     Adot = pinv(J)*vdot;
%     W=[1 0 0 0 0 0 0; 0 1 0 0 0 0 0; 0 0 1 0 0 0 0;0 0 0 1 0 0 0;0 0 0 0 1 0 0;0 0 0 0 0 1 0;0 0 0 0 0 0 1 ];
    Adot = J'*inv(J*J' + 0.05*eye(6))*vdot;
    A = A+Adot;
    
    


    
    g1=SE3(rotz(A(1)*180/pi),[0 0 0]');
    g2=SE3(roty(A(2)*180/pi),[l0 0 0]');
    g3=SE3(rotx(A(3)*180/pi),[l1 0 0]');
    g4=SE3(roty(A(4)*180/pi),[l2 0 0]');
    g5=SE3(rotx(A(5)*180/pi),[l3 0 0]');
    g6=SE3(roty(A(6)*180/pi),[l4 0 0]');
    g7=SE3(rotx(A(7)*180/pi),[l5 0 0]');
    g8=SE3(eye(3),[l6 0 0]');
    g_check = getTranslation(g1*g2*g3*g4*g5*g6*g7*g8);   
    
    X(i) = g_check(1);
    Y(i) = g_check(2);
    Z(i) = g_check(3);
    
end

scatter3(X,Y,Z)


start = g_check;
diff = goal5 - start;

for i = 1:N
    g1=SE3(rotz(A(1)*180/pi),[0 0 0]');
    g2=SE3(roty(A(2)*180/pi),[l0 0 0]');
    g3=SE3(rotx(A(3)*180/pi),[l1 0 0]');
    g4=SE3(roty(A(4)*180/pi),[l2 0 0]');
    g5=SE3(rotx(A(5)*180/pi),[l3 0 0]');
    g6=SE3(roty(A(6)*180/pi),[l4 0 0]');
    g7=SE3(rotx(A(7)*180/pi),[l5 0 0]');
    g8=SE3(eye(3),[l6 0 0]');
    g_curr = g1*g2*g3*g4*g5*g6*g7*g8;
    
    pose = start + (i/N)*diff;
    curr = getTranslation(g_curr);
    vdot = [pose(1) - curr(1),pose(2) - curr(2),pose(3) - curr(3),0,0,0]';
    
%     calculate the jacobian here
    T = g2*g3*g4*g5*g6*g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J1 = inv(AdT)*si0;
    
    T = g3*g4*g5*g6*g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J2 = inv(AdT)*si1;
    
    T = g4*g5*g6*g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J3 = inv(AdT)*si2;
    
    T = g5*g6*g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J4 = inv(AdT)*si3;
    
    T = g6*g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J5 = inv(AdT)*si4;

    T = g7*g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J6 = inv(AdT)*si5;

    T = g8;
    d = getTranslation(T);
    R = getR(T);
    hat_d = [0 , -d(3) , d(2); ...
        d(3), 0, -d(1); ...
        -d(2), d(1), 0];
    AdT = [R , hat_d*R ; zeros(3) , R];
    J7 = inv(AdT)*si6;
    
    J = [J1 J2 J3 J4 J5 J6 J7];
    
%     Adot = pinv(J)*vdot;
%     W=[1 0 0 0 0 0 0; 0 1 0 0 0 0 0; 0 0 1 0 0 0 0;0 0 0 1 0 0 0;0 0 0 0 1 0 0;0 0 0 0 0 1 0;0 0 0 0 0 0 1 ];
    Adot = J'*inv(J*J' + 0.05*eye(6))*vdot;
    A = A+Adot;
    
    


    
    g1=SE3(rotz(A(1)*180/pi),[0 0 0]');
    g2=SE3(roty(A(2)*180/pi),[l0 0 0]');
    g3=SE3(rotx(A(3)*180/pi),[l1 0 0]');
    g4=SE3(roty(A(4)*180/pi),[l2 0 0]');
    g5=SE3(rotx(A(5)*180/pi),[l3 0 0]');
    g6=SE3(roty(A(6)*180/pi),[l4 0 0]');
    g7=SE3(rotx(A(7)*180/pi),[l5 0 0]');
    g8=SE3(eye(3),[l6 0 0]');
    g_check = getTranslation(g1*g2*g3*g4*g5*g6*g7*g8);   
    
    X(i) = g_check(1);
    Y(i) = g_check(2);
    Z(i) = g_check(3);
    
end

scatter3(X,Y,Z)

daspect([1 1 1])

% axis([-5 5 -5 5 -5 5])
axis normal