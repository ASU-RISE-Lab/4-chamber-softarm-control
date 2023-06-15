function dxdt = funcRigidode_v1(t,x)
    m2 = (100 + 34*2 + 25*5)/1000; %kg
    m1 = m2;
    g = 9.8;
    Iyy2 = 0;
    Iyy5 = 0;
a1 = 0.02;
q1 = x(1);
q2 = x(2);
q3 = x(3);
q4 = x(4);
q5 = x(5);
q6 = x(6);


M6x6 = [m1 + m2, -m2*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4)),m2*cos(q2),m2*cos(q2), 0, 0;
-m2*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4)), m2*(cos(q2)*(a1 + q3) + cos(q2)*(a1 + q4))^2 + m2*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4))^2 + Iyy2*cos(q2)^2 + Iyy5*cos(q2)^2 + Iyy2*sin(q2)^2 + Iyy5*sin(q2)^2, m2*sin(q2)*(cos(q2)*(a1 + q3) + cos(q2)*(a1 + q4)) - m2*cos(q2)*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4)), m2*sin(q2)*(cos(q2)*(a1 + q3) + cos(q2)*(a1 + q4)) - m2*cos(q2)*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4)), Iyy5*cos(q2)*(cos(q2)*cos(q5) - sin(q2)*sin(q5)) + Iyy5*sin(q2)*(cos(q2)*sin(q5) + cos(q5)*sin(q2)), 0;
 m2*cos(q2), m2*sin(q2)*(cos(q2)*(a1 + q3) + cos(q2)*(a1 + q4)) - m2*cos(q2)*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4)), m2*cos(q2)^2 + m2*sin(q2)^2, m2*cos(q2)^2 + m2*sin(q2)^2, 0, 0;
 m2*cos(q2), m2*sin(q2)*(cos(q2)*(a1 + q3) + cos(q2)*(a1 + q4)) - m2*cos(q2)*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4)), m2*cos(q2)^2 + m2*sin(q2)^2, m2*cos(q2)^2 + m2*sin(q2)^2, 0, 0;
0, Iyy5*cos(q2)*(cos(q2)*cos(q5) - sin(q2)*sin(q5)) + Iyy5*sin(q2)*(cos(q2)*sin(q5) + cos(q5)*sin(q2)), 0, 0, Iyy5*(cos(q2)*sin(q5) + cos(q5)*sin(q2))^2 + Iyy5*(cos(q2)*cos(q5) - sin(q2)*sin(q5))^2, 0;
0, 0, 0, 0, 0, 0;];

C6x6 = [ 0, 0,0,0, 0, 0;
- m2*qd3*sin(q2) - m2*qd4*sin(q2) - m2*qd2*(cos(q2)*(a1 + q3) + cos(q2)*(a1 + q4)), qd3*(m2*cos(q2)*(cos(q2)*(a1 + q3) + cos(q2)*(a1 + q4)) + m2*sin(q2)*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4))) + qd4*(m2*cos(q2)*(cos(q2)*(a1 + q3) + cos(q2)*(a1 + q4)) + m2*sin(q2)*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4))), -qd2*(m2*cos(q2)*(cos(q2)*(a1 + q3) + cos(q2)*(a1 + q4)) + m2*sin(q2)*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4))), -qd2*(m2*cos(q2)*(cos(q2)*(a1 + q3) + cos(q2)*(a1 + q4)) + m2*sin(q2)*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4))), 0, 0;
 -m2*qd2*sin(q2), qd2*(m2*cos(q2)*(cos(q2)*(a1 + q3) + cos(q2)*(a1 + q4)) + m2*sin(q2)*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4))),0,0, 0, 0;
 -m2*qd2*sin(q2), qd2*(m2*cos(q2)*(cos(q2)*(a1 + q3) + cos(q2)*(a1 + q4)) + m2*sin(q2)*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4))),0,0, 0, 0;
 0,-qd5*(Iyy5*cos(q2)*(cos(q2)*sin(q5) + cos(q5)*sin(q2)) - Iyy5*sin(q2)*(cos(q2)*cos(q5) - sin(q2)*sin(q5))),0,0, 0, 0;
 0, 0,0,0, 0, 0;
];

G6x1= [                                  g*m1 + g*m2;
-g*m2*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4));
                                 g*m2*cos(q2);
                                 g*m2*cos(q2);
                                            0;
                                            0;];

temp_dxdt = -M6x6\(C6x6*[x(2);x(4);x(6);x(8);] + G6x1);
dxdt = [x(2);temp_dxdt(1);
        x(4);temp_dxdt(2);
        x(6);temp_dxdt(3);
        x(8);temp_dxdt(4);];
end


