function [new_R,new_v,new_p,new_P] = timeUpdate(dt,in2,in3,in4,in5,in6,in7,in8,in9,in10,g,th)
%TIMEUPDATE
%    [NEW_R,NEW_V,NEW_P,NEW_P] = TIMEUPDATE(DT,IN2,IN3,IN4,IN5,IN6,IN7,IN8,IN9,IN10,G,TH)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    31-Mar-2021 23:00:40

P1_1 = in7(1);
P1_2 = in7(10);
P1_3 = in7(19);
P1_4 = in7(28);
P1_5 = in7(37);
P1_6 = in7(46);
P1_7 = in7(55);
P1_8 = in7(64);
P1_9 = in7(73);
P2_1 = in7(2);
P2_2 = in7(11);
P2_3 = in7(20);
P2_4 = in7(29);
P2_5 = in7(38);
P2_6 = in7(47);
P2_7 = in7(56);
P2_8 = in7(65);
P2_9 = in7(74);
P3_1 = in7(3);
P3_2 = in7(12);
P3_3 = in7(21);
P3_4 = in7(30);
P3_5 = in7(39);
P3_6 = in7(48);
P3_7 = in7(57);
P3_8 = in7(66);
P3_9 = in7(75);
P4_1 = in7(4);
P4_2 = in7(13);
P4_3 = in7(22);
P4_4 = in7(31);
P4_5 = in7(40);
P4_6 = in7(49);
P4_7 = in7(58);
P4_8 = in7(67);
P4_9 = in7(76);
P5_1 = in7(5);
P5_2 = in7(14);
P5_3 = in7(23);
P5_4 = in7(32);
P5_5 = in7(41);
P5_6 = in7(50);
P5_7 = in7(59);
P5_8 = in7(68);
P5_9 = in7(77);
P6_1 = in7(6);
P6_2 = in7(15);
P6_3 = in7(24);
P6_4 = in7(33);
P6_5 = in7(42);
P6_6 = in7(51);
P6_7 = in7(60);
P6_8 = in7(69);
P6_9 = in7(78);
P7_1 = in7(7);
P7_2 = in7(16);
P7_3 = in7(25);
P7_4 = in7(34);
P7_5 = in7(43);
P7_6 = in7(52);
P7_7 = in7(61);
P7_8 = in7(70);
P7_9 = in7(79);
P8_1 = in7(8);
P8_2 = in7(17);
P8_3 = in7(26);
P8_4 = in7(35);
P8_5 = in7(44);
P8_6 = in7(53);
P8_7 = in7(62);
P8_8 = in7(71);
P8_9 = in7(80);
P9_1 = in7(9);
P9_2 = in7(18);
P9_3 = in7(27);
P9_4 = in7(36);
P9_5 = in7(45);
P9_6 = in7(54);
P9_7 = in7(63);
P9_8 = in7(72);
P9_9 = in7(81);
R1_1 = in2(1);
R1_2 = in2(4);
R1_3 = in2(7);
R2_1 = in2(2);
R2_2 = in2(5);
R2_3 = in2(8);
R3_1 = in2(3);
R3_2 = in2(6);
R3_3 = in2(9);
a1 = in5(1,:);
a2 = in5(2,:);
a3 = in5(3,:);
ac1 = in9(1,:);
ac2 = in9(2,:);
ac3 = in9(3,:);
gc1 = in8(1,:);
gc2 = in8(2,:);
gc3 = in8(3,:);
p1 = in4(1,:);
p2 = in4(2,:);
p3 = in4(3,:);
v1 = in3(1,:);
v2 = in3(2,:);
v3 = in3(3,:);
vc1 = in10(1,:);
vc2 = in10(2,:);
vc3 = in10(3,:);
w1 = in6(1,:);
w2 = in6(2,:);
w3 = in6(3,:);
t2 = cos(th);
t3 = sin(th);
t4 = R1_1.*a1;
t5 = R1_2.*a2;
t6 = R1_3.*a3;
t7 = R2_1.*a1;
t8 = R2_2.*a2;
t9 = R2_3.*a3;
t10 = R3_1.*a1;
t11 = R3_2.*a2;
t12 = R3_3.*a3;
t13 = P4_1.*dt;
t14 = P4_2.*dt;
t15 = P4_4.*dt;
t16 = P4_5.*dt;
t17 = P4_6.*dt;
t18 = P5_1.*dt;
t19 = P5_2.*dt;
t20 = P5_4.*dt;
t21 = P5_5.*dt;
t22 = P5_6.*dt;
t23 = P6_1.*dt;
t24 = P6_2.*dt;
t25 = P6_4.*dt;
t26 = P6_5.*dt;
t27 = P6_6.*dt;
t28 = dt.^2;
t29 = w1.^2;
t30 = w2.^2;
t31 = w3.^2;
t32 = P1_1.*dt.*g;
t33 = P1_2.*dt.*g;
t34 = P1_4.*dt.*g;
t35 = P1_5.*dt.*g;
t36 = P1_6.*dt.*g;
t37 = P2_1.*dt.*g;
t38 = P2_2.*dt.*g;
t39 = P2_4.*dt.*g;
t40 = P2_5.*dt.*g;
t41 = P2_6.*dt.*g;
t42 = 1.0./th;
t43 = t42.^2;
t44 = t2-1.0;
t45 = P7_1+t13;
t46 = P7_2+t14;
t47 = P7_6+t17;
t48 = P8_1+t18;
t49 = P8_2+t19;
t50 = P8_6+t22;
t51 = P9_1+t23;
t52 = P9_2+t24;
t53 = P9_6+t27;
t54 = P5_1+t32;
t55 = P5_2+t33;
t56 = P5_6+t36;
t57 = -t37;
t58 = -t38;
t59 = -t39;
t60 = -t40;
t61 = -t41;
t65 = dt.*t3.*t42.*w1;
t66 = dt.*t3.*t42.*w2;
t67 = dt.*t3.*t42.*w3;
t68 = t4+t5+t6;
t69 = t7+t8+t9;
t70 = g+t10+t11+t12;
t62 = P4_1+t57;
t63 = P4_2+t58;
t64 = P4_6+t61;
t71 = t28.*t43.*t44.*w1.*w2;
t72 = t28.*t43.*t44.*w1.*w3;
t73 = t28.*t43.*t44.*w2.*w3;
t74 = t28.*t29.*t43.*t44;
t75 = t28.*t30.*t43.*t44;
t76 = t28.*t31.*t43.*t44;
t77 = -t71;
t78 = -t72;
t79 = -t73;
t80 = t65+t73;
t81 = t66+t72;
t82 = t67+t71;
t86 = t74+t75+1.0;
t87 = t74+t76+1.0;
t88 = t75+t76+1.0;
t83 = t65+t79;
t84 = t66+t78;
t85 = t67+t77;
new_R = reshape([-R1_3.*t81+R1_2.*t85+R1_1.*t88,-R2_3.*t81+R2_2.*t85+R2_1.*t88,-R3_3.*t81+R3_2.*t85+R3_1.*t88,-R1_1.*t82+R1_3.*t83+R1_2.*t87,-R2_1.*t82+R2_3.*t83+R2_2.*t87,-R3_1.*t82+R3_3.*t83+R3_2.*t87,-R1_2.*t80+R1_1.*t84+R1_3.*t86,-R2_2.*t80+R2_1.*t84+R2_3.*t86,-R3_2.*t80+R3_1.*t84+R3_3.*t86],[3,3]);
if nargout > 1
    new_v = [v1+dt.*t68;v2+dt.*t69;v3+dt.*t70];
end
if nargout > 2
    new_p = [p1+dt.*v1+(t28.*t68)./2.0;p2+dt.*v2+(t28.*t69)./2.0;p3+dt.*v3+(t28.*t70)./2.0];
end
if nargout > 3
    new_P = reshape([P1_1+gc1,P2_1,P3_1,t62,t54,P6_1,t45,t48,t51,P1_2,P2_2+gc2,P3_2,t63,t55,P6_2,t46,t49,t52,P1_3,P2_3,P3_3+gc3,P4_3-P2_3.*dt.*g,P5_3+P1_3.*dt.*g,P6_3,P7_3+P4_3.*dt,P8_3+P5_3.*dt,P9_3+P6_3.*dt,P1_4-t33,P2_4+t58,P3_4-P3_2.*dt.*g,P4_4+ac1+t59-dt.*g.*t63,P5_4+t34-dt.*g.*t55,P6_4-g.*t24,P7_4+t15-dt.*g.*t46,P8_4+t20-dt.*g.*t49,P9_4+t25-dt.*g.*t52,P1_5+t32,P2_5+t37,P3_5+P3_1.*dt.*g,P4_5+t60+dt.*g.*t62,P5_5+ac2+t35+dt.*g.*t54,P6_5+g.*t23,P7_5+t16+dt.*g.*t45,P8_5+t21+dt.*g.*t48,P9_5+t26+dt.*g.*t51,P1_6,P2_6,P3_6,t64,t56,P6_6+ac3,t47,t50,t53,P1_7+P1_4.*dt,P2_7+P2_4.*dt,P3_7+P3_4.*dt,P4_7+dt.*(P4_4+t59)-P2_7.*dt.*g,P5_7+dt.*(P5_4+t34)+P1_7.*dt.*g,P6_7+t25,P7_7+vc1+P4_7.*dt+dt.*(P7_4+t15),P8_7+P5_7.*dt+dt.*(P8_4+t20),P9_7+P6_7.*dt+dt.*(P9_4+t25),P1_8+P1_5.*dt,P2_8+P2_5.*dt,P3_8+P3_5.*dt,P4_8+dt.*(P4_5+t60)-P2_8.*dt.*g,P5_8+dt.*(P5_5+t35)+P1_8.*dt.*g,P6_8+t26,P7_8+P4_8.*dt+dt.*(P7_5+t16),P8_8+vc2+P5_8.*dt+dt.*(P8_5+t21),P9_8+P6_8.*dt+dt.*(P9_5+t26),P1_9+P1_6.*dt,P2_9+P2_6.*dt,P3_9+P3_6.*dt,P4_9+dt.*t64-P2_9.*dt.*g,P5_9+dt.*t56+P1_9.*dt.*g,P6_9+t27,P7_9+P4_9.*dt+dt.*t47,P8_9+P5_9.*dt+dt.*t50,P9_9+vc3+P6_9.*dt+dt.*t53],[9,9]);
end
