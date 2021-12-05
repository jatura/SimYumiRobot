%clear all
clc
clear
cla reset

%define
%data
base        = [0;0;0];
heightRobot      = 571;
armDistance      = 260/2;
widghtRobot      = 421;
link1Lenght      = sqrt(166^2+30^2);
link2Lenght      = sqrt((251.5/2)^2+30^2);
link3Lenght      = sqrt((251.5/2)^2+40.5^2);
link4Lenght      = sqrt((265/2)^2+40.5^2);
link5Lenght      = sqrt((265/2)^2+27^2);
link6Lenght      = sqrt(27^2+36^2);
%angle
leftAngle1   = 0;
leftAngle2   = 0;
leftAngle3   = 0;
leftAngle4   = 0;
leftAngle5   = 0;
leftAngle6   = 0;
leftAngle7   = 0;
rightAngle1  = 0;
rightAngle2  = 0;
rightAngle3  = 0;
rightAngle4  = 0;
rightAngle5  = 0;
rightAngle6  = 0;
rightAngle7  = 0;

%transition matrix
transitionMatrixBase        = [ 1 0 0 0 ;
                                0 1 0 heightRobot ;
                                0 0 1 0 ;
                                0 0 0 1 ];
transitionMatrixWidght      = [ 1 0 0 0 ;
                                0 1 0 0 ;
                                0 0 1 widghtRobot ;
                                0 0 0 1 ];
%left side
transitionMatrixLeftBase    = [ 1 0 0 -armDistance ;
                                0 1 0 0 ;
                                0 0 1 0 ;
                                0 0 0 1 ];
transitionMatrixLeftJoint1_2    = [ 1                   0                   0                   0 ;
                                    0                   cosd(leftAngle1)    -sind(leftAngle1)   link1Lenght*cosd(leftAngle1) ;
                                    0                   sind(leftAngle1)    cosd(leftAngle1)    -link1Lenght*sind(leftAngle1) ;
                                    0                   0                   0                   1 ];
transitionMatrixLeftJoint2_3    = [ cosd(leftAngle2)    -sind(leftAngle2)   0                   0 ;
                                    sind(leftAngle2)    cosd(leftAngle2)    0                   link2Lenght*cosd(leftAngle2) ;
                                    0                   0                   1                   -link2Lenght*sind(leftAngle2) ;
                                    0                   0                   0                   1 ];
transitionMatrixLeftJoint3_4    = [ cosd(leftAngle3)    0                   sind(leftAngle3)    0 ;
                                    0                   1                   0                   link3Lenght ;
                                    -sind(leftAngle3)   0                   cosd(leftAngle3)    0 ;
                                    0                   0                   0                   1 ];               
transitionMatrixLeftJoint4_5    = [ cosd(leftAngle4)    -sind(leftAngle4)   0                   0 ;
                                    sind(leftAngle4)    cosd(leftAngle4)    0                   link4Lenght*cosd(leftAngle4) ;
                                    0                   0                   1                   -link4Lenght*sind(leftAngle4) ;
                                    0                   0                   0                   1 ];
transitionMatrixLeftJoint5_6    = [ cosd(leftAngle5)    -sind(leftAngle5)   0                   0 ;
                                    sind(leftAngle5)    cosd(leftAngle5)    0                   0 ;
                                    0                   0                   1                   link5Lenght ;
                                    0                   0                   0                   1 ];       
transitionMatrixLeftJoint6_7    = [ cosd(leftAngle6)    -sind(leftAngle6)   0                   0 ;
                                    sind(leftAngle6)    cosd(leftAngle6)    0                   link6Lenght*cosd(leftAngle6) ;
                                    0                   0                   1                   -link6Lenght*sind(leftAngle6) ;
                                    0                   0                   0                   1 ];
%right side
transitionMatrixRightBase    = [ 1 0 0 armDistance ;
                                0 1 0 0 ;
                                0 0 1 0 ;
                                0 0 0 1 ];
transitionMatrixRightJoint1_2    = [ 1                   0                   0                   0 ;
                                    0                   cosd(rightAngle1)    -sind(rightAngle1)   link1Lenght*cosd(rightAngle1) ;
                                    0                   sind(rightAngle1)    cosd(rightAngle1)    -link1Lenght*sind(rightAngle1) ;
                                    0                   0                   0                   1 ];
transitionMatrixRightJoint2_3    = [ cosd(rightAngle2)    -sind(rightAngle2)   0                   0 ;
                                    sind(rightAngle2)    cosd(rightAngle2)    0                   link2Lenght*cosd(rightAngle2) ;
                                    0                   0                   1                   -link2Lenght*sind(rightAngle2) ;
                                    0                   0                   0                   1 ];
transitionMatrixRightJoint3_4    = [ cosd(rightAngle3)    0                   sind(rightAngle3)    0 ;
                                    0                   1                   0                   link3Lenght ;
                                    -sind(rightAngle3)   0                   cosd(rightAngle3)    0 ;
                                    0                   0                   0                   1 ];               
transitionMatrixRightJoint4_5    = [ cosd(rightAngle4)    -sind(rightAngle4)   0                   0 ;
                                    sind(rightAngle4)    cosd(rightAngle4)    0                   link4Lenght*cosd(rightAngle4) ;
                                    0                   0                   1                   -link4Lenght*sind(rightAngle4) ;
                                    0                   0                   0                   1 ];
transitionMatrixRightJoint5_6    = [ cosd(rightAngle5)    -sind(rightAngle5)   0                   0 ;
                                    sind(rightAngle5)    cosd(rightAngle5)    0                   0 ;
                                    0                   0                   1                   link5Lenght ;
                                    0                   0                   0                   1 ];       
transitionMatrixRightJoint6_7    = [ cosd(rightAngle6)    -sind(rightAngle6)   0                   0 ;
                                    sind(rightAngle6)    cosd(rightAngle6)    0                   link6Lenght*cosd(rightAngle6) ;
                                    0                   0                   1                   -link6Lenght*sind(rightAngle6) ;
                                    0                   0                   0                   1 ];
%find position
homoTranB_H = transitionMatrixBase;
homoTranB_W = homoTranB_H*transitionMatrixWidght;
positionB_H = homoTranB_H*[0;0;0;1;];
positionB_W = homoTranB_W*[0;0;0;1;];

homoTranB_L1 = homoTranB_W*transitionMatrixLeftBase;
homoTranB_L2 = homoTranB_L1*transitionMatrixLeftJoint1_2;
homoTranB_L3 = homoTranB_L2*transitionMatrixLeftJoint2_3;
homoTranB_L4 = homoTranB_L3*transitionMatrixLeftJoint3_4;
homoTranB_L5 = homoTranB_L4*transitionMatrixLeftJoint4_5;
homoTranB_L6 = homoTranB_L5*transitionMatrixLeftJoint5_6;
homoTranB_L7 = homoTranB_L6*transitionMatrixLeftJoint6_7;
positionB_L1 = homoTranB_L1*[0;0;0;1;];
positionB_L2 = homoTranB_L2*[0;0;0;1;];
positionB_L3 = homoTranB_L3*[0;0;0;1;];
positionB_L4 = homoTranB_L4*[0;0;0;1;];
positionB_L5 = homoTranB_L5*[0;0;0;1;];
positionB_L6 = homoTranB_L6*[0;0;0;1;];
positionB_L7 = homoTranB_L7*[0;0;0;1;];

homoTranB_R1 = homoTranB_W*transitionMatrixRightBase;
homoTranB_R2 = homoTranB_R1*transitionMatrixRightJoint1_2;
homoTranB_R3 = homoTranB_R2*transitionMatrixRightJoint2_3;
homoTranB_R4 = homoTranB_R3*transitionMatrixRightJoint3_4;
homoTranB_R5 = homoTranB_R4*transitionMatrixRightJoint4_5;
homoTranB_R6 = homoTranB_R5*transitionMatrixRightJoint5_6;
homoTranB_R7 = homoTranB_R6*transitionMatrixRightJoint6_7;
positionB_R1 = homoTranB_R1*[0;0;0;1;];
positionB_R2 = homoTranB_R2*[0;0;0;1;];
positionB_R3 = homoTranB_R3*[0;0;0;1;];
positionB_R4 = homoTranB_R4*[0;0;0;1;];
positionB_R5 = homoTranB_R5*[0;0;0;1;];
positionB_R6 = homoTranB_R6*[0;0;0;1;];
positionB_R7 = homoTranB_R7*[0;0;0;1;];

%plot robot
plot3([base(1) positionB_H(1)], [base(2) positionB_H(2)], [base(3) positionB_H(3)], [positionB_H(1) positionB_W(1)], [positionB_H(2) positionB_W(2)], [positionB_H(3) positionB_W(3)], [positionB_W(1) positionB_L1(1)], [positionB_W(2) positionB_L1(2)], [positionB_W(3) positionB_L1(3)],[positionB_W(1) positionB_R1(1)], [positionB_W(2) positionB_R1(2)], [positionB_W(3) positionB_R1(3)],'-o', 'Color', '#fcf6ea', 'LineWidth', 3)
hold on
xlabel('X')
ylabel('Y')
zlabel('Z')
grid on

show(robot);