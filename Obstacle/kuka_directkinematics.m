function T = kuka_directkinematics(q)
%
% Computes the homogeneous transformation matrix for the Kuka LWR
%
% function T = kuka_directkinematics
%
% input:
%       q     dim 7x1     joint positions
%
% output:
%       T     dim 4x4     homogeneous transformation matrix from base to
%       end-effector
%
% G. Antonelli, Sistemi Robotici, fall 2012

d0=0.3105;
d3=0.40; 
d5=0.39; 
d7=0.078; 
db=0; 


q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);
q5=q(5);
q6=q(6);
q7=q(7);

s1 = sin(q1);
c1 = cos(q1);
s2 = sin(q2);
c2 = cos(q2);
s3 = sin(q3);
c3 = cos(q3);
s4 = sin(q4);
c4 = cos(q4);
s5 = sin(q5);
c5 = cos(q5);
s6 = sin(q6);
c6 = cos(q6);
s7 = sin(q7);
c7 = cos(q7);

%%% the frame assignment is different with the assigned frame in kuka doccument
Tb0=[ 1 0 0 0;
      0 1 0 db;
      0 0 1 d0;
      0 0 0 1];       

A01=[ c1 0 s1 0;
      s1 0 -c1 0;
      0 1 0 0;
      0 0 0 1];

A12=[ c2 0 -s2 0;
      s2 0 c2 0;
      0 -1 0 0;
      0 0 0 1]; 

A23=[ -c3 0 -s3 0;
      -s3 0 c3 0;
      0 1 0 d3;
      0 0 0 1];

A34=[-c4 0 -s4 0;
      -s4 0 c4 0;
      0 1 0 0;
      0 0 0 1]; 

A45=[ c5 0 s5 0;
      s5 0 -c5 0;
      0 1 0 d5;

      0 0 0 1];

A56=[ c6 0 -s6 0;
      s6 0 c6 0;
      0 -1 0 0;
      0 0 0 1];

A67=[ c7 -s7 0 0;
      s7 c7 0 0;
      0 0 1 0;
      0 0 0 1];

T7e=[ 1 0  0  0;
      0 1 0  0 ;
      0 0 1 d7;
      0 0 0 1];  
  
T = Tb0*A01*A12*A23*A34*A45*A56*A67*T7e; % transformation matrix
 
 
