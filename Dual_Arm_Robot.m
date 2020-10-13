clear;
clc;
setenv('MW_MINGW64_LOC','C:\TDM-GCC-64')
mex -setup
mex -setup
loadlibrary('export_puma560_fik','export_puma560_fik.h');
libfunctions export_puma560_fik -full

L1 = 313;
L2 = 200;
L3 = 0;
L4 = 248;
%L5 = 70;
L5 =262;
L6 = 0;
L7 = 0.0;
dh= [L1, L2,L3,L4,L5,L6,L7];

pdh_robot = libpointer('doublePtr', dh);

jnt_pose_rad= [0 0 0 0 -90/180*pi 0];%�ؽڽǶ�

pfAxesPosRad = libpointer('doublePtr', jnt_pose_rad);%��libpointer����������ָ�� 

xyz_rpy_result_temp= zeros(1,6);
pxyz_rpy_rad = libpointer('doublePtr', xyz_rpy_result_temp);%��libpointer����������ָ�� 
ret=calllib('export_puma560_fik', 'fk_hrg_puma560_DefTcs', pdh_robot,pfAxesPosRad,pxyz_rpy_rad)

fk_result_arr = get(pxyz_rpy_rad, 'Value')


pdh_robot = libpointer('doublePtr', dh);%��libpointer����������ָ�� 
%get(pv, 'Value')%��ʾָ������� 
ref_jnt_pose_rad= [0 0 0 0 -90/180*pi 0];%�ο���ʼλ��
pRefJntPosRad = libpointer('doublePtr', ref_jnt_pose_rad);%��libpointer����������ָ�� 

target_carte_frame= [248.0000         0  -62.0000         0    3.1416   -1.5708];%�ѿ�������xyz rpy ĩ��
%����
target_carte_frame= [0         510  200         -77.8/180*pi    3.1416/2   77.8/180*pi ];%�ѿ�������xyz rpy ĩ��

ptarget_carte_frame = libpointer('doublePtr', target_carte_frame);%��libpointer����������ָ�� 

ikResultRad= zeros(1,6);
pikResultRad = libpointer('doublePtr', ikResultRad);%��libpointer����������ָ�� 
ret=calllib('export_puma560_fik', 'ik_hrg_puma560', pdh_robot,ptarget_carte_frame,pRefJntPosRad,pikResultRad)

ik_result_arr = get(pikResultRad, 'Value') %һ�����Ž�
clc


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%˫��
L(1)=Link([0       -0.25        0        pi/2      0     -pi/2  ],'modified'); 
L(2)=Link([0       0          0        -pi/2      0    0  ],'modified');
L(3)=Link([0       0           0.2        0          0    -pi/2 ],'modified');
L(4)=Link([0       0.248        0           -pi/2      0     ],'modified');
L(5)=Link([0       0           0           -pi/2       0     ],'modified');
L(6)=Link([0       0            0           pi/2      0     ],'modified');
%                  0.262
p560L=SerialLink(L,'name','LEFT');
p560L.tool=[0 -1 0 0;
               1 0 0 0;
               0 0 1 0.262 ;
               0 0 0 1;]; 
           
R(1)=Link([0       0.25        0        pi/2      0     -pi/2   ],'modified'); 
R(2)=Link([0      0          0           -pi/2      0    0  ],'modified');
R(3)=Link([0       0           0.2        0          0     -pi/2],'modified');
R(4)=Link([0       0.248         0           -pi/2      0     ],'modified');
R(5)=Link([0       0           0           -pi/2       0     ],'modified');
R(6)=Link([0       0           0           pi/2      0     ],'modified');
%                  0.262
p560R=SerialLink(R,'name','RIGHT');
p560R.tool=[0 -1 0 0;
               1 0 0 0;
               0 0 1 0.262 ;
               0 0 0 1;]; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   platform

platform=SerialLink([0 0 0 0],'name','platform','modified');%�����ؽ�
platform.base=[1 0 0 0;
               0 1 0 0;
               0 0 1 0 ;
               0 0 0 1;]; %�����߶�
%platform2=SerialLink([0 0 0 0,0,pi],'name','platform2','modified');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   R
 
% p560R.links(1).d=0.365; %p560R�Ĺؽ�1��������ؽڵľ���
% p560R.links(1).a=0;

pR=SerialLink([platform,p560R],'name','R');

pR.plot([0 0 0 0 0 0 0])
%pR.plot([0 pi/1.5 -pi/6 pi/6 pi/7 -pi/8 pi/7]) %ʵ�ʺͷ��棬�ұ�1��2,3,6�ǶԵ�,4,5����
hold on
%�����Ǻ�˫�۷���Ա�1�ؽ�+��� 2�ؽ�+����.3�ؽ�+����,4�ؽ�+����.5�ؽ�+���ϣ�6�ؽ�+����
%pR.plot([0 0 -pi/2 pi/2  0 0 0])%1�ؽ�+�ϣ��ԣ� 2�ؽ�+���ڣ��ԣ�.3�ؽ�+���ڣ��ԣ�,4�ؽ�+���ϣ��ԣ�.5�ؽ�+���⣨�ԣ���6�ؽ�+���ڣ��ԣ�

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   L

% p560L.links(1).d=0; %p560R�Ĺؽ�1��������ؽڵľ���
% p560L.links(1).a=0;

pL=SerialLink([platform,p560L],'name','L');
%pL.plot([0 pi/2 pi/6 -pi/8 -pi/8 pi/8 pi/8]) ʵ��ģ�� �ͷ���6�� 1 4 5 �� 2 3  6��

%dlmwrite('zhixian.txt',zhixian,'delimiter',' ')
pL.plot([0 0 0 0  0 0 0])
%�����Ǻ�˫�۷���Ա� ����1�ؽ�+��� 2�ؽ�+����.3�ؽ�+����,4�ؽ�+����.5�ؽ�+���ϣ�6�ؽ�+����
%pL.plot([0 0 pi/2 -pi/2  0 0 0])%1�ؽ�+��ǰ 2�ؽ�+���ϣ�3�ؽ�+���ϣ�4�ؽ�+���ڣ�5�ؽ�+���£�6�ؽ�+���ڣ�1235��46��
hold on%([0 0 pi/2 -pi/2  0 0 0]),���ڵ��ۣ�2��3�ؽ�+90��-90ƫ������Ҫ���ϣ���Ҫ��֤�͵��۵�����ʼλ��һ�£�


zhu=load('zhu.txt');
zx=zhu(:,1);
zy=zhu(:,2);
zx=-zx;
zy=-zy;

rr=ones(length(zx),1)*(-98.293);   
pp=ones(length(zx),1)*(20.015);    
yy=ones(length(zx),1)*(113.066);

zz=ones(length(zx),1)*(-0.146);

pR.plot([0 68.4 -68.4 86.4 0 -50.4 0]/180*pi)
hold on
%plot3(zy*0.00015+0.35,zx*0.00015-0.125,zz)

P=[zy*0.0002+0.4,zx*0.0002-0.125,zz,rr,pp,yy];

double_right_X1=zy*0.0002+0.4;
double_right_Y1=zx*0.0002-0.125;
double_right_Z1=zz;
double_wx1=rr;
double_wy1=pp;
double_wz1=yy;

for i=1:length(zx)
    
    [xR1(i),yR1(i),zR1(i),wx1(i),wy1(i),wz1(i)]=right_arm(double_right_X1(i),double_right_Y1(i),double_right_Z1(i),double_wx1(i),double_wy1(i),double_wz1(i));
end
xR1=xR1';
yR1=yR1';
zR1=zR1';
wx1=wx1';
wy1=wy1';
wz1=wz1';

p=[xR1,yR1,zR1,wx1,wy1,wz1];
count=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:length(zx)

    if count==0
    
    ref_jnt_pose_rad= [0 0 0 0 -90/180*pi 0];%�ο���ʼλ��   
    
    elseif count~=0    
 
    ref_jnt_pose_rad=invqq(count,:);  
        
    end 
    
    pRefJntPosRad = libpointer('doublePtr', ref_jnt_pose_rad);%��libpointer����������ָ��  
    
    target_carte_frame=[p(i,1)*1000   p(i,2)*1000  p(i,3)*1000      wx1(i)    wy1(i)  wz1(i)];
    
    ptarget_carte_frame = libpointer('doublePtr', target_carte_frame);%��libpointer����������ָ�� 
  
    ikResultRad= zeros(1,6);
    pikResultRad = libpointer('doublePtr', ikResultRad);%��libpointer����������ָ�� 
    ret=calllib('export_puma560_fik', 'ik_hrg_puma560', pdh_robot,ptarget_carte_frame,pRefJntPosRad,pikResultRad);
    flag(count+1)=ret;
    ik_result_arr = get(pikResultRad, 'Value') ;

    invqq(i,:)=ik_result_arr;

    count=count+1;

end
invqq(:,2)=-invqq(:,2);invqq(:,3)=-invqq(:,3);
bR=invqq;

total_step=length(bR);

c=bR;
c=[zeros(total_step,1),c];
c(:,3)=c(:,3)-pi/2;%��ʼƫ��
c(:,4)=c(:,4)+pi/2;
%c(:,2)=pi/4;%Ϊ����ʾ��ͬƽ�滭Բ+Ϊ����%ע�������ѿ����켣һ��Ҫȥ��������������Ϊ�˻�԰
TR=pR.fkine(c);
PR=transl(TR);
hold on
plot3(PR(:,1),PR(:,2),PR(:,3))
c2=c(1:floor(total_step/50):total_step,:);%������ͼ
pR.plot(c2);


function [x,y,z,wx,wy,wz]=right_arm(double_right_X,double_right_Y,double_right_Z,double_wx,double_wy,double_wz)
%double_right_X=0.0000; double_right_Y=-0.4500+dist;double_right_Z=-0.5100;
dist=0.25;
single_right_X=-double_right_Z;
single_right_Y=double_right_X;
single_right_Z=-(double_right_Y+dist);

double_wx=double_wx/180*pi;
double_wy=double_wy/180*pi;
double_wz=double_wz/180*pi;
%%%%%%%%%%%%%%��˫����������ϵת��������R_base
R_double=rotx(double_wx)*roty(double_wy)*rotz(double_wz);
R_base=rotx(pi/2)*rotz(-pi/2);%��ģ����һ��90��offset
R_single=inv(R_base)*R_double;

single_rpy=tr2rpy(R_single);

x=single_right_X;
y=single_right_Y;
z=single_right_Z;
wx=single_rpy(1);
wy=single_rpy(2);
wz=single_rpy(3);
end


