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

jnt_pose_rad= [0 0 0 0 -90/180*pi 0];%关节角度

pfAxesPosRad = libpointer('doublePtr', jnt_pose_rad);%用libpointer函数来构造指针 

xyz_rpy_result_temp= zeros(1,6);
pxyz_rpy_rad = libpointer('doublePtr', xyz_rpy_result_temp);%用libpointer函数来构造指针 
ret=calllib('export_puma560_fik', 'fk_hrg_puma560_DefTcs', pdh_robot,pfAxesPosRad,pxyz_rpy_rad)

fk_result_arr = get(pxyz_rpy_rad, 'Value')


pdh_robot = libpointer('doublePtr', dh);%用libpointer函数来构造指针 
%get(pv, 'Value')%显示指针的内容 
ref_jnt_pose_rad= [0 0 0 0 -90/180*pi 0];%参考初始位置
pRefJntPosRad = libpointer('doublePtr', ref_jnt_pose_rad);%用libpointer函数来构造指针 

target_carte_frame= [248.0000         0  -62.0000         0    3.1416   -1.5708];%笛卡尔坐标xyz rpy 末端
%测试
target_carte_frame= [0         510  200         -77.8/180*pi    3.1416/2   77.8/180*pi ];%笛卡尔坐标xyz rpy 末端

ptarget_carte_frame = libpointer('doublePtr', target_carte_frame);%用libpointer函数来构造指针 

ikResultRad= zeros(1,6);
pikResultRad = libpointer('doublePtr', ikResultRad);%用libpointer函数来构造指针 
ret=calllib('export_puma560_fik', 'ik_hrg_puma560', pdh_robot,ptarget_carte_frame,pRefJntPosRad,pikResultRad)

ik_result_arr = get(pikResultRad, 'Value') %一组最优解
clc


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%双臂
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

platform=SerialLink([0 0 0 0],'name','platform','modified');%腰部关节
platform.base=[1 0 0 0;
               0 1 0 0;
               0 0 1 0 ;
               0 0 0 1;]; %基座高度
%platform2=SerialLink([0 0 0 0,0,pi],'name','platform2','modified');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   R
 
% p560R.links(1).d=0.365; %p560R的关节1距离基座关节的距离
% p560R.links(1).a=0;

pR=SerialLink([platform,p560R],'name','R');

pR.plot([0 0 0 0 0 0 0])
%pR.plot([0 pi/1.5 -pi/6 pi/6 pi/7 -pi/8 pi/7]) %实际和仿真，右臂1，2,3,6是对的,4,5反了
hold on
%这里是和双臂仿真对比1关节+向后 2关节+向下.3关节+向下,4关节+向内.5关节+向上，6关节+向内
%pR.plot([0 0 -pi/2 pi/2  0 0 0])%1关节+上（对） 2关节+向内（对）.3关节+向内（对）,4关节+向上（对）.5关节+向外（对），6关节+向内（对）

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   L

% p560L.links(1).d=0; %p560R的关节1距离基座关节的距离
% p560L.links(1).a=0;

pL=SerialLink([platform,p560L],'name','L');
%pL.plot([0 pi/2 pi/6 -pi/8 -pi/8 pi/8 pi/8]) 实际模型 和仿真6轴 1 4 5 反 2 3  6正

%dlmwrite('zhixian.txt',zhixian,'delimiter',' ')
pL.plot([0 0 0 0  0 0 0])
%这里是和双臂仿真对比 单笔1关节+向后 2关节+向下.3关节+向下,4关节+向内.5关节+向上，6关节+向内
%pL.plot([0 0 pi/2 -pi/2  0 0 0])%1关节+向前 2关节+向上，3关节+向上，4关节+向内，5关节+向下，6关节+向内，1235反46正
hold on%([0 0 pi/2 -pi/2  0 0 0]),对于单臂，2，3关节+90，-90偏移量需要加上，需要保证和单臂的逆解初始位置一致！


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
    
    ref_jnt_pose_rad= [0 0 0 0 -90/180*pi 0];%参考初始位置   
    
    elseif count~=0    
 
    ref_jnt_pose_rad=invqq(count,:);  
        
    end 
    
    pRefJntPosRad = libpointer('doublePtr', ref_jnt_pose_rad);%用libpointer函数来构造指针  
    
    target_carte_frame=[p(i,1)*1000   p(i,2)*1000  p(i,3)*1000      wx1(i)    wy1(i)  wz1(i)];
    
    ptarget_carte_frame = libpointer('doublePtr', target_carte_frame);%用libpointer函数来构造指针 
  
    ikResultRad= zeros(1,6);
    pikResultRad = libpointer('doublePtr', ikResultRad);%用libpointer函数来构造指针 
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
c(:,3)=c(:,3)-pi/2;%初始偏移
c(:,4)=c(:,4)+pi/2;
%c(:,2)=pi/4;%为了显示不同平面画圆+为向上%注意其他笛卡尔轨迹一定要去掉这个，这个就是为了花园
TR=pR.fkine(c);
PR=transl(TR);
hold on
plot3(PR(:,1),PR(:,2),PR(:,3))
c2=c(1:floor(total_step/50):total_step,:);%单独画图
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
%%%%%%%%%%%%%%从双臂世界坐标系转换到单臂R_base
R_double=rotx(double_wx)*roty(double_wy)*rotz(double_wz);
R_base=rotx(pi/2)*rotz(-pi/2);%建模多了一个90的offset
R_single=inv(R_base)*R_double;

single_rpy=tr2rpy(R_single);

x=single_right_X;
y=single_right_Y;
z=single_right_Z;
wx=single_rpy(1);
wy=single_rpy(2);
wz=single_rpy(3);
end


