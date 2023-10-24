%% Load data
clear all;
data = load('LTI_dyn_step_100.txt');
data2 = load('LTI_dyn_step_200.txt');
data3 = load('LTI_dyn_step_300.txt');
data4 = load('LTI_dyn_step_400.txt');

t=0:10:1000;

M=data(:,1:3)-512; M2=data2(:,1:3)-512; M3=data3(:,1:3)-512; M4=data4(:,1:3)-512;
M(:,3)=-M(:,3); M2(:,3)=-M2(:,3); M3(:,3)=-M3(:,3); M4(:,3)=-M4(:,3); %passa para positivo
MM=M*90/1024; MM2=M2*90/1024; MM3=M3*90/1024; MM4=M4*90/1024; %Change motor data to deg

%-----------CONVERT VELOCITY TO deg/sec----------------------------
V=data(:,4:6); V2=data2(:,4:6); V3=data3(:,4:6); V4=data4(:,4:6);
i=1;
aux=size(V(:,1));
while i<aux(1,1)
    if(V(i,3)>500)
        V(i,3)=V(i,3)-1024;
    end
    i=i+1;
end
VV=V*0.111*6; %para converter para deg/sec
i=1;
aux=size(V2(:,1));
while i<aux(1,1)
    if(V2(i,3)>500)
        V2(i,3)=V2(i,3)-1024;
    end
    i=i+1;
end
VV2=V2*0.111*6; %para converter para deg/sec
i=1;
aux=size(V3(:,1));
while i<aux(1,1)
    if(V3(i,3)>500)
        V3(i,3)=V3(i,3)-1024;
    end
    i=i+1;
end
VV3=V3*0.111*6; %para converter para deg/sec
i=1;
aux=size(V4(:,1));
while i<aux(1,1)
    if(V4(i,3)>500)
        V4(i,3)=V4(i,3)-1024;
    end
    i=i+1;
end
VV4=V4*0.111*6; %para converter para deg/sec

X = data(:,11:13); X2 = data2(:,11:13); X3 = data3(:,11:13); X4 = data4(:,11:13);    
nn=size(X(:,1));
t=0:10:(10*(nn(1,1)-1));

%REMOVE MEANS
X(:,3)=X(:,3)-X(1,3); X2(:,3)=X2(:,3)-X2(1,3); X3(:,3)=X3(:,3)-X3(1,3); X4(:,3)=X4(:,3)-X4(1,3);
X(:,1)=X(:,1)-X(1,1); X2(:,1)=X2(:,1)-X2(1,1); X3(:,1)=X3(:,1)-X3(1,1); X4(:,1)=X4(:,1)-X4(1,1);

figure(1); clf;
subplot(3,1,1)
plot(t,X(1:nn(1,1),3),'Displayname', 's'); hold on; plot(t,X2(1:nn(1,1),3),'Displayname', '2s'); 
plot(t,X3(1:nn(1,1),3),'Displayname', '3s'); plot(t,X4(1:nn(1,1),3),'Displayname', '4s');
axis([0 200 -6 6])
ylabel('Horizontal Comp. [deg]');
title('Vertical Saccade with different Step Input (s)');
legend('show')

subplot(3,1,2)
%plot(t,MM(1:nn(1,1),1),'Displayname', 's'); hold on;
%plot(t,MM2(1:nn(1,1),1),'Displayname', '2*s')
%plot(t,MM3(1:nn(1,1),1),'Displayname', '3*s')
%plot(t,MM4(1:nn(1,1),1),'Displayname', '4*s')
%ylabel('Motor Rotation [deg]');
%legend('show
plot(t,X(1:nn(1,1),1),'Displayname', 's'); hold on; plot(t,X2(1:nn(1,1),1),'Displayname', '2s'); 
plot(t,X3(1:nn(1,1),1),'Displayname', '3s'); plot(t,X4(1:nn(1,1),1),'Displayname', '4s');
axis([0 200 -6 6])
ylabel('Torsional Comp. [deg]');
%title('Vertical Saccade with different Step Input (s)');
%legend('show')

subplot(3,1,3)
%plot(t,VV(1:nn(1,1),1),'Displayname', 'vel 135'); hold on;
%plot(t,VV2(1:nn(1,1),1),'Displayname', 'vel 265')
%plot(t,VV3(1:nn(1,1),1),'Displayname', 'vel 340')
%plot(t,VV4(1:nn(1,1),1),'Displayname', 'vel 680')
%ylabel('Motor Speed [deg/s]'); 
%legend('show')
plot(t,MM(1:nn(1,1),1),'Displayname', 's'); hold on;
plot(t,MM2(1:nn(1,1),1),'Displayname', '2*s')
plot(t,MM3(1:nn(1,1),1),'Displayname', '3*s')
plot(t,MM4(1:nn(1,1),1),'Displayname', '4*s')
ylabel('Motor Rotation [deg]');
xlabel('Time [ms]');
axis([0 200 -20 20])
%legend('show')

figure(2); clf;
subplot(3,1,1)
plot(t,X(1:nn(1,1),1),'Displayname', 's'); hold on; plot(t,X2(1:nn(1,1),1),'Displayname', '2s'); 
plot(t,X3(1:nn(1,1),1),'Displayname', '3s'); plot(t,X4(1:nn(1,1),1),'Displayname', '4s');
%axis([0 300 -3 3])
ylabel('Torsional Component [deg]');
title('Vertical Saccade with different Step Input (s)');
legend('show')

subplot(3,1,2)
plot(t,MM(1:nn(1,1),1),'Displayname', 's'); hold on;
plot(t,MM2(1:nn(1,1),1),'Displayname', '2*s')
plot(t,MM3(1:nn(1,1),1),'Displayname', '3*s')
plot(t,MM4(1:nn(1,1),1),'Displayname', '4*s')
ylabel('Motor Rotation [deg]');
%legend('show')

subplot(3,1,3)
plot(t,VV(1:nn(1,1),1),'Displayname', 'vel 135'); hold on;
plot(t,VV2(1:nn(1,1),1),'Displayname', 'vel 265')
plot(t,VV3(1:nn(1,1),1),'Displayname', 'vel 340')
plot(t,VV4(1:nn(1,1),1),'Displayname', 'vel 680')
ylabel('Motor Speed [deg/s]');
xlabel('Time [ms]'); 
%legend('show')

figure(3); clf;
subplot(3,1,1)
plot(t,X(1:nn(1,1),2),'Displayname', 's'); hold on; plot(t,X2(1:nn(1,1),2),'Displayname', '2s'); 
plot(t,X3(1:nn(1,1),2),'Displayname', '3s'); plot(t,X4(1:nn(1,1),2),'Displayname', '4s');
%axis([0 300 -3 3])
ylabel('Vertical Component [deg]');
title('Vertical Saccade with different Step Input (s)');
legend('show')

subplot(3,1,2)
plot(t,MM(1:nn(1,1),1),'Displayname', 's'); hold on;
plot(t,MM2(1:nn(1,1),1),'Displayname', '2*s')
plot(t,MM3(1:nn(1,1),1),'Displayname', '3*s')
plot(t,MM4(1:nn(1,1),1),'Displayname', '4*s')
ylabel('Motor Rotation [deg]');
%legend('show')

subplot(3,1,3)
plot(t,VV(1:nn(1,1),1),'Displayname', 'vel 135'); hold on;
plot(t,VV2(1:nn(1,1),1),'Displayname', 'vel 265')
plot(t,VV3(1:nn(1,1),1),'Displayname', 'vel 340')
plot(t,VV4(1:nn(1,1),1),'Displayname', 'vel 680')
ylabel('Motor Speed [deg/s]');
xlabel('Time [ms]'); 
%legend('show')
