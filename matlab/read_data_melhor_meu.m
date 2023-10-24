%
% Load data
%
%    version August 26 2017
%
clear all;
data = load('completo_down.txt');
U = data(:,1:3);
Q = data(:,7:10);         %%    [q0,qx,qy,qz]  according to the manual 
X = data(:,11:13);        %%    [x0,x1,x2] = [PHI, THETA, PSI] in the ZYX (PSI (hor),THETA (ver),PHI (tor)) rotation system  (see manual page 73)
F = data(:,4:6);          %%    motor force

%_______________________________Converts to Quaternion____________________________________
a=size(X);
i=1;
while (i<a(1)+1)
    
    O=[X(i,1) X(i,2) X(i,3)]; %para Euler para vector
    O1=[X(i,3) X(i,2) X(i,1)];
    
    Qq=(eul2quat(O*pi/180)); %funciona bem (devolve ZYX) (função do MatLab)
    Qq1(i,:)=(eul2quat(O1*pi/180)); %funciona bem porque ficamos com XYZ

    Cconfirm=my_euler_conv_1(Qq); %confirma os valores
    Cconfirm1=my_euler_conv_1(Qq1); %USAR ESTE, DEVOLVE XYZ!!!
    i=i+1;
end
size(Qq1);    
Q=Qq1;
%______________________________Normalises the FORCE____________________________________-
force=F;
i=1;
n=size(force);
while i<n(1,1)
    if force(i,1)>1023
        force(i,1)=force(i,1)-1023;
    end
    if force(i,2)>1023
        force(i,2)=force(i,2)-1023;
    end
    if force(i,3)>1023
        force(i,3)=force(i,3)-1023;
    end
    i=i+1;
end

F=force;
%force = 100*(force/1023); % para normalizar em relação a max torque
%__________________________________Converts the MOTOR________________________
motor=U;

motor(:, 1)=motor(:,1)-512; %para ficar IO & SO positivo é IO
motor(:, 2)=-(motor(:,2)-512); % para ficar MR & LR positivo é o MR
motor(:, 3)=-(motor(:,3)-512); %para ficar SR & IR positivo é SR

U=motor;
%_________________________________________________________________________
%
%   note that the data X can be obtained from the quaternions Q through
%
%       phi = atan2(2(q0q1+q2q3), 1-2(q1^2+q2^2))
%     theta = asin(2(q0q2-q3q1))
%       psi = atan2(2(q0q3+q1q2), 1-2(q2^2+q3^2))
%
%   let's also check these consistencies  
%   

%% plot 
C = sqrt(sum(U.^2,2));
figure(1); clf;
scatter3(X(:,1),X(:,2),X(:,3),100,C,'filled');
title('Measured Euler angles: x1=PHI(x), x2=THT(y), X3=PSI(z)')
colormap(jet);
xlabel('X = phi');
ylabel('Y = theta');
zlabel('Z = psi');
colorbar;

figure(2); clf;
% euler angles in deg
subplot(3,1,1); plot(X(:,1),'ro','markerfacecolor','r'); xlabel('X1 = phi in deg = tor');
title('Measured Euler angles in deg')
subplot(3,1,2); plot(X(:,2),'bo','markerfacecolor','b'); xlabel('X2 = theta in deg = ver');
subplot(3,1,3); plot(X(:,3),'go','markerfacecolor','g'); xlabel('X3 = psi in deg = hor');

figure(3); clf;
QL = sqrt(sum(Q.^2,2));
scatter3(Q(:,2),Q(:,3),Q(:,4),100, C, 'filled');
title('Measured Quaternions: q1=q0, q2=qx, q3=qy, q4=qz')
colormap(jet);
xlabel('qx (torsion)');
ylabel('qy (vertical)');
zlabel('qz (horizontal)');
colorbar;

figure(4); clf;
% quaternion components
subplot(5,1,1); plot(Q(:,1),'ro','markerfacecolor','r'); xlabel('Q1 = q0');
title('Measured Quaternion components: q1=q0, q2=qx, q3=qy, q4=qz')
subplot(5,1,2); plot(Q(:,2),'bo','markerfacecolor','b'); xlabel('Q2 = qx');
subplot(5,1,3); plot(Q(:,3),'go','markerfacecolor','g'); xlabel('Q3 = qy');
subplot(5,1,4); plot(Q(:,4),'ko','markerfacecolor','k'); xlabel('Q4 = qz');
subplot(5,1,5); plot(QL,'k-'); axis([0 250 0.5 1.5]); xlabel('Q-length');

% test: calculate the four quaternion components from the measured Euler angles
%       using the relationships: with Euler angles psi = x3 = z-axis, 
%           theta = x2 = y-axis and phi = x1 = x-axis  (= the ZYX Euler convention)
%
%        see Wikipedia (and other sources, including the Matlab Robot toolbox) for the conversion
%  
E = X*pi/360; % Euler angles in degrees converted to half-radians. 
%   Note: first rotation (psi) is about z-axis, 2nd rotation (theta) about y-axis and third rotation (phi)  about x-axis.  
QTEST = Q;
%for n=1:250
%   QTEST(n,1) = cos(E(n,1))*cos(E(n,2))*cos(E(n,3)) + sin(E(n,1))*sin(E(n,2))*sin(E(n,3));
%   QTEST(n,2) = sin(E(n,1))*cos(E(n,2))*cos(E(n,3)) - cos(E(n,1))*sin(E(n,2))*sin(E(n,3));
%   QTEST(n,3) = cos(E(n,1))*sin(E(n,2))*cos(E(n,3)) + sin(E(n,1))*cos(E(n,2))*sin(E(n,3));
%   QTEST(n,4) = cos(E(n,1))*cos(E(n,2))*sin(E(n,3)) - sin(E(n,1))*sin(E(n,2))*cos(E(n,3));
%end

figure(5); clf;
QLTEST = sqrt(sum(QTEST.^2,2));
subplot(5,1,1); plot(QTEST(:,1),'ro','markerfacecolor','r'); xlabel('QTEST1 = q0');
title('Calculated Quaternions from measured Euler angles')
subplot(5,1,2); plot(QTEST(:,2),'bo','markerfacecolor','b'); xlabel('QTEST2 = qx');
subplot(5,1,3); plot(QTEST(:,3),'go','markerfacecolor','g'); xlabel('QTEST3 = qy');
subplot(5,1,4); plot(QTEST(:,4),'ko','markerfacecolor','k'); xlabel('QTEST4 = qz');
subplot(5,1,5); plot(QLTEST,'k-'); axis([0 250 0.5 1.5]); xlabel('Q-length');

figure(6); clf;
scatter3(QTEST(:,2),QTEST(:,3),QTEST(:,4),100, C, 'filled');
hold on
title('Calculated Quaternions from measured Euler angles')
colormap(jet);
xlabel('qx-test (torsional)');
ylabel('qy-test (vertical)');
zlabel('qz-test (horizontal)');
colorbar;

XTEST = X;
for n=1:250
    XTEST(n,1) = 180/pi*atan2(2*(Q(n,1)*Q(n,2)+Q(n,3)*Q(n,4)), 1-(Q(n,2)^2+Q(n,3)^2));  % phi     
    XTEST(n,2) = 180/pi*asin(2*(Q(n,1)*Q(n,3) - Q(n,2)*Q(n,4)));                        % theta
    XTEST(n,3) = 180/pi*atan2(2*(Q(n,1)*Q(n,4)+Q(n,2)*Q(n,3)), 1-(Q(n,3)^2+Q(n,4)^2));  % psi    
end

figure(7); clf;
% euler angles in deg  (values of nr. 3 don't seem to make a lot of sense!)
subplot(3,1,1); plot(XTEST(:,1),'ro','markerfacecolor','r'); hold on; xlabel('X1-tst = phi in deg = tor');
title('Calculated Euler angles from Quaternion data (black: difference with Fig. 2)')
subplot(3,1,2); plot(XTEST(:,2),'bo','markerfacecolor','b'); hold on; xlabel('X2-tst = theta in deg = ver');
subplot(3,1,3); plot(XTEST(:,3),'go','markerfacecolor','g'); hold on; xlabel('X3-tst = psi in deg = hor');
XDIFF1 = XTEST - X;
subplot(3,1,1); plot(XDIFF1(:,1),'k.'); subplot(3,1,2); plot(XDIFF1(:,2),'k.'); subplot(3,1,3); plot(XDIFF1(:,3),'k.');

XTEST2 = X;
for n=1:250
    XTEST2(n,1) = 180/pi*atan2(2*(QTEST(n,1)*QTEST(n,2)+QTEST(n,3)*QTEST(n,4)), 1-(QTEST(n,2)^2+QTEST(n,3)^2));  % phi     
    XTEST2(n,2) = 180/pi*asin(2*(QTEST(n,1)*QTEST(n,3) - QTEST(n,2)*QTEST(n,4)));                        % theta
    XTEST2(n,3) = 180/pi*atan2(2*(QTEST(n,1)*QTEST(n,4)+QTEST(n,2)*QTEST(n,3)), 1-(QTEST(n,3)^2+QTEST(n,4)^2));  % psi    
end

figure(8); clf;
subplot(3,1,1); plot(XTEST2(:,1),'ro','markerfacecolor','r'); hold on; xlabel('X1-tst2 = phi in deg = tor');
title('Calculated Euler angles from calculated Quaternions (black: difference with Fig. 2)')
subplot(3,1,2); plot(XTEST2(:,2),'bo','markerfacecolor','b'); hold on; xlabel('X2-tst2 = theta in deg = ver');
subplot(3,1,3); plot(XTEST2(:,3),'go','markerfacecolor','g'); hold on; xlabel('X3-tst2 = psi in deg = hor');
XDIFF2 = XTEST2 - X;
subplot(3,1,1); plot(XDIFF2(:,1),'k.'); subplot(3,1,2); plot(XDIFF2(:,2),'k.'); subplot(3,1,3); plot(XDIFF2(:,3),'k.');

figure(9); clf;
subplot(2,1,1);
plot(X(:,1), 'ro', 'markerfacecolor','r'); hold on; plot(X(:,2), 'bo','markerfacecolor','b'); plot(X(:,3),'go', 'markerfacecolor','g');
title('Measured Euler angles and Calculated Quaternions');
legend('phi','theta','psi');
xlabel('Trial Number')
ylabel('Euler angles (deg)')

subplot(2,1,2);
plot(QTEST(:,1)-0.75,'ko','markerfacecolor','k'); hold on; plot(QTEST(:,2),'ro','markerfacecolor','r');
plot(QTEST(:,3),'bo','markerfacecolor','b'); plot(QTEST(:,4),'go','markerfacecolor','g');
legend('q0-0.75','qx','qy','qz');
xlabel('Trial Number')
ylabel('Quaternion half-radians')
axis([0 250 -0.3 0.3])

figure(10); clf;
subplot(2,1,1);
plot(X(:,1), 'ro', 'markerfacecolor','r'); hold on; plot(X(:,2), 'bo','markerfacecolor','b'); plot(X(:,3),'go', 'markerfacecolor','g');
title('Measured Euler angles and Measured Quaternions');
legend('phi','theta','psi');
xlabel('Trial Number')
ylabel('Euler angles (deg)')

subplot(2,1,2);
plot(Q(:,1),'ko','markerfacecolor','k'); hold on; plot(Q(:,2),'ro','markerfacecolor','r');
plot(Q(:,3),'bo','markerfacecolor','b'); plot(Q(:,4),'go','markerfacecolor','g');
legend('q0','qx','qy','qz');
xlabel('Trial Number')
ylabel('Quaternion half-radians')
axis([0 250 -1.1 1.1])
%
%          next step: find planes in the data, described by
%                      qx = a qz + b qy     
%            under the following two constraints:
%                1) deviation from the plane is minimal
%                2) total cost of the motor action C is minimal
%        the vector (1, b, a)  is then perpendicular to Listing's plane
%             and after a rotation of the data, this vector should return to
%             (1, 0, 0), describing the plane:     qx = 0
%
%  How are we going to do this? 
%     
%    Transform the quaternion data to (Euler's) rotation vectors:   
%          r = q/q0 = tan(rho/2)n
%    (vectors with only three components, and slightly simpler multiplication rules than quaternions)
ROT = QTEST(:,2:4);
ntrials = size(ROT,1);
for n=1:3
    ROT(:,n) = ROT(:,n)./QTEST(:,1);
end
%
%    1) first we need to characterize the single-axis effects of motors 1, 2, and 3 on
%    the rotation of the eye from all initial eye positions. Perhaps it's
%    possible to fit a function to this:   
%          input (qx0, qy0, qz0)   -> MODEL MOTOR M -> output (qxM, qyM, qzM)
%           the relationships should be obtained from the data matrix
%
%   following Miguel's suggestion, we make a three column matrix,
%   indicating the action of motors 1, 2, 3 respectively for each trial:
%    loop is:     for motor 1      right-vertical (let's call it RALP)
%                    for motor 2   left-vertical  (LARP)
%                      for motor 3  horizontal    (HOR)
%
% let's first plot the trials in xz and yz format:    
%
figure(11); clf;
plot([-0.05 0.05],[0, 0],'k-'); hold on; plot([0,0], [-0.05 0.05],'k-'); 
axis([-0.15 0.15 -0.15 0.15]);
xlabel('ry [rad/2]'); ylabel('rz [rad/2]');
title('Rotation vectors YZ plot');
plot( ROT(1,2), ROT(1,3), 'ko','markersize',10, 'markerfacecolor','k'); % first trial
plot( ROT(ntrials,2), ROT(ntrials,3), 'ko','markersize',10, 'markerfacecolor','b'); % last trial

figure(12); clf;
plot([-0.05 0.05],[0, 0],'k-'); hold on; plot([0,0], [-0.05 0.05],'k-'); 
axis([-0.2 0.15 -0.15 0.15]);
xlabel('rx [rad/2]'); ylabel('rz [rad/2]');
title('Rotation vectors XZ plot');
plot( ROT(1,1), ROT(1,3), 'ko','markersize',10, 'markerfacecolor','k'); 
plot( ROT(ntrials,1), ROT(ntrials,3), 'ko','markersize',10, 'markerfacecolor','b'); 

figure(11);
for n=1:ntrials-1  
    plot( ROT(n,2), ROT(n,3), 'k.','markersize',8); 
    plot( [ROT(n,2), ROT(n+1,2)],[ROT(n,3), ROT(n+1,3)], 'r-');
end
figure(12);
for n=1:ntrials-1   
    plot( ROT(n,1), ROT(n,3), 'k.','markersize',8); 
    plot( [ROT(n,1), ROT(n+1,1)],[ROT(n,3), ROT(n+1,3)], 'r-');    
end
    
%
%   Calculate the angular rotation vectors, qrot, that bring the eye from r_n (eye orientation in trial n) to
%   r_(n+1):  these are given by:   qrot = r_(n+1) o r_n^-1 = 
%               [r_(n+1) - r_n + r_(n+1) x r_n] / (1 - r_n+1 dot r_n)
%
% NOTE: each trial was a
% rotation from the reference position (aribtraily assumed to be (0,0,0) )! Not a rotation trial n to n+1 !
QROT = ROT(1:ntrials-1,:);
QDIFF = QROT; QOUT = QROT; QDOT = QROT(:,1);
for n=1:ntrials-1
    QDIFF(n,:) = ROT(n+1,:) - ROT(n, :); % difference vector 
    QDOT(n) = ROT(n,1)*ROT(n+1,1)+ROT(n,2)*ROT(n+1,2)+ROT(n,3)*ROT(n+1,3); % dot product
    QOUT(n,1) = ROT(n+1,2)*ROT(n,3) - ROT(n+1,3)*ROT(n,2);    % outer product x
    QOUT(n,2) = ROT(n+1,3)*ROT(n,1) - ROT(n+1,1)*ROT(n,3);    % outer product y
    QOUT(n,3) = ROT(n+1,1)*ROT(n,2) - ROT(n+1,2)*ROT(n,1);    % outer product z
    
    QROT(n,:) = (QDIFF(n,:) + QOUT(n,:))/(1 - QDOT(n));   % the single-axis rotation from trial n to n+1
end

%
%    1)  we  have all reachable eye-orientations for each trial, obtained
%        from an (unknown) reference position, (r0x, r0y, r0z). They do not
%        yet lie in a plane, because there have been lots of motor commands that
%        create additional torsional eye rotations. 
%        The question therefore boils down to: Find a planar cross section
%        through the data, such it covers all gaze directions (rEy, rEz),
%        and select the measured points around the plane with a minimum scatter perpendicular to that
%        plane. Then calculate the total motor effort (normalized for the number of points included).
%        We can thus build a model, from which we can calculate the desired optimal motor commands, such that the
%        eye orientations remain in a plane and the effort of the control is minimized.
%
%     So the generalized model is:
%     input (r0x, r0y, r0z)   ->  MOTORS M1,2,3 -> output (rEx, rEy, rEz)
%             with rEx + b rEy + c rEz = constant (a plane) AND  minimum total motor effort across all gaze directions (rEy, rEz).
%             note that a = a(r0x, r0y, r0z),  b = b(r0x, r0y, r0z)   and c = c(r0x, r0y, r0z)
%
%     Let's do it brute force:  (later perhaps fminsearch) 
%
%     1) define a plane:    rx = a + b ry + c rz   for a given [a,b,c]    
%        a in [-0.15 to +0.15]     b,c in [-2 to +2] as three nested loops   
%     2) select all data points that are close to this plane (for this, a
%        criterion C is needed, e.g. C<0.03 which is about 3 deg) )
%     3) calculate the total mean absolute motor effort for these data
%        points, provided N>=10
%     4) repeat 1-3
%     5) plot the result (plane and data points in xz, yz format) with the
%        parameters of the 'best' plane. 
%     6) Around the optimal parameters a finer search may be performed next.
%

% let's first generate a test-plane and plot it, to see where it lies
a=-0.05; b = 0.4; c=-0.4;
plane = zeros(961,3);  v=0; w=0;
for ry=-0.15:0.01:0.15
    v=v+1;
    for rz=-0.15:0.01:0.15
        w=w+1;
        n=w+31*(v-1);
        rx = a+b*ry+c*rz;
        plane(n,1) = rx;
        plane(n,2)=ry;
        plane(n,3)=rz;
    end
end
figure(13); clf; 
scatter3(plane(:,2),plane(:,3),plane(:,1),10, 'r', 'filled');
title('Just a test plane')
%
%   the distance of point (x1,y1,z1) to the plane ax+by+cz+d=0 is given by
%       D = abs(ax1+by1+cz1+d)/sqrt(a^2+b^2+c^2)
%
%    our plane is defined by:    x-by-cz-a = 0, so 
%    D = abs(x1-by1-cz1-a)/sqrt(b^2+c^2+1)
% 
CRIT = 0.03; n=0;
PlaneSearch = zeros(15059, 6);   % maybe include also the achieved oculomotor range?  Structure?
n=0;
for a=-0.15:0.03:+0.15       % estava -0.15:0.03:+0.15 numero de planos: 20535
    for b=-3.6:0.2:3.6       % estava -2.0:0.4:2.0
        for c=-3.6:0.2:3.6   % estava -2.0:0.4:2.0
            n=n+1;
            % calculate the distance of all points to the plane and select
            % those points for which this distance is smaller than the
            % criterion
            norm = sqrt(1+b^2+c^2);
            Distance = abs(ROT(:,1)-b*ROT(:,2)-c*ROT(:,3)-a)/norm;
            SelectPoints = ROT(Distance<CRIT,:);
            SelectEul = X(Distance<CRIT,:);
            SelectMot = U(Distance<CRIT,:);
            SelectFor = F(Distance<CRIT,:);
            %
            %  calculate the total mean motor effort for these selected pointswithin 3 deg from the plane 
            %
            %if size(SelectMot,1) >= 10   % a minimum of 10 points near the plane
            %   Effort = sqrt(sum(sum(SelectMot.^2,2)))/size(SelectMot,1);
            %else
            %    Effort = 10000;
            %end
            
            Effort=sum(sum(SelectFor)/size(SelectFor,1)); % Sums each motor effort and divides it by the number of points
            
            %
            %  fill planesearch matrix with a,b,c,NrPoints,Effort
            %
            PlaneSearch(n,1)=a; PlaneSearch(n,2)=b; PlaneSearch(n,3)=c; PlaneSearch(n,4)=size(SelectMot,1);
            PlaneSearch(n,5)=Effort; PlaneSearch(n,6) = std(Distance);
        end
    end
end       % do this for 1331 different planes ....

     
          
%--------------------PPARAMETERIZATION----------------------

     figure(97);
     aux=sqrt(PlaneSearch(:,1).*PlaneSearch(:,1))*10;
     scatter3(PlaneSearch(:,2),PlaneSearch(:,3),PlaneSearch(:,5),[],aux,'filled')
     title('Effort vs Plane Coefficients Parameterization x=a+bY+cZ');
     xlabel('b')
     ylabel('c')
     zlabel('Effort')


     aux=(PlaneSearch(:,1)==0); %choose offset = 0
     figure(98);
     scatter3(PlaneSearch(aux,2),PlaneSearch(aux,3),PlaneSearch(aux,5),[],PlaneSearch(aux,5),'filled')
     title('Effort vs Plane Coefficients Parameterization x=bY+cZ');
     xlabel('b')
     ylabel('c')
     zlabel('Effort')
     
     tri = delaunay(PlaneSearch(aux,2),PlaneSearch(aux,3));
     
     figure(99);
     h=trisurf(tri,PlaneSearch(aux,2),PlaneSearch(aux,3),PlaneSearch(aux,5));
     axis vis3d
     lighting phong
     shading interp
     colormap default
     title('Effort vs Plane Coefficients Parameterization x=bY+cZ');
     xlabel('b')
     ylabel('c')
     zlabel('Effort')
     colorbar EastOutside
     
     
     aux_size=size(PlaneSearch(aux,2));
     N=zeros(aux_size(1),3);
     N(:,1)=-1; N(:,2)=PlaneSearch(aux,2); N(:,3)=PlaneSearch(aux,3);
     norm=sqrt(N(:,1).^2+N(:,2).^2+N(:,3).^2);
     
     x_ct=N(:,2); y_ct=N(:,3); z_ct=PlaneSearch(aux,5);
     
     nub=1;
     for a=1:1:37
         for b=1:1:37
             vvv(a,b)=z_ct(nub);
             nub=nub+1;
         end
     end
     
     xvx=-3.6:0.2:3.6; yvy=xvx; %-3.6:0.2:3.6
     figure(44)
     contour(xvx,yvy,vvv,'showtext','on');
     xlabel('c')
     ylabel('b')
     %title('')

     figure(46)
     surf(xvx,yvy,vvv);
     xlabel('c')
     ylabel('b')
     zlabel('Effort')
     
     
     
     nnn=zeros(aux_size(1),3);
     nnn(:,1)=N(:,1)./norm(:,1); nnn(:,2)=N(:,2)./norm(:,1); nnn(:,3)=N(:,3)./norm(:,1);
     
     figure(100);
     scatter3(nnn(:,2),nnn(:,3),PlaneSearch(aux,5),[],PlaneSearch(aux,5),'filled')
     title('Effort vs Plane Coefficients Parameterization ax+by+bz=0 with normalized vector (a=sqrt(b^2+c^2))');
     xlabel('b')
     ylabel('c')
     zlabel('Effort')
     
     tri = delaunay(nnn(:,2),nnn(:,3));
     
     figure(101);
     h=trisurf(tri,nnn(:,2),nnn(:,3),PlaneSearch(aux,5));
     axis vis3d
     %lighting phong
     %shading interp
     colorbar EastOutside
     %colormap default
     title('Effort vs Plane Coefficients Parameterization ax+by+bz=0 with normalized vector (a=sqrt(b^2+c^2))');
     xlabel('b')
     ylabel('c')
     zlabel('Effort')
     
     
     
     %------------------END OF
     %PARAMETERIZATION------------------------------

     I=find(PlaneSearch(:,5)==min(PlaneSearch(:,5)));
     minenergy=PlaneSearch(I,:)  % the plane for which the motors needed minimum effort....  
     
     I3=find(PlaneSearch(:,5)==max(PlaneSearch(:,5)));
     maxenergy=PlaneSearch(I3,:)  % the plane for which the motors needed maximum effort....
     
     I2=find(PlaneSearch(:,4)==max(PlaneSearch(:,4))); 
     maxpoints=PlaneSearch(I2,:) % the plane that has most points
     
     zero_zero=((PlaneSearch(:,1)==0)&(PlaneSearch(:,2)==0)&(PlaneSearch(:,3)==0));
     zero_plane=PlaneSearch(zero_zero,:)
     
     %zero_dis=X(:,1);
     %criterio=3;
     %SelectEulmax = X((zero_dis<criterio)&(zero_dis>-criterio),:);
     

     figure(14); clf;
     plot(PlaneSearch(:,5),'k.','markersize',8); hold on; %axis([0 1400 30 600]);
     title('Effort distribution for all 1331 planes');
     plot(I, PlaneSearch(I,5),'ko','markerfacecolor','r','markersize',12);
     text(I-25, PlaneSearch(I,5)-15, 'Min Energy')
     plot(I2, PlaneSearch(I2,5),'ko','markerfacecolor','g','markersize',12);
     text(I2-25, PlaneSearch(I2,5)-15, 'Max Points')
%
%   plot the data points for this plane!
%
a=PlaneSearch(I,1); b=PlaneSearch(I,2); c=PlaneSearch(I,3);
norm = sqrt(1+b^2+c^2);
Distance = abs(ROT(:,1)-b*ROT(:,2)-c*ROT(:,3)-a)/norm;
SelectPoints = ROT(Distance<CRIT,:);
SelectEulmin = X(Distance<CRIT,:);
D = Distance(Distance<CRIT);

figure(15); clf;
scatter3(SelectPoints(:,2), SelectPoints(:,3), SelectPoints(:,1), 100, 'r','filled');
hold on
plane = zeros(961,3);  v=0; w=0;
for ry=-0.15:0.01:0.15
    v=v+1;
    for rz=-0.15:0.01:0.15
        w=w+1;
        n=w+31*(v-1);
        rx = a+b*ry+c*rz;
        plane(n,1) = rx;
        plane(n,2)=ry;
        plane(n,3)=rz;
    end
end
scatter3(plane(:,2),plane(:,3),plane(:,1),10, 'k', 'filled');
s=sprintf('rx = %f + %f*ry + %f*rz   CRIT~%2d deg  Min. Effort = %f  ',a,b,c,100*CRIT,PlaneSearch(I,5));
title(s);
xlabel('y[rad]')
ylabel('z[rad]')
zlabel('x[rad]')

figure(16); clf;
plot([-0.05 0.05],[0, 0],'k-'); hold on; plot([0,0], [-0.05 0.05],'k-'); 
axis([-0.15 0.15 -0.15 0.15]);
xlabel('rz [rad/2]'); ylabel('ry [rad/2]');
title('Rotation vectors ZY plot around Optimal Plane');
plot(SelectPoints(:,3), SelectPoints(:,2), 'ko','markersize',10, 'markerfacecolor','b'); 

figure(17); clf;
plot([-0.05 0.05],[0, 0],'k-'); hold on; plot([0,0], [-0.05 0.05],'k-'); 
axis([-0.2 0.15 -0.15 0.15]);
xlabel('rx [rad/2]'); ylabel('ry [rad/2]');
title('Rotation vectors XY plot around Optimal Plane');
plot( SelectPoints(:,1), SelectPoints(:,2), 'ko','markersize',10, 'markerfacecolor','b'); 

%   plot the data points for the plane with max points!!
%
a=PlaneSearch(zero_zero,1); b=PlaneSearch(zero_zero,2); c=PlaneSearch(zero_zero,3);
norm = sqrt(1+b^2+c^2);
Distance = abs(ROT(:,1)-b*ROT(:,2)-c*ROT(:,3)-a)/norm;
SelectPoints = ROT(Distance<CRIT,:);
SelectEulmax = X(Distance<CRIT,:);
%D = Distance(Distance<CRIT);

figure(18); clf;
scatter3(SelectPoints(:,2), SelectPoints(:,3), SelectPoints(:,1), 100, 'r','filled');
hold on
plane = zeros(961,3);  v=0; w=0;
for ry=-0.15:0.01:0.15
    v=v+1;
    for rz=-0.15:0.01:0.15
        w=w+1;
        n=w+31*(v-1);
        rx = a+b*ry+c*rz;
        plane(n,1) = rx;
        plane(n,2)=ry;
        plane(n,3)=rz;
    end
end
scatter3(plane(:,2),plane(:,3),plane(:,1),10, 'k', 'filled');
s=sprintf('rx = %f + %f*ry + %f*rz   CRIT~%2d deg  Effort = %f  ',a,b,c,100*CRIT,PlaneSearch(I2,5));
title(s);
xlabel('y[rad]')
ylabel('z[rad]')
zlabel('x[rad]')

figure(19); clf;
plot([-0.05 0.05],[0, 0],'k-'); hold on; plot([0,0], [-0.05 0.05],'k-'); 
axis([-0.15 0.15 -0.15 0.15]);
xlabel('rz [rad/2]'); ylabel('ry [rad/2]');
title('Rotation vectors ZY plot around Optimal Plane');
plot(SelectPoints(:,3), SelectPoints(:,2), 'ko','markersize',10, 'markerfacecolor','b'); 

figure(20); clf;
plot([-0.05 0.05],[0, 0],'k-'); hold on; plot([0,0], [-0.05 0.05],'k-'); 
axis([-0.2 0.15 -0.15 0.15]);
xlabel('rx [rad/2]'); ylabel('ry [rad/2]');
title('Rotation vectors XY plot around Optimal Plane');
plot( SelectPoints(:,1), SelectPoints(:,2), 'ko','markersize',10, 'markerfacecolor','b'); 

%--------------------------EULER GRAPHS------------------------------
figure (21)
scatter(SelectPoints(:,3),SelectPoints(:,2),'filled')
title('YZ Plane (Frontal View)');
xlabel('Rz')
ylabel('Ry')
axis([-0.3 0.3 -0.3 0.3])

figure (22)
scatter(SelectPoints(:,1),SelectPoints(:,2),'filled')
title('XY Plane (Top View)');
xlabel('Rx')
ylabel('Ry')
axis([-0.3 0.3 -0.3 0.3])

figure (23)
scatter(SelectPoints(:,1),SelectPoints(:,3),'filled')
title('XZ Plane (Side View)');
xlabel('Rx')
ylabel('Rz')
axis([-0.3 0.3 -0.3 0.3])

figure(24); clf;
scatter3(SelectPoints(:,1),SelectPoints(:,2),SelectPoints(:,3))
title('Euler for min energy plane')
xlabel('X(º)')
ylabel('Y(º)')
zlabel('Z(º)')
axis([-0.3 0.3 -0.3 0.3 -0.3 0.3])

figure (25)
scatter(SelectEulmax(:,3),SelectEulmax(:,2),'filled')
title('YZ Plane');
xlabel('Horizontal Component [deg]')
ylabel('Vertical Component [deg]')
axis([-30 30 -30 30])

figure (26)
scatter(SelectEulmax(:,1),SelectEulmax(:,2),'filled')
title('XY Plane');
xlabel('Torsional Component [deg]')
ylabel('Vertical Component [deg]')
axis([-30 30 -30 30])

figure (27)
scatter(SelectEulmax(:,1),SelectEulmax(:,3),'filled')
title('XZ Plane');
xlabel('Torsional Component [deg]')
ylabel('Horizontal Component [deg]')
axis([-30 30 -30 30])

figure(28)
scatter3(SelectEulmax(:,1),SelectEulmax(:,2),SelectEulmax(:,3))
title('Euler for zero_zero points plane')
xlabel('X(º)')
ylabel('Y(º)')
zlabel('Z(º)')
axis([-30 30 -30 30 -30 30])


%
%   next, analyze the motor commands belonging to the points near
%   the 'optimal' plane
%
%   Further: an alternative (and more unbiased) search strategy is the following: 
%         - draw 100 random points from the rotation-vector data base, such that they cover
%           (approximately) the entire (y,z) gaze-direction range (ignore x).
%         - calculate the mean motor effort across these 100 points
%         - do this 1000 times (bootstrapping)
%         - find the minimum motor effort, and see what kind or surface
%           in 3D space emerges. Is it (close to) a plane? Same as above?
%
%