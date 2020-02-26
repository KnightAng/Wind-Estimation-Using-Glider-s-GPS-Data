% 2020.02.26
% created by MING KE
% This code is from my work "Thermal identification", here I write it
% to show the result after the estimation.
% Based on the glider GPS data, v_wind can be estimated.  
% version 1 esitmate wind velocity every circling
% version 2 can control number of samples every circling

% I find circling clockwise make estimated wind direction more than 180 deg
% Coefficient "const" depends on which kind of gliders and wind velocity.

clc;
clear all;
close all;


% CONFIGURATION
% Input truth motion profile filename
%"no thermal" case is used for checking estimation of v_wind


% ASG29 ASK21 £¨right turn| Clockwise£©5kt 10kt 15 kt 180deg wind
%input_profile_name = 'data_202002181837wind(50fps d=2000ft)part2.csv';
%input_profile_name = 'data_202002181837wind(50fps d=2000ft).csv';

%input_profile_name = 'data_202002232355estimatewind5kt180degASG29.csv';
%input_profile_name = 'data_202002232336estimatewind5kt180degASK21.csv';

%input_profile_name = 'data_202002240045estimatewind10kt180degASG29.csv';
%input_profile_name = 'data_202002240027estimatewind10kt180degASK21.csv';
% ASK21 result is very good
% 
% input_profile_name = 'data_202002240039estimatewind15kt180degASG29.csv';
 input_profile_name = 'data_202002240033estimatewind15kt180degASK21.csv';


% Begins

% Input truth motion profile from .csv format file
[in_profile,n_epochs,ok] = importcsv(input_profile_name);
% in_profile: inputdata
% n_epochs: numbers of measurements

% End script if there is a problem with the file
if ~ok
    disp('Input file has the wrong number of columns');
    return;
end

time=in_profile(:,1);
time=time-in_profile(1,1)*ones(n_epochs,1);
%Accelerometer=in_profile(:,2:4);
%Gyroscope=in_profile(:,5:7);
%Airspeed= in_profile(:,8);
%[Latitude,Longitude,altitude]=in_profile(:,9:11);
%Magnetometer= in_profile(:,12:14);

Latitude=in_profile(:,9);
Longitude=in_profile(:,10);
%altitude=in_profile(:,11);

groundtrack= in_profile(:,15);
%verticalv= in_profile(:,16);
n_epochs=length(time);
%thermallift=in_profile(:,18);

%% wind parameters
const=0.8;%coefficient shows the relationship between glider ASG29 and wind(5kt)
const=0.83;%coefficient shows the relationship between glider ASK21 and wind(5kt)

const=0.8;%coefficient shows the relationship between glider ASG29 and wind(10kt)
const=0.82;%coefficient shows the relationship between glider ASK21 and wind(10kt)

const=0.9;%coefficient shows the relationship between glider ASG29 and wind(15kt)
const=0.88;%coefficient shows the relationship between glider ASK21 and wind(15kt)

truevelocity=2.57222*3;
truedirection=180; %unit deg
% 180 deg means wind from South to North
%deg2rad=0.01745329251994329576;

%%
%equatorial radius of earth, unit: m
a=6378137;
Px = (Latitude-Latitude(1)*ones(n_epochs,1))*a;
Py = (Longitude-Longitude(1)*ones(n_epochs,1))*a*cos(Latitude(1));
%Py = (Longitude-Longitude(1)*ones(n_epochs,1))*a.*cos(Latitude);


%% estimate v_wind
Position=zeros(n_epochs,2); 
% position save the positions that aircraft have turned around
num_sample=50; %number of sampling every circling

for i=1:n_epochs
    Position(i,:)=[Px(i) Py(i)];
end

%% to find samples
sequence_num=zeros(n_epochs,1);%save all near 0 or 2*pi points on the circles 
sequence_number=zeros(n_epochs,1);%save all sample-points on the circles 
for i=1:n_epochs-1
    if abs(groundtrack(i+1)-groundtrack(i))>=6.2      
        sequence_num(i)=i;
    end
end

for p=1:n_epochs-1
    if sequence_num(p)~=0
        q=p+1;
        while sequence_num(q)==0 && q<=n_epochs-1
            q=q+1;
        end
        if q~=n_epochs
            tempdiff=sequence_num(q)-sequence_num(p);
            for o=0:tempdiff/num_sample:tempdiff
                o_temp=round(o);
                sequence_number(p+o_temp)=p+o_temp;
            end
        end
    end
end

%% seq only save non-zero sequence_number
j=1;
for i=1:n_epochs
    if sequence_number(i)~=0
        seq(j)=i;
        j=j+1;
    end
end
length_seq=length(seq);


v_wind=zeros(n_epochs,2);
for j=1:length_seq-num_sample
    temp=(Position(seq(j+num_sample),:)-Position(seq(j),:))...
        /(time(seq(j+num_sample))-time(seq(j)));
    v_wind(seq(j):seq(j+1)-1,:)=[temp(1)*ones(seq(j+1)-seq(j),1) temp(2)*ones(seq(j+1)-seq(j),1)];
    last=seq(j+1);
end

v_wind(last:n_epochs,:)=v_wind(last-1,:).*ones(n_epochs-last+1,2);

absv_wind=sqrt(v_wind(:,1).^2+v_wind(:,2).^2);
% here v_wind is movement of the wind pro second.

%% estimate direction_v_wind, unit: deg
% 0 deg means wind from South to North
direction_v_wind=zeros(n_epochs,1);

for j=1:n_epochs
    if v_wind(j,2)>=0 %v_wind(j,2)>=0 means wind goes to East
        direction_v_wind(j)=acos(v_wind(j,1)/absv_wind(j))*180/pi;
    else
        direction_v_wind(j)=360-acos(v_wind(j,1)/absv_wind(j))*180/pi; 
    end
end

%% estimate direction_wind, unit: deg
% Wind direction is reported by the direction from which it originates.
% 180 deg means wind from South to North
% a wind blowing from the north has a wind direction of  0 deg(360deg)
% a wind blowing from the  east has a wind direction of 90 deg
direction_wind=zeros(n_epochs,1);

for j=1:n_epochs
    if v_wind(j,2)>=0 %v_wind(j,2)>=0 means wind goes to East
        direction_wind(j)=180+acos(v_wind(j,1)/absv_wind(j))*180/pi;
    else
        direction_wind(j)=180-acos(v_wind(j,1)/absv_wind(j))*180/pi; 
    end
end

%% figure shows results
absv_wind=const*absv_wind;


n=n_epochs-1;%n_max=n_epochs-1;
figure()
subplot(2,1,1)
hold on;
plotvw1=plot(time(1:n),absv_wind(1:n),'b');
plotvw2=plot(time(1:n),truevelocity*ones(n,1),'r');
legend([plotvw1,plotvw2], ...
    ["estimated velocity of wind","true velocity of wind"]);
title('Velocity and direction of wind');
ylabel('Velocity (m/s)');
xlabel('Time (s)');
ylim([0 10]);
hold off;

subplot(2,1,2)
hold on;
plotdw1=plot(time(1:n),direction_wind(1:n),'b');
plotdw2=plot(time(1:n),truedirection*ones(n,1),'r');
legend([plotdw1,plotdw2], ...
    ["estimated direction of wind","true direction of wind"]);
ylabel('Direction (deg)');
xlabel('Time (s)');
ylim([0 360]);
hold off;

% subplot(2,1,2)
% hold on;
% plotdw1=plot(time(1:n),direction_v_wind(1:n),'b');
% plotdw2=plot(time(1:n),truedirection*ones(n,1),'r');
% legend([plotdw1,plotdw2], ...
%     ["estimated direction of wind","true direction of wind"]);
% ylabel('Direction (deg)');
% xlabel('Time (s)');
% ylim([0 360])
% hold off;
