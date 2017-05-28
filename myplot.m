figure(1)
clc;
clear all;
todraw = importdata('D:\cs\FinalYearProject\data\2s_update.csv');
loss0 = importdata('D:\cs\FinalYearProject\data\no_loss.log');
%for i = 100:9600
    %plot(todraw(i,1),todraw(i,2),'.b')
    %hold on
    %plot(loss0(i,2),loss0(i,3),'.g')
    %hold on
%end
%plot(todraw(:,1),todraw(:,2),'*r',loss0(:,2),loss0(:,3),'*b','linewidth',1);
%hold on
plot(todraw(:,1),todraw(:,2),'-r',loss0(:,2),loss0(:,3),'-b','linewidth',2);
grid on
%LineWidth(4);