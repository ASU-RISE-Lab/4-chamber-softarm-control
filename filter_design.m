clear all
close all
clc
fprintf( 'Loading... \n' );
load('raw_filter_design2.mat');
fprintf( 'Data loaded \n' );

figure(1)
for i =2:5
plot(data3(:,i),'--')
hold on
plot(data3(:,i+4))
hold on
end

fprintf( 'Loading... \n' );
load('raw_filter_design3.mat');
fprintf( 'Data loaded \n' );

figure(2)
for i =2:5
plot(data1(:,i),'--')
hold on
plot(data1(:,i+4))
hold on
end
%%
v_mat = zeros(length(data1),4);
a_mat = v_mat;
v_mat(2:end,:) = (data1(2:end,6:9)- data1(1:end-1,6:9))./(data1(2:end,1)- data1(1:end-1,1));

windowSize = 2; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
for i =1:4
f_vmat(:,i)=filter(b,a,v_mat(:,i));
hold on
end
a_mat(2:end,:) = (f_vmat(2:end,:)- f_vmat(1:end-1,:))./(data1(2:end,1)- data1(1:end-1,1));
%%
figure(3)
for i =1:4
    subplot(4,1,i)
plot(v_mat(:,i),'--')
hold on
plot(filter(b,a,v_mat(:,i)))
hold on
end

figure(4)
for i =1:4
    subplot(4,1,i)
plot(a_mat(:,i),'--')
hold on
plot(filter(b,a,a_mat(:,i)))
hold on
end