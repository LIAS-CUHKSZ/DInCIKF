
function [landmarks] = camera_genlandmark(n_landmarks,center,wlh, s, biaxyz)


landmarks=zeros(n_landmarks,3);
n=n_landmarks/4;
%lwh is the length, width and height of each wall
%where is the center of each wall 
%s is used to adjust the scaler


center1=[center(1)+biaxyz(1),center(2),center(3)];
center2=[center(1)-biaxyz(1),center(2),center(3)];
center3=[center(1),center(2)+biaxyz(2),center(3)];
center4=[center(1),center(2)-biaxyz(2),center(3)];

landmarks(1:n,1)=( center1(1)-wlh(1) +rand(n,1)* 2*wlh(1) )/s;
landmarks(1:n,2)=( center1(2)-wlh(2) +rand(n,1)* 2*wlh(2) )/s;
landmarks(1:n,3)=( center1(3)-wlh(3) +rand(n,1)* 2*wlh(3) )/s;

landmarks(n+1:n*2,1)=( center2(1)-wlh(1) +rand(n,1)* 2*wlh(1) )/s;
landmarks(n+1:n*2,2)=( center2(2)-wlh(2) +rand(n,1)* 2*wlh(2) )/s;
landmarks(n+1:n*2,3)=( center2(3)-wlh(3) +rand(n,1)* 2*wlh(3) )/s;

landmarks(2*n+1:n*3,1)=( center3(1)-wlh(2) +rand(n,1)* 2*wlh(2) )/s;
landmarks(2*n+1:n*3,2)=( center3(2)-wlh(1) +rand(n,1)* 2*wlh(1) )/s;
landmarks(2*n+1:n*3,3)=( center3(3)-wlh(3) +rand(n,1)* 2*wlh(3) )/s;
% 
landmarks(3*n+1:n*4,1)=( center4(1)-wlh(2) +rand(n,1)* 2*wlh(2) )/s;
landmarks(3*n+1:n*4,2)=( center4(2)-wlh(1) +rand(n,1)* 2*wlh(1) )/s;
landmarks(3*n+1:n*4,3)=( center4(3)-wlh(3) +rand(n,1)* 2*wlh(3) )/s;


xlabel('x');
ylabel('y');
zlabel('z');




axis equal;
% draw landmarks
scatter3( landmarks(:, 1), landmarks(:, 2), landmarks(:, 3), 20, 'filled','pentagram'); hold on;

hold on;

end