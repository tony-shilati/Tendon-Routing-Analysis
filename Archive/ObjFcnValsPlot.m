                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   ;
l3 = linspace(0.3, 0.6, 100);
theta3_i = linspace(10, 60, 100) * pi/180;

ObjFuncVal = zeros(length(theta3_i), length(l3));


for i=1:(length(l3))
    for j=1:length(theta3_i)
        ObjFuncVal(j, i) = N((i-1)*length(theta3_i) + j);
    end
end

[xx, yy] = meshgrid(l3, theta3_i);

% xx_cut = xx(1:25,:);
% yy_cut = yy(1:25,:);
% ObjFuncVal_cut = ObjFuncVal(1:25,:);
% surf(xx_cut, yy_cut, ObjFuncVal_cut)
figure
surf(xx, yy, ObjFuncVal, "EdgeColor","none")
colorbar

xlabel("X")
ylabel("Y")



% for i=1:(length(XCenter))
%     for j=1:length(YCenter)
% plot3(XCenter(i),YCenter(j), Fvals((i-1)*length(YCenter) + j), 'o')
% hold on
%     end
% end

%axis([-2, 8, -2, 8, -1, 100])
% title(['Stiffness matrix: K=[3, -2, 1; -2, 6, 5; 1, 5, 9]; Fingertips: P1 = [0, 0]; ' ...
%     'P2 = [2, 1];'])
