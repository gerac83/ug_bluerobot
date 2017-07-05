function showCamera(R,T,textdesc,coordCenter, textleft)

nx = 800;
ny = 600;
fc(1) = 600; fc(2) = 600;
cc(1) = nx/2; cc(2) = ny/2;

IP = 100*[1 0 0;0 1 0;0 0 1]*[1/fc(1) 0 0;0 1/fc(2) 0;0 0 1]*[1 0 -cc(1);0 1 -cc(2);0 0 1]*[0 nx-1 nx-1 0 0 ; 0 0 ny-1 ny-1 0;1 1 1 1 1];
BASE = 100*([0 1 0 0 0 0;0 0 0 1 0 0;0 0 0 0 0 1]);
IP = reshape([IP;BASE(:,1)*ones(1,5);IP],3,15);

BASE = 100*([0 1 0 0 0 0;0 0 0 1 0 0;0 0 0 0 0 1]);
BASE = R'*(BASE - repmat(T,[1 6]));
IP= R'*(IP - repmat(T,[1 15]));

plot3(IP(1,:),IP(3,:),-IP(2,:),'r-','linewidth',2);
hold on;

if coordCenter
plot3(BASE(1,:),BASE(3,:),-BASE(2,:),'b-','linewidth',2');
text(BASE(1,2),BASE(3,2),-BASE(2,2),'X','HorizontalAlignment','center','FontWeight','bold');
text(BASE(1,6),BASE(3,6),-BASE(2,6),'Z','HorizontalAlignment','center','FontWeight','bold');
text(BASE(1,4),BASE(3,4),-BASE(2,4),'Y','HorizontalAlignment','center','FontWeight','bold');
end

text(BASE(1,1)+20,BASE(3,1)+150,BASE(2,1)+180,textdesc,'HorizontalAlignment','center','FontWeight','bold');

end