function animate_drone(out, speed)
% animate_drone(out)  — 단일 모터고장 실사형 애니메이션 (Coder/FlightGear 불필요)
%   왼쪽 : 고정 카메라 전체뷰 (비행 경로 전체)
%   오른쪽: 드론 따라가는 확대 추적뷰
%   ● 파랑 프로펠러 = 정상(회전), ● 빨강 = 고장(모터2, 정지)
%   speed : 재생 배속 (기본 1)

if nargin < 2, speed = 1; end
failed = 2;

%% --- 데이터 ---
t = out.s_g.Time(:);
P = sq3(out.s_g.Data);
try, E = sq3(out.Euler_angles.Data); catch, E = zeros(3,numel(t)); end
M = min([numel(t), size(P,2), size(E,2)]);
t=t(1:M); P=P(:,1:M); E=E(:,1:M);
Nn=P(1,:); Ee=P(2,:); Uu=-P(3,:);

%% --- 로터배치(X형) + 확대 ---
bx=0.0795; by=0.099; sc=5;
prop = sc*[ bx -by 0; bx by 0; -bx by 0; -bx -by 0]';
pr   = sc*0.05;
na=20; aa=linspace(0,2*pi,na); circ = pr*[cos(aa);sin(aa);zeros(1,na)];

%% --- figure 2분할 ---
f=figure('Name','드론 애니메이션 (빨강=고장 모터2)','Color',[.96 .97 .99],'Position',[60 90 1200 560]); clf(f);
ax1=subplot(1,2,1,'Parent',f); hold(ax1,'on'); grid(ax1,'on'); box(ax1,'on'); daspect(ax1,[1 1 1]);
xlabel(ax1,'East'); ylabel(ax1,'North'); zlabel(ax1,'고도'); view(ax1,[-37 22]); title(ax1,'전체뷰 (카메라 고정)');
ax2=subplot(1,2,2,'Parent',f); hold(ax2,'on'); grid(ax2,'on'); box(ax2,'on'); daspect(ax2,[1 1 1]);
xlabel(ax2,'East'); ylabel(ax2,'North'); zlabel(ax2,'고도'); view(ax2,[-37 22]); title(ax2,'확대 추적뷰');

% 전체뷰 고정 범위 + 전체 경로 흐리게
plot3(ax1,Ee,Nn,Uu,'Color',[.8 .8 .88]);
pad=sc*0.12;
xlim(ax1,[min(Ee)-pad max(Ee)+pad]); ylim(ax1,[min(Nn)-pad max(Nn)+pad]); zlim(ax1,[min(Uu)-pad max(Uu)+pad]);

H1=mkH(ax1); H2=mkH(ax2);
ttl=sgtitle('');

W=sc*0.22; spin=0; step=max(1,round(speed*M/300));
for k=1:step:M
    R=eulR(E(1,k),E(2,k),E(3,k));
    c=[Nn(k);Ee(k);P(3,k)];
    pW=R*prop+c;
    % 팔
    AX=[];AY=[];AZ=[];
    for i=1:4, s=n2p([c pW(:,i)]); AX=[AX s(1,:) nan]; AY=[AY s(2,:) nan]; AZ=[AZ s(3,:) nan]; end
    hc=n2p(c);
    % 프로펠러 원반 + 블레이드
    D=cell(1,4); B=cell(1,4); DC=cell(1,4); BC=cell(1,4);
    for i=1:4
        D{i}=n2p(R*(prop(:,i)+circ)+c);
        ang=(i~=failed)*spin;
        bl=[cos(ang) -sin(ang);sin(ang) cos(ang)]*[-pr pr;0 0];
        B{i}=n2p(R*(prop(:,i)+[bl;0 0])+c);
        if i==failed, DC{i}=[.9 .15 .15]; BC{i}=[.55 .55 .55];
        else,         DC{i}=[.15 .45 .95]; BC{i}=[.05 .05 .05]; end
    end
    tx=Ee(1:k); ty=Nn(1:k); tz=Uu(1:k);
    setH(H1,AX,AY,AZ,hc,D,DC,B,BC,tx,ty,tz);
    setH(H2,AX,AY,AZ,hc,D,DC,B,BC,tx,ty,tz);
    % 추적뷰 범위
    xlim(ax2,[Ee(k)-W Ee(k)+W]); ylim(ax2,[Nn(k)-W Nn(k)+W]); zlim(ax2,[Uu(k)-W Uu(k)+W]);
    set(ttl,'String',sprintf('t = %.2f s    고도 = %.2f m    (빨강 = 고장 모터2)', t(k), Uu(k)));
    spin=spin+0.8; drawnow;
end
end

%% ---- 보조 ----
function H=mkH(ax)
    H.tr =plot3(ax,nan,nan,nan,'-','Color',[.4 .6 1],'LineWidth',1.2);
    H.arm=plot3(ax,nan,nan,nan,'-','Color',[.15 .15 .15],'LineWidth',3);
    H.hub=plot3(ax,nan,nan,nan,'s','MarkerSize',9,'MarkerFaceColor',[.3 .3 .35],'MarkerEdgeColor','k');
    for i=1:4
        H.d(i) =patch('Parent',ax,'XData',nan,'YData',nan,'ZData',nan,'EdgeColor',[.2 .2 .2],'FaceAlpha',.30);
        H.bl(i)=plot3(ax,nan,nan,nan,'-','LineWidth',2.5);
    end
end
function setH(H,AX,AY,AZ,hc,D,DC,B,BC,tx,ty,tz)
    set(H.arm,'XData',AX,'YData',AY,'ZData',AZ);
    set(H.hub,'XData',hc(1),'YData',hc(2),'ZData',hc(3));
    for i=1:4
        set(H.d(i),'XData',D{i}(1,:),'YData',D{i}(2,:),'ZData',D{i}(3,:),'FaceColor',DC{i});
        set(H.bl(i),'XData',B{i}(1,:),'YData',B{i}(2,:),'ZData',B{i}(3,:),'Color',BC{i});
    end
    set(H.tr,'XData',tx,'YData',ty,'ZData',tz);
end
function Q=n2p(V), Q=[V(2,:); V(1,:); -V(3,:)]; end
function R=eulR(phi,theta,psi)
    cx=cos(phi);sx=sin(phi);cy=cos(theta);sy=sin(theta);cz=cos(psi);sz=sin(psi);
    R=[cz -sz 0;sz cz 0;0 0 1]*[cy 0 sy;0 1 0;-sy 0 cy]*[1 0 0;0 cx -sx;0 sx cx];
end
function X=sq3(D)
    X=squeeze(D); if size(X,1)~=3 && size(X,2)==3, X=X.'; end
    if size(X,1)~=3, X=reshape(D,3,[]); end
end
