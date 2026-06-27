%% demo4_adapt_IRIS — 실험⑤ 적응 비교 핵심 결과 (비용 J 막대그래프)
% optimize_gain_IRIS 격자탐색 결과를 시각화 (즉시, 시뮬 없음).
% 헤드라인: (가) 질량·관성 반영 [현실 기준선] vs (가)+(나) 결합 [제안].
% 사용법:  demo4_adapt_IRIS
clear; clc;

Pg   = [200 300 340];      % 화물[g]
Jga  = [29.8 32.7 28.2];   % (가) 질량·관성 반영
Jboth= [14.7 19.7 13.1];   % (가)+(나) 결합

figure('Name','실험5: 적응 비교 (핵심)','Color','w','Position',[100 120 820 500]); clf;
b=bar([Jga(:) Jboth(:)],'grouped'); grid on;
b(1).FaceColor=[.85 .55 .2]; b(2).FaceColor=[0 .45 .74];
set(gca,'XTickLabel',compose('%dg',Pg),'FontSize',13);
ylabel('종합 비용 J (낮을수록 우수)','FontSize',13);
legend({'(가) 질량·관성 반영 [기준선]','(가)+(나) 결합 [제안]'},'Location','northeast','FontSize',12);
title('실험 ⑤ 적응 비교 — 질량 반영 위에 게인 적응 추가 시 ~40~54% 추가 개선','FontSize',14);
for i=1:numel(Pg)
    text(i-0.15,Jga(i)+0.8,  sprintf('%.1f',Jga(i)),  'HorizontalAlignment','center','FontSize',11);
    text(i+0.15,Jboth(i)+0.8,sprintf('%.1f',Jboth(i)),'HorizontalAlignment','center','FontSize',11,'Color',[0 .3 .6]);
end
fprintf('결합 적응(가+나)이 전 화물에서 최우수. (수치 = optimize_gain_IRIS 결과)\n');

%% (부록) 340g 2x2 요인분석 — 개선 주인자 = 게인 적응
figure('Name','실험5 부록: 2x2 요인','Color','w','Position',[140 90 640 420]); clf;
M=[46.9 21.1; 28.2 13.1];   % 행: 모델(미반영/반영), 열: 게인(고정/적응)
b2=bar(M); grid on;
b2(1).FaceColor=[.7 .5 .3]; b2(2).FaceColor=[0 .45 .74];
set(gca,'XTickLabel',{'모델 미반영','모델 반영(가)'},'FontSize',12);
ylabel('비용 J (340g)','FontSize',12);
legend({'게인 고정','게인 적응'},'Location','northeast','FontSize',11);
title('부록: 게인 적응이 개선 주인자 (게인만으로도 21.1 < 모델만 28.2)','FontSize',12);
