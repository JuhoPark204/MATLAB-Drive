%% sweep_adapt_gain_IRIS.m — 적응 게인 탐색 (500g, 질량보정 + 게인 스윕)
% 배경: compare_BC_IRIS 결과 — 질량/관성을 제어기에 반영해도 500g 추락을 못 막음.
%        (INDI는 incremental이라 질량오차에 이미 강건. 진짜 킬러는 15m 고도강하)
% 질문: 제어 게인(민첩도 AgilityAtti/Pos)을 화물에 맞게 올리면
%        고장 후 강하를 빨리 잡아서 생존시킬 수 있는가?  (계획서 H2)
% plant = 진짜 500g. 제어기 = 질량+관성 보정 + 민첩도 격자 스윕.
%        (무게추정 정확도는 이미 별도 검증 → 여기선 게인 효과만 분리해서 봄. 진짜질량 사용)
%
% 사용법:  sweep_adapt_gain_IRIS              % 기본 500g
%          TRUE_PAYLOAD=1.0; sweep_adapt_gain_IRIS

if ~exist('TRUE_PAYLOAD','var') || isempty(TRUE_PAYLOAD), TRUE_PAYLOAD = 0.5; end
clc; addPathFtc();
poffset = 0.10;

%% 공통 설정
failure_time_mot_1=1000; failure_time_mot_2=1; failure_time_mot_3=1000; failure_time_mot_4=1000;
simin = automatedStickCommands();
copter = copterLoadParams('copter_params_IRIS');
envir = envirLoadParams('params_envir','envir',0);
IC.omega_Kb=[0;0;0]; IC.q_bg=euler2Quat([0;0;0]); IC.V_Kb=[0;0;0]; IC.s_Kg=[0;0;0];
IC.omega_mot=[1;1;1;1]*500;
grnd=groundLoadParams('params_ground_default');
pos_ref.lat=37.6117; pos_ref.lon=-122.37822; pos_ref.alt=15;
fg.remoteURL='127.0.0.1'; fg.remotePort=5502;

copter_dry   = copter;  m_dry = copter_dry.body.m;
copter_ctrl  = mkplant(copter_dry, TRUE_PAYLOAD, poffset, false);  % 제어기 설계용(질량+관성)
copter_plant = mkplant(copter_dry, TRUE_PAYLOAD, poffset, true);   % plant(진짜 화물)

m='QuadcopterSimModel_Loiter_FTC'; load_system(m);
set_param([m '/To FlightGear'],'Commented','on');
set_param([m '/Environment/From FlightGear'],'Commented','on');
set_param(m,'UnconnectedInputMsg','none');

%% 0) 참고: baseline B (마른 제어기, 0.5/0.5)
fm_loiter = mkctrl(copter_dry, 0.5, 0.5);  copter = copter_plant;
out = sim(m,'StopTime','18'); [dB,~,zB,okB]=pmetrics(out);
fprintf('\n[기준] B(마른,0.5/0.5): 강하 %.2f m, 최종 %.2f m, %s\n', dB, zB, sv(okB));

%% 1) 게인 격자 스윕 (질량보정 제어기)
aAttiList = [0.5 0.7 0.9 1.2];
aPosList  = [0.5 0.7 0.9 1.2];

fprintf('\n   화물 %.0f g | 질량보정 제어기 + 게인 스윕\n', TRUE_PAYLOAD*1000);
fprintf(' AgiAtti \\ AgiPos ');  fprintf('|  %4.1f  ', aPosList); fprintf('\n');
fprintf('------------------'); fprintf('+--------'); fprintf(repmat('--------',1,numel(aPosList)-1)); fprintf('\n');

best=struct('ok',false,'drop',inf,'aA',nan,'aP',nan);
DROP=nan(numel(aAttiList),numel(aPosList));
for i=1:numel(aAttiList)
    fprintf('   %4.1f           ', aAttiList(i));
    for j=1:numel(aPosList)
        fm_loiter = mkctrl(copter_ctrl, aAttiList(i), aPosList(j));
        copter = copter_plant;
        try
            out = sim(m,'StopTime','18');
            [drop,~,zf,ok] = pmetrics(out);   %#ok<ASGLU>
            DROP(i,j)=drop;
            mark='✗'; if ok, mark='✓'; end
            fprintf('| %5.1f%s ', drop, mark);
            if ok && drop<best.drop, best=struct('ok',true,'drop',drop,'aA',aAttiList(i),'aP',aPosList(j)); end
        catch
            fprintf('|  err  ');
        end
    end
    fprintf('\n');
end
fprintf('  (숫자=최대강하[m], ✓=생존 ✗=추락)\n\n');

if best.ok
    fprintf('>> ★ 생존 발견! AgilityAtti=%.1f, AgilityPos=%.1f (강하 %.2f m)\n', best.aA, best.aP, best.drop);
    fprintf('   = 게인 적응이 답(H2). 이 게인을 적응형 FTC(C2)로 채택.\n\n');
else
    fprintf('>> 생존 게인 없음. 500g는 게인만으론 부족 → 추력여유/시작고도/전략 재검토 필요.\n\n');
end

%% 히트맵
figure(51); clf; imagesc(aPosList, aAttiList, DROP); axis xy; colorbar;
xlabel('AgilityPos'); ylabel('AgilityAtti');
title(sprintf('화물 %.0f g: 게인별 고장후 최대강하[m]', TRUE_PAYLOAD*1000));
set(gca,'XTick',aPosList,'YTick',aAttiList);

%% ===== 보조 함수 =====
function fm = mkctrl(cdes, aAtti, aPos)
    copter = cdes;   % autoCreate가 evalin('caller','copter')로 읽음
    fm = lindiCopterAutoCreate(copter,'AgilityAtti',aAtti,'AgilityPos',aPos, ...
                               'FilterStrength',0,'CntrlEffectScaling',1);
    fm.psc.inditype = 1;
end

function c = mkplant(cdry, pm, poff, withCoG)
    c = cdry;
    c.body.m = c.body.m + pm;
    c.body.I = c.body.I + pm*poff^2*diag([1 1 0]);
    if withCoG, c.config.CoG_Pos_c = c.config.CoG_Pos_c + [0;0;poff]; end
end

function [drop,trec,zf,ok] = pmetrics(out)
    ts=out.s_g.Time(:); up=-squeeze(out.s_g.Data(3,1,:));
    post=ts>=1; upp=up(post); tp=ts(post);
    [mn,idx]=min(upp); drop=-mn;
    after=idx:numel(upp); rec=find(abs(upp(after))<0.5,1);
    if isempty(rec), trec=inf; else, trec=tp(after(rec))-1; end
    zf = up(end);
    ss=ts>=(ts(end)-3); ok=isfinite(trec) && max(abs(up(ss)))<2;
end

function s=sv(ok), if ok, s='생존 ✓'; else, s='추락 ✗'; end, end
