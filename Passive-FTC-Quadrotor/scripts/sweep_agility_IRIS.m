%% sweep_agility_IRIS.m — IRIS 무화물 baseline agility 튜닝
% 단일 모터고장(모터2, t=1s), 화물 없음 조건에서
% AgilityAtti × AgilityPos 격자를 돌려가며
% [최대강하량 / 회복시간 / 최대기울기]를 비교해 깔끔한 baseline을 고른다.
%
% 사용법:  sweep_agility_IRIS

clear; clc;
addPathFtc();

%% 고정 설정 (init_IRIS_Loiter_FTC 와 동일)
failure_time_mot_1=1000; failure_time_mot_2=1; failure_time_mot_3=1000; failure_time_mot_4=1000;
simin  = automatedStickCommands();
copter = copterLoadParams('copter_params_IRIS');
envir  = envirLoadParams('params_envir','envir',0);
IC.omega_Kb=[0;0;0]; IC.q_bg=euler2Quat([0;0;0]); IC.V_Kb=[0;0;0]; IC.s_Kg=[0;0;0];
IC.omega_mot=[1;1;1;1]*500;
grnd = groundLoadParams('params_ground_default');
pos_ref.lat=37.6117; pos_ref.lon=-122.37822; pos_ref.alt=15;
fg.remoteURL='127.0.0.1'; fg.remotePort=5502;

m='QuadcopterSimModel_Loiter_FTC';
load_system(m);
set_param([m '/To FlightGear'],'Commented','on');
set_param([m '/Environment/From FlightGear'],'Commented','on');
set_param(m,'UnconnectedInputMsg','none');   % FlightGear 주석처리로 생기는 미연결 포트 경고 끄기

%% 스윕 격자  (값 클수록 민첩)
attiList = [0.5 0.7 0.9];
posList  = [0.2 0.35 0.5];

fprintf('\n AgilAtti AgilPos | 최대강하[m] | 회복시간[s] | 최대기울기[deg] | 판정\n');
fprintf('--------------------------------------------------------------------------\n');
best=[]; bestScore=inf;
for aa=attiList
  for ap=posList
    fm_loiter = lindiCopterAutoCreate(copter,'AgilityAtti',aa,'AgilityPos',ap, ...
                                      'FilterStrength',0,'CntrlEffectScaling',1);
    fm_loiter.psc.inditype = 1;
    try
      out = sim(m,'StopTime','18');
      [drop,trec,sstilt,stable] = metrics(out);
      v = '추락/불안정'; if stable, v='OK(안정)'; end
      fprintf('  %.2f    %.2f   | %9.2f | %9.2f | %12.1f | %s\n', aa,ap,drop,trec,sstilt,v);
      score = drop + 0.3*trec;
      if stable && score<bestScore, bestScore=score; best=[aa ap drop trec mtilt]; end
    catch ME
      fprintf('  %.2f    %.2f   |  에러: %s\n', aa,ap,ME.message);
    end
  end
end
fprintf('--------------------------------------------------------------------------\n');
if ~isempty(best)
  fprintf('>> 추천 baseline: AgilityAtti=%.2f, AgilityPos=%.2f  (강하 %.2fm, 회복 %.1fs, 기울기 %.1fdeg)\n\n', ...
          best(1),best(2),best(3),best(4),best(5));
else
  fprintf('>> 안정적인 조합 없음 → 격자 범위 조정 필요.\n\n');
end

%% ---- 지표 ----
function [drop,trec,sstilt,ok]=metrics(out)
  ts=out.s_g.Time(:); S=sq3(out.s_g.Data); up=-S(3,:);   % 고도(위+)
  te=out.Euler_angles.Time(:); E=sq3(out.Euler_angles.Data);
  post = ts>=1;                                           % 고장(t=1) 이후
  upp = up(post); tp = ts(post);
  [mn,idx]=min(upp); drop=-mn;                            % 최대강하(양수)
  after=idx:numel(upp);
  rec=find(abs(upp(after))<0.3,1);                        % 최저점 이후 ±0.3m 복귀
  if isempty(rec), trec=inf; else, trec=tp(after(rec))-1; end
  % 정상상태(마지막 3초) 평가
  ss  = ts>=(ts(end)-3);  sse = te>=(te(end)-3);
  ok  = isfinite(trec) && max(abs(up(ss)))<1.5;          % 회복 + 정상상태 고도 안정
  sstilt = mean(sqrt(E(1,sse).^2+E(2,sse).^2))*180/pi;   % 정상상태 평균 기울기(정보용)
end
function X=sq3(D)
  X=squeeze(D); if size(X,1)~=3 && size(X,2)==3, X=X.'; end
  if size(X,1)~=3, X=reshape(D,3,[]); end
end
