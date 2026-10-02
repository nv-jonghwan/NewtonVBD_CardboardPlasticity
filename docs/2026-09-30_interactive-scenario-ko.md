# UR10 인터랙티브 상자 시나리오

실행: `./scripts/python.sh scripts/live_isaac.py`.

오른쪽 **Cardboard Scenario**에서 Play / Resume, Pause, Reset to Home으로 제어한다. 창을 열면 초기 자세에서 대기하며, 완료 후 Play는 새 Newton 모델로 재시작한다. Reset은 초기 형상과 소성 이력까지 복원한다. 원래 11초 고정 위치 실험과 구분되는 16초 접촉 시나리오이다.

| 시뮬레이션 시간 | 동작 |
|---|---|
| 0–1 s | 상자에서 떨어진 기본 자세 |
| 1–3 s | 상자 위로 이동 |
| 3–5 s | 열린 손가락을 상자 양옆으로 하강 |
| 5–7 s | 18 N 모터 제한으로 집기 |
| 7–9 s | 12 cm 상승 목표로 들어 올리기 |
| 9–12 s | 70 N까지 부드럽게 증가, 개구 목표 110 mm로 압착하고 마지막 1.05초 유지 |
| 12–13 s | 손가락 벌리기 |
| 13–16 s | 손을 위로 빼기, 중력에 의한 낙하와 테이블 접촉 |

힘은 각 prismatic finger actuator의 구동 제한이다. 실제 접촉력이나 UR 관절 토크의 hard bound를 뜻하지 않는다. 실제 손가락 간격은 상자 반력과 제어 오차에 따라 목표보다 크다. 상자 부착 constraint, 상자 위치 스크립트, 변형 blend shape를 사용하지 않는다.

`CardboardScenarioAPI`를 추가하여 전체 등록 API는 6개이다. `/World/Physics`에 homeOffset, approachClearance, liftHeight, graspGap, crushGap, graspForce, crushForce, crushRampFraction, phaseEnds를 저장한다. Newton driver가 이를 읽는다. `exts/cardboard.scenario`는 Kit IExt 기반 창이며, bridge가 별도 Newton worker와 atomic command/state 파일로 연결한다. GUI 및 PhysX와 Newton이 동시에 같은 물체를 적분하지 않도록 표시 stage의 강체/충돌/관절/Articulation API를 session layer에서 비활성화한다.

초기 35 N 시나리오 검증 (`outputs/scenario_validation_final/report.json`; 비교 기준으로 보존):

- 12개 시나리오 검사 통과. 접근 중 소성 이력 0.
- 들어 올린 상자 최저점과 테이블 사이 실제 간격 약 90.97 mm.
- 압착 구간 손가락 간격 약 12.94 mm 추가 감소; 소성 힌지 10 → 21개.
- 낙하 후 소성 힌지 608개. 최종 변형의 상당 부분은 낙하/접촉 중 발생하므로 압착만의 변형량으로 해석하지 않는다.
- 최종 양손 접촉력 0 N, 상자 최저점 z=0.65114 m (테이블 z=0.650 m).
- 마지막 최대 입자 속도 0.0736 m/s: 테이블 접촉은 확인했으나 완전 정적 평형까지 대기한 결과는 아니다.

기존 재료 테스트 3개와 USD 검사 10개도 통과한다. 원래 정량 수렴/실물 동정 한계는 그대로 적용한다. 시뮬레이션 16초를 계산하는 실제 시간은 렌더러 및 다른 GPU 작업에 따라 더 길며 실시간 성능을 주장하지 않는다.

재현 명령:

```bash
./scripts/python.sh scripts/stream_newton.py --scenario --stream outputs/scenario_validation_final/state.npz
./scripts/python.sh scripts/validate_scenario.py
./scripts/python.sh scripts/live_isaac.py --duration 0.3 --verify-controls --exit-when-done
```

Kit가 표시하는 Internal Session 데이터 수집 안내는 사용자 선택 사항이다. 이 데모는 해당 동의 설정을 변경하지 않는다.

## 압착 강화 후 최신 검증

재료는 변경하지 않고 압착 모터 제한 70 N, 목표 개구 110 mm, crushRampFraction 0.65로 조정했다. 3초 압착 구간 중 1.95초 동안 부드럽게 닫고 1.05초 유지한 후 놓는다.

- `outputs/scenario_stronger/report.json`: 시나리오 및 이전 버전 비교 14개 검사 통과.
- 실제 압착 종료 간격: 이전 254.37 mm → 192.27 mm (추가 62.10 mm 폐쇄). 목표 110 mm는 반력 때문에 도달하지 않았다.
- 들어 올린 뒤부터 압착 종료까지 실제 간격 감소: 75.05 mm.
- 낙하 전 소성 힌지: 이전 21개 → 415개; 낙하 후 828개.
- 최종 양손 접촉력 0 N, 상자 최저점 z=0.65126 m. 마지막 최대 속도 0.0703 m/s이므로 완전 정적 평형은 아니다.
- `phase-04.npz`, `phase-05.npz`, `phase-06.npz`, `phase-07.npz`에 실제 단계별 형상을 저장했다. `shape_comparison.png`는 중심을 맞추고 동일 스케일로 그린 셸 형상이다.

최신 재현:

```bash
./scripts/python.sh scripts/stream_newton.py --scenario --stream outputs/scenario_stronger/state.npz
./scripts/python.sh scripts/validate_scenario.py
```

## 골판지 강성 5% 증가 (현재 적용값)

요청은 같은 힘에 조금 더 버티는 강성이므로, 항복을 쉽게 만드는 변경은 적용하지 않았다. yieldCurvature=12를 유지하고 membraneShear 12000→12600, membraneArea 20000→21000, bendingMD 0.04→0.042, bendingCD 0.0192→0.02016으로 변경했다. 그리퍼 힘 70 N과 목표 간격, 유지시간, 경화/손상 파라미터는 그대로이다.

`outputs/scenario_stiffer/report.json`: 12개 시나리오 검사 통과. 같은 힘에서 압착 종료 실제 간격이 192.27→193.67 mm로 1.40 mm 더 넓어, 약간 더 버티는 반응을 확인했다. 압착 직후 소성 힌지는 415→412개, 낙하 후 770개이며, 최종 손가락 접촉력은 양쪽 모두 0 N이다. 최종 입자 최대 속도는 0.0181 m/s이다. 강성 변화율과 전체 변형 변화율은 좌굴/접촉 때문에 선형 비례하지 않는다.

```bash
./scripts/python.sh scripts/stream_newton.py --scenario --stream outputs/scenario_stiffer/state.npz
./scripts/python.sh scripts/validate_scenario.py --output outputs/scenario_stiffer --baseline outputs/scenario_stronger --comparison-mode report
```
