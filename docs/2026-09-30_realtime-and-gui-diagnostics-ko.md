# 실시간 실행 모드와 GUI 종료 진단

## 종료 기록에서 확인한 것

11:24:40에 시작한 GUI(pid580863)는11:29:58, 실제317.7초에 `SimulationApp.close`로 종료됐다. 마지막 Newton 상태는 시뮬레이션11.15초의 압착 단계였고 값은 유한했다. 같은 시간대의 시스템 기록에서 OOM, NVIDIA Xid, segmentation fault를 찾지 못했고 Newton 예외도 없었다. 시작 로그의 `previous crash`는 이전 세션의 덤프 처리이며 이번 종료의 원인으로 단정할 수 없다.

기존 코드에는 진단 문제가 있었다. Python 예외가 생기면 `finally`에서 Kit의 fast shutdown을 호출해, 원래 예외가 출력되기 전에 프로세스가 끝날 수 있었다. 이제 종료 전에 traceback, 마지막 시각/상태, worker 종료 코드와 로그 경로를 `outputs/live/gui-error-PID.json` 및 표준 오류에 먼저 기록하고 flush한다. SIGINT/SIGTERM/SIGHUP와 전용 패널의 버튼 동작도 기록한다. worker 로그는 GUI PID와 시작 시각별로 분리해 Reset 후에도 보존한다. 강제 예외 주입 시험에서 JSON과 traceback을 남긴 뒤 종료 코드1을 반환하는 것을 확인했다.

원래 정밀 설정으로 전체16초의 headless live bridge를 다시 실행해961개 snapshot과 정상 종료를 확인했다. 새 실시간 모드에서는 화면을 켠 전체 물리 실행과 기록 재생, Play/Pause/Reset을 확인했다. **보고된 종료의 정확한 트리거는 아직 확인하지 못했다.** 정상 재현 시험과 진단 보완을 영구적인 충돌 해결 보장으로 해석하지 않는다.

## 실시간화를 위한 변경

원래 정밀 모드는3458개 물리 정점,96회 반복,16 substeps를 사용했다. 프로파일에서 프레임당409.4ms 중405.2ms가 GPU graph 실행/대기였고, 별도의 표시 곡면 생성은45.4ms였다. 그래프에는 kernel88240개, memset10992개, memcpy1600개가 있었다. 이미 CUDA graph를 사용하고 있어서 단순 Python 호출 줄이기만으로 해결할 상황은 아니었다. [Warp 실행 문서](https://github.com/NVIDIA/warp/blob/main/docs/user_guide/runtime.rst), [Newton VBD 구현](https://github.com/newton-physics/newton/blob/v1.6.0/newton/_src/solvers/vbd/solver_vbd.py)

실시간 모드는 다음과 같이 별도로 구성했다.

- 물리 셸:8×8 분할의386 정점,768 삼각형.8회 VBD 반복,프레임당8 substeps,60Hz 제어이므로 접촉 적분은480Hz다.
- 표시:원본 외관/UV와57280 삼각형을 유지한다.171840개의 표시 정점 중 동일한 binding28642개만 계산하고 원래 배열로 펼친다. n8 표시 계산은 약8.8ms다. 이 중복 제거 자체는 형상을 바꾸지 않는다.
- UR 팔과 palm은 처방된 관절 자세를 따라간다. 팔 자세는 substep 사이에 보간한다. **양쪽 손가락은 계속 동적 강체이며150N 구동력 상한과 Newton 접촉 반력으로 움직인다. 상자는 입자 셸이고 부착 제약을 사용하지 않는다.**
- 정해진16초 팔 경로의 IK를 시작 전에 미리 계산해 실행 중 반복적인 최적화를 줄인다. 상자의 물리 상태는 미리 계산하지 않는다.
- 같은5mm,0.9kg/m² 골판지 물성,방향별 굽힘 강성0.556/0.278Nm,6ms 내부 굽힘 감쇠,소성 상태/손상 법칙을 유지한다.
- 접촉 점 밀도가9배 줄어드는 것에 맞춰 입자별 contact stiffness/damping을20000/0.1에서180000/0.9로 조정한다. 이는 접촉 이산화 변경이며 실물 접촉 보정은 아니다.
- 해제 시 개구를350mm로 넓히고0.25초에 목표 개구에 도달하도록 명령해, 휘어진 상자가 손가락에 걸려 올라가는 현상을 줄인다. 관절의 허용 개구352mm 안이다.
- 계산에 여유가 있는 경우 벽시계 속도에 맞춰 기다리는 pacing을 적용한다. 처리 속도가 부족하면 진행 속도가 떨어지며 UI의 Live배율에 실제 비율을 표시한다.

추가한 `CardboardDemoAPI` 필드는 `kinematicArm`, `precomputeCommands`, `realtimePacing`, `rigidCompliantALM`이다. `CardboardScenarioAPI`에는 `releaseRampFraction`을 추가했다. 모든 기본값은 이전 자산 동작을 유지하며, 실시간 자산이 필요한 값을 명시한다. ALM 시험은 접촉 품질을 통과하지 못했으므로 실시간 모드도 `rigidCompliantALM=false`를 쓴다.

## 측정 결과와 한계

동일 장비의 RTX6000Ada 두 장에서 물리GPU1/표시GPU0를 사용했다. 기록 생성과 추가 변형 진단을 포함한16초 계산은14.72초였다. GUI를 켜고 pacing한 시험에서는 물리 진행16.00초에 실제16.00초, UI snapshot 갱신은 평균 약38fps였다. 초기 앱 시작·모델 구성·IK 준비 시간은 실행 시작 전 별도이며 이 수치에 포함하지 않는다. 다른 장비에서 같은 속도를 보장하지 않는다.

실시간 기록에서 압착 종료 간격은223.6mm,최종 체적은초기의87.66%,14→16초 체적 변화는−0.0064%,최종 최대 정점 속도는0.057mm/s다. 마지막1초의 판별 변형 속도 평균은0.0056mm/s로 안착 후의 자유 변형이 작다. 같은 표시 메쉬에서 인접 삼각형의 법선각3도 이하로 평가한 평면 면적은 압착 후78.13%,안착 후75.58%였다. 정밀 기준 모드는 각각89.89%,92.51%다.

**실시간 모드는 정밀 모드와 동일한 정확도가 아니다.** 물리 해상도와 반복 수가 줄었고 UR 팔의 동역학을 처방 운동으로 바꿨다. 접힘 위치와 반력,낙하 궤적이 달라질 수 있다. 팔 모터 토크 학습이나 정밀 재료 동정용으로 검증하지 않았다. 원래3458정점 정밀 모드와 기록을 보존했다. 물리 격자의 인접 면 각도는 간격에 따라 달라지므로 n24와n8의 원래3도 지표를 같은 의미로 비교하지 않는다. 원래 값도 `shape_retention.json`에 남기고,해상도가 같은 표시 삼각형 지표를 별도로 비교했다. 표시 평면성은 수치 수렴을 증명하지 않는다.

실시간 전용15개 검사,동작12개 검사,USD10개 검사,단위11개 시험을 통과했다. 실패한 단순 반복 감소,n8 동적 전체 팔,ALM,n12 저반복 시험은 채택하지 않았다. 상세 측정은 `outputs/realtime_validation.json`과 `outputs/realtime/`에 있다.

## 실행과 파일

```bash
# 기본:새 물리를 실시간으로 계산. 앱 준비 후 Play / Resume.
./scripts/python.sh scripts/live_isaac.py

# 기존 정밀 계산 모드
./scripts/python.sh scripts/live_isaac.py --quality reference

# 실시간 자산 재생성
./scripts/python.sh scripts/build_realtime_asset.py
```

기본 자산은 `assets/cardboard_realtime.usda`, `assets/demo_scene_realtime.usda`다. `Recorded replay`는 기록을 다시 보여주며, `Play / Resume`는 새 Newton 계산이다. repo 이름은 기존 `NewtonVBD_WeatherStrip`과 맞춘 `NewtonVBD_CardboardPlasticity`를 추천했으며 경로 변경이나 Git 초기화는 하지 않았다.
