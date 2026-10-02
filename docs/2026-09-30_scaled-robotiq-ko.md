# UR10 + 가상 2.3배 Robotiq 2F-140

현재 280 × 208 × 204 mm 박스를 유지하고 NVIDIA가 배포한 Robotiq 2F-140 USD를 2.3배 확대했다. 제품의 명목 개구 140 mm를 기준으로 목표 개구는 322 mm이다. 원본 CAD의 패드 안쪽 기준점으로 측정한 완전 개방 기하 개구는 약332 mm이며, 시나리오에서는322 mm를 명령한다.

## 실행

```bash
./scripts/python.sh scripts/live_isaac.py --quality robotiq
```

오른쪽 Cardboard Scenario의 Play / Resume으로 새 물리 계산, Recorded replay로 검증된16초 기록을 재생한다. 기존15mm 가상 평행 그리퍼 기본 모드는 유지된다. 새 모드에는 `Virtual Robotiq 2F-140 x2.3`가 표시된다.

## 장착과 물리

- UR10과 박스 크기는 그대로이며 `ee_link → Gripper/Palm` 고정 관절을 사용한다. 원본 UR10 손목 메시의 장착면은 `wrist_3_link`의 Z=0, 바깥 방향은 +Z이며 이는 `ee_link`의 **+X**에 해당한다. 기존 +Z축 장착 오류를 수정해 그리퍼 +Z축과 플랜지 법선을 일치시켰다.
- 어댑터는 18 mm 목과 7 mm 상판으로 구성한다. 확대된 그리퍼 바닥이 원점보다 약7.72 mm 뒤에 있으므로 베이스 원점은 플랜지에서32.72 mm 떨어진다. 어댑터 상판과 실제 그리퍼 바닥은25 mm 위치에서 맞닿는다. IK와 개구에 따른 패드 높이 보정도 동일한 장착 축을 사용한다. 실제 볼트 체결 도면이나 하중 검증은 아니다.
- 원본11개 메시,9개 그리퍼 강체,10개 회전 관절과 양쪽 폐루프를 유지한다. 그리퍼의8개 가동 링크는 동역학으로 계산하며 UR10 팔과 그리퍼 베이스는 기존과 같은 처방 운동이다.
- 모든 링크 길이·메시·관절 위치는2.3배, 질량은2.3³배, 관성은2.3⁵배로 확대했다. 원본 베이스에 질량/관성이 없어0.65 kg, COM(0,0,0.04)m, 대각관성(0.0006,0.0006,0.0005)kg m²를 확대 전 가정으로 사용했다. 그리퍼 강체 총질량은약12.877 kg이다. **실제 UR10의 적재하중을 만족한다고 주장하지 않는다.**
- 원본의 단일 모터/차동·언더액추에이션을 완전히 재현하지 않는다. 양쪽 입력을 독립된 암시적 드라이브로 구동하고 외측 손가락 관절에 핀치 자세를 유지하는 스프링을 둔 가상 모델이다. 입력과 핀치 스프링 모두 K=8000 N m/rad, D=80 N m s/rad이다. 이전 K=500, D=8보다 강성과 감쇠를 높였다.
- 입력 스프링의 목표 오차는 `구동 기준 힘 × 0.08 m × scale`의 토크 기준으로 제한한다. GUI의 Drive reference는 이 기준값이며 감쇠·접촉 반력의 절대 상한이 아니다. 집기250 N, 압착4000 N은 가상 구동 기준값이며 실제 Robotiq 사양이나 접촉력 상한이 아니다.
- 원본의 회전 기구를 이용해 개구→입력각을 계산하고, 닫히면서 변화하는 패드 높이에 맞춰 UR10 TCP를 보정한다. 실제 링크 위치나 박스 형상을 강제로 덮어써서 파지하지 않는다.
- Newton VBD가 직접 지원하는 폐루프 회전 관절을 사용한다. USD를 일반 PhysX Play로 여는 것만으로 현재 소성 박스와 커스텀 구동기가 실행되지는 않는다. 제공 launcher/driver가 필요하다.

## 흔들림 감소와 압착 강화 검증

정전 직전 완료된 후보의 기록을 복구하고 전체 궤적을 재검사했다. 기본 Robotiq 자산을 생성하는 스크립트에도 동일 설정을 반영했고, 생성된 USD는 검증 후보와 바이트 단위로 일치한다. 박스의 메시·15mm 재료와 수정된 플랜지는 유지했다.

집기 명령은 해당 구간의70%에서 완료해 상승 전에 잠시 안정화한다. 상승은 구간의85%에서 완료하며, 이동에는 시작·끝 가속도가0인5차 보간을 사용한다. 강한 관절/압착 구동에서도 연결점 오차2mm 기준을 만족하도록 반복을16→32로 늘렸다(substeps16 유지).

| 측정 항목 | 이전 장착 수정본 | 현재 개선본 |
|---|---:|---:|
| 공중 상승 중 패드 상대 각속도 RMS |26.39°/s|1.43°/s|
| 공중 상승 중 박스 상대 속도 RMS |68.89 mm/s|3.90 mm/s|
| 상승 중 박스 최대 입자 속도 |539.77 mm/s|144.12 mm/s|
| 들어 올린 바닥 높이 |90.38 mm|98.97 mm|
| 압착 종료 실제 패드 간격 |260.76 mm|173.84 mm|
| 상승 종료 이후 추가 닫힘 |13.07 mm|101.61 mm|
| 압착 구간 소성 일 |1.82 J|61.87 J|
| 최종 소성 이력 힌지 |214|873|
| 전체 관절 최대 연결점 오차 |1.879 mm|1.833 mm|
| 최종 최대 박스 속도 |0.064 mm/s|5.172 mm/s|
|16초 물리 계산의 실제 시간 |63.49초|122.76초|

흔들림 지표는 테이블에서 박스 바닥이15mm 이상 떨어진 상승 샘플에서 측정했다. 패드 각속도와 박스 중심 속도는 모두 그리퍼 베이스에 대한 상대값이다. 표시 필터나 박스 위치 고정 없이 각각 약94.6%,94.3% 감소했다. 계산 시간은 장비 부하에 따라 달라지며 실시간 동작이 아니다. Recorded replay는16초로 볼 수 있다.

전체 접촉·집기·압착·놓기·안착·플랜지·폐루프 연결15개 검사, 기존 대비 동역학/동일 재료9개 검사, 단위·기하20개 검사를 통과했다. 최종 속도는 기존보다 높지만 기존 안착 기준10mm/s 미만을 만족한다. 완전 정지 또는 실물 재료 동정·수치 수렴을 의미하지 않는다. 최대 영구 굽힘각은약129.6°로 큰 접힘이 발생한다.

- 최종 기록/검증: `outputs/robotiq_scaled/trajectory.npz`, `validation.json`, `dynamics_comparison.json`.
- 압착 렌더: `outputs/robotiq_scaled/crush_overview.png`.
- 이전 장착 수정 결과: `outputs/robotiq_before_dynamics_fix`, `outputs/robotiq_mount_fixed`.
- 이전 설정: `assets/demo_scene_robotiq_mount_baseline.usda`.
- 회귀 검사: `outputs/robotiq_strong_final_tests.log`.

이 모델은 **실제 접촉 계산으로 박스를 집고 변형시키는 가상 확대 그리퍼**다. 현재 실제 압착 간격174mm는 원래 박스 폭280mm의약62%이다. 상용 Robotiq 구동 성능·UR10 허용하중 검증이나 기존 평행 그리퍼의140mm 반폭 압착 달성을 의미하지 않는다.

## 재현 및 출처

```bash
./scripts/python.sh scripts/fetch_robotiq_assets.py
./scripts/python.sh scripts/build_scaled_robotiq.py --scale 2.3 --iterations 32 --substeps 16
./scripts/python.sh scripts/stream_newton.py --scenario --record \
  --scene assets/demo_scene_robotiq_scaled.usda --stream outputs/robotiq_scaled/state.npz
./scripts/python.sh scripts/validate_scaled_robotiq.py --output outputs/robotiq_scaled
./scripts/python.sh scripts/compare_robotiq_dynamics.py --candidate outputs/robotiq_scaled
./scripts/python.sh -m unittest discover -s tests -v
```

원본 파일은`assets/source/robotiq_2f140`에 보존하며 URL·해시는 해당`manifest.json`에 기록했다. 파생 자산 생성기는 원본을 수정하지 않는다. 원본 NVIDIA/Robotiq의 권리·배포 조건을 유지하며, 파생 가상 모델을 상용 제품 또는 인증된 SimReady 모델로 표현하지 않는다.

- [NVIDIA 공식 로봇/그리퍼 자산 목록](https://docs.isaacsim.omniverse.nvidia.com/latest/assets/usd_assets_robots_manipulator.html)
- [원본 NVIDIA USD](https://omniverse-content-production.s3.us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Robots/Robotiq/2F-140/Robotiq_2F_140_physics_edit.usd)
- [Robotiq 실제 제품 사양](https://robotiq.com/products/adaptive-grippers)

GUI 제어 및 전체 기록 재생의 최종 재검증 로그는 `outputs/robotiq_strong_gui_test.log`에 저장한다. `outputs/robotiq_scaled/gui_crush.png`, `gui_settled.png`는 실제 GUI 뷰포트 결과다. 물리 계산 시간은 기록 검증의 `validation.json`, GUI 전달 성능은 `live_performance.json`을 사용한다.

재부팅 후 새 GUI 물리 계산에서도12개 시퀀스 기준을 통과했다: 상승99.09mm, 실제 압착174.44mm, 최종 최대 속도0.958mm/s,16초 계산119.67초. 전체 GUI 전달은약8.06fps였다. `live_sequence_validation.json`과 `live_full_performance.json`에 보존했다. Play/Pause/Reset 및16초 기록 재생의 Pause/Resume/최종 프레임 검사를 통과했다.

움직이는 기록을 캡처할 때 일시적으로 검은 면이 나타나, 자동 시각 검증에서는11.8초 압착 프레임을 일시정지하고0.5초 렌더 안정화 후 캡처한다. 재검증에서 정상 표면과 접힘을 확인했다. 이는 검증 캡처 타이밍 보완이며 물리·재생 궤적을 바꾸지 않는다. 추가0.3초 제어/전체 기록 시험은 `outputs/robotiq_strong_capture_test.log`, 해당 짧은 시험 성능은 `capture_test_performance.json`으로 분리했다.

표면 후속 개선: 구겨진 상단의 격자 느낌을 줄이는 표시용 두께 보간을 적용했다. 물리 결과와 재료는 동일하다. [비교·검증·한계](2026-09-30_natural-cardboard-surface-ko.md).
