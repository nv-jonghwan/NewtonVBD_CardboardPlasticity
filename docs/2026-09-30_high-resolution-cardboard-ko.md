# Robotiq 골판지 물리 메시 해상도 증가

표면 보간만으로 남아 있던 격자형 주름을 줄이기 위해 실제 물리 셸과 화면용 메시를 함께 세분화했다. 새 정점과 굽힘 힌지에서 전체 집기·압착·낙하 동작을 다시 계산했다. 현재는 `--quality robotiq-fine`으로 제공하는 시각 검토용 버전이다. 기존 `--quality robotiq`는 유지한다.

| 항목 | 이전 | 고해상도 검토 모드 |
|---|---:|---:|
| 면별 격자 |8×8|24×24|
| 물리 정점 |386|3,458|
| 물리 삼각형 |768|6,912|
| 물리 최대 모서리 길이 |40.99mm|13.66mm|
| 표시용 삼각형 |57,280|173,760|
| 표시용 최대 모서리 길이 |약11.8mm|5.90mm|

박스의 초기 외형·중심면 크기·표면적·질량·15mm 재료값과 Robotiq 장착/구동 명령은 유지한다. 메시 변경에 따라 굽힘 힌지의 인덱스·기준각·폭·강성을 다시 계산하고, 원본 텍스처와 UV를 보존하며 표시용 메시를 새 물리 셸에 다시 연결한다. 기존 기록은 정점 수가 달라 재사용할 수 없다.

## 접촉과 수치 설정

15mm 자기 접촉 거리는 새8–11mm 격자 간격보다 크다. 그대로 적용하면 초기부터 이웃 요소가 서로 밀어내므로, 고해상도 장면에만 `cardboard:selfContactRestExclusion=0.01501`m를 설정했다. Newton의 `particle_rest_shape_contact_exclusion_radius`로 초기 형상에서 이미 이 거리보다 가까운 요소 쌍을 제외한다. 그리퍼·테이블 접촉과 비국소 자기 접촉은 유지한다. 기존 장면의 기본값은0이다.

이 옵션의 의미는 설치된 Newton1.6 코드와 [Newton 공식 SolverVBD 문서](https://newton-physics.github.io/newton/latest/api/_generated/newton.solvers.SolverVBD.html)에서 확인했다. 초기 이웃의 접촉을 제외하는 모델링 가정이며, 해당 이웃 쌍이 이후 접혀도 자기 접촉 대상으로 다시 추가되지는 않는다.

초기1초 정지 시험에서48회 반복×16 substeps는 충분히 수렴하지 않아 소성변형이 발생했다.128×16은 소성변형0으로 통과했지만74.3초가 걸렸다. 더 작은 시간 간격의16×64와8×64도 소성변형0으로 통과했고,8×64는25.4초였다.

8×64의 전체16초 시험은393.6초가 걸렸다. 집기 전 소성변형0, 상승90.80mm, 압착 간격192.90mm와 낙하는 확인했지만, 최종 최대 속도13.59mm/s와 관절 연결점 오차2.790mm가 기존10mm/s·2mm 기준을 넘었다. 이 시험은 `outputs/robotiq_fine24_i8s64_full/`에 보존하고,16회 반복×64 substeps로 다시 계산했다. 재료를 부드럽게 바꾸거나 검증 기준을 완화하지 않았다.

## 최종 계산과 제한

16×64 계산은16초 동작에712.66초(약11.9분)가 걸렸다. 전체15개 동작 기준 중14개를 통과했다.

- 집기 전 소성변형0, 양쪽 접촉을 유지한 상승93.39mm.
- 압착 종료 실제 간격179.99mm, 영구 소성 힌지8,051개.
- 전체 기록 중 최대 관절 연결점 오차1.704mm, 관절축 오차0.00116도.
- 놓기·낙하·접지와 유한 좌표는 통과했다.
- **최종 순간 최대 속도12.10mm/s로, 기존10mm/s 안착 기준은 미통과**다. 마지막2초의30Hz 기록 위치에서 계산한 속도는 최대4.25mm/s, 전체 정점/프레임 RMS1.13mm/s다. 두 측정은 표본 간격이 다르므로 서로 대체하지 않는다.

재료·초기 크기·면적/질량·메시 구조와 강한 압착에 대한9개 추가 기준은 통과했다. 동작 전체 통과를 요구하는 결합 기준은 실패로 남겼다. `validation.json`과 `resolution_validation.json`의 `passed`는 모두 `false`이며, 검증 완료 안정판을 대체하지 않는다. 해상도 증가로 큰 격자 단차는 줄었지만 미세한 격자 흔적과 수치 잔진동이 완전히 제거된 것은 아니다. 정량 수렴이나 실물 재료 동정도 검증하지 않았다.

기존24개 단위·형상 회귀 검사는 통과했다.8×64 미리보기의50개 프레임에서 표면 좌표/법선은 모두 유한하고, 법선 길이 오차는1.2e-7 이하였다. 이때 CPU 표면 평가 중앙값은100.7ms로 측정되어, 기록 재생도 화면 갱신 빈도는 기존 메시보다 낮을 수 있다.

| 이전 최종 화면 | 고해상도 최종 화면 |
|---|---|
| ![이전](../outputs/mesh_refinement/coarse_settled.png) | ![고해상도](../outputs/robotiq_fine24/settled.png) |

같은 카메라·조명의 독립 시뮬레이션 결과다. 물리 해상도가 달라졌으므로 최종 접힘 형상도 달라진다.

## 재현과 보존

```bash
./scripts/python.sh scripts/live_isaac.py --quality robotiq-fine
```

오른쪽 **Recorded replay**로 저장된16초 결과를 재생하고 **Box detail**로 상단을 확인한다. **Play / Resume**는 약12분이 필요한 새 물리 계산이다. 기존 검증 완료 장면은 `--quality robotiq`로 실행한다.

재생성 및 검증(현재 마지막 두 검증 명령은 위 잔진동 기준 때문에 종료 코드1):

```bash
./scripts/python.sh scripts/build_refined_robotiq.py
./scripts/python.sh scripts/stream_newton.py --scenario --record \
  --scene assets/demo_scene_robotiq_fine24.usda \
  --stream outputs/robotiq_fine24/state.npz
./scripts/python.sh scripts/validate_scaled_robotiq.py --output outputs/robotiq_fine24
./scripts/python.sh scripts/validate_refined_robotiq.py --output outputs/robotiq_fine24
```

- 기존 장면: `assets/demo_scene_robotiq_coarse_preserved.usda`.
- 기존 기록: `outputs/robotiq_scaled/`.
- 새 자산/장면: `assets/cardboard_robotiq_fine24.usda`, `assets/demo_scene_robotiq_fine24.usda`.
- 시험·진단 로그: `outputs/mesh_refinement/`.

GUI의0.3초 새 물리 실행으로 Play/Pause/Reset을 확인하고, 전체16초 기록의 일시정지·재개·최종 프레임 표시를 검증했다. 두 검증 모두 통과했으며 로그는 `outputs/mesh_refinement/fine24_gui_test.log`, 화면은 `outputs/robotiq_fine24/gui_crush.png`와 `gui_settled.png`다. 최종 GUI PID49904의 실제 창과 Home/t=0/소성변형0/3,458개 유한 정점을 확인했다. 이 GUI 검증은 위 안착 기준의 미통과를 대체하지 않는다.
