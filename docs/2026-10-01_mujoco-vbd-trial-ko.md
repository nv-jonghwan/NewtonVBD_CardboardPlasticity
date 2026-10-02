# 같은 해상도의 MuJoCo–VBD 결합 시험

## 결론

Newton 1.6 안에서 MuJoCo의 관절 계산과 기존 VBD 종이 계산을 연결하는 **별도 실험 구현을 완료했다. 현재 시험 후보 중 기존 품질 기준을 모두 만족하는 후보는 없다.** 빠르게 끝난 실행을 곧바로 최적화 성공으로 채택하지 않았다. 현재 GUI, 기본 장면, 기존 결과와 공유 Python 환경은 교체하지 않았다.

물리 정점 3,458개 / 삼각형 6,912개, 표시 삼각형 173,760개, 종이 두께 10 mm, 막·굽힘·소성·손상 법칙, 그리퍼 힘·궤적, 자기접촉과 정지 판정 기준을 유지했다. 표시만 같은 축소 모델이나 기록 재생 방식이 아니다.

## 구현과 수정

- `.workspace/mujoco-vbd-trial1/runtime`에 MuJoCo 3.12.0 / mujoco-warp 3.12.0을 분리 설치했다. Newton 1.6.0 / Warp 1.17.0 / NumPy 2.3.1을 사용한다. 기존 Newton 환경은 읽기만 한다. 정확한 추가 의존성은 `requirements.lock`, 전체 핵심 버전과 경로는 `runtime_manifest.json`에 있다.
- `src/cardboard/coupled_trial.py`는 Newton의 `SolverCoupledProxy`를 사용하며 접촉 반력을 강체 쪽으로 되돌린다. 기본 `Simulation`은 `coupled_options=None`이므로 기존 VBD 경로를 사용한다.
- imported joint mode가 NONE이어도 기존 VBD는 설정된 PD 계수를 사용했지만 MuJoCo에서는 구동기가 생기지 않았다. 결합 모델 view에서 해당 관절의 POSITION mode를 명시했다. 종이나 원본 USD의 계수는 바꾸지 않았다.
- 자세에서 복원한 관절 좌표를 매 substep MuJoCo에 덮어쓰는 방식은 이동 구간에서 불안정해졌다. 기존 VBD 기록에서도 `-6.2537 rad`와 `0.0295 rad`처럼 같은 자세를 나타내는 2π 분기가 확인된다. 현재 구현은 MuJoCo의 동적 관절 상태를 유지하고 처방된 팔 관절만 동기화한다. 결합 반복 재시작은 native qpos/qvel/warm-start 스냅샷으로 복원한다. 벤치마크의 조기 중단 검사도 각도의 주기를 고려한다.
- 결합 view가 복사한 hinge 배열은 원본과 순서가 같은지 확인한 후 rest angle / bending stiffness 배열을 공유하도록 했다. 소성 이력은 기존 parent state의 return map에서 substep마다 갱신한다. 중력 ramp도 두 solver에 전달한다.
- MuJoCo로 합성된 폐루프 CONNECT 제약은 기본 20 ms time constant에서 최대 7.9 mm 연결 오차를 보였다. 종이 재료를 바꾸지 않고 관절 제약의 수치적 탄성을 2.5 ms / 2.1 ms로 시험했다. [MuJoCo solver parameters](https://mujoco.readthedocs.io/en/stable/modeling.html#solver-parameters)의 solref 의미를 따른다.
- 자유롭게 움직이는 결합용 강체만으로는 이 강한 파지 조건을 안정적으로 유지하지 못했다. VBD 쪽 결합용 강체에도 관절을 유지하는 `--proxy-joints`가 필요했다. 따라서 그리퍼 관련 계산을 VBD에서 완전히 제거하지는 못했다.

활성 구간의 종이 반복은 원래 예산과 같은 substep당 17회이고, 지지된 정지 구간은 48회다. `--proxy-joints`에서는 VBD 관절도 이 횟수만큼 풀고 MuJoCo 계산이 추가된다. 이 비용과 정지 구간에 머무는 시간이 실제 전체 속도를 좌우한다.

## 16초 전체 시험 결과

| 시험 출력 폴더 | 계산 시간 | 시퀀스·관절 15개 | 정지·속도 8개 | 주된 미통과 항목 |
|---|---:|---:|---:|---|
| `baseline_same_harness_v2` (기존 VBD) | 142.66초 | 15/15 | 7/8 | post-14s 정지; 기존 미완료 항목 |
| `retained_full16` | 71.56초 | 11/15 | 8/8 | 파지 소실, 들기 실패, 관절 연결 오차 |
| `tight_loop16` | 159.41초 | 12/15 | 5/8 | 파지·들기, 최종 정지 |
| `articulated_proxy` | 93.75초 | 14/15 | 8/8 | 관절 연결 2.240 mm > 2 mm, 압착 중 움직임·턱 되열림 |
| `articulated_proxy21` | 158.12초 | 15/15 | 5/8 | post-14s 정지, 압착 중 움직임·턱 되열림 |
| `articulated_relaxed` | 182.15초 | 13/15 | 4/8 | 관절 연결 2.335 mm, 최종 정지, 압착 중 움직임 |

모든 폴더는 `outputs/coupled_trial/` 아래에 있다. 중간에 파지 기준을 충족하지 못한 force exchange 2회 / proxy mass 10배 시험은 압착 전에 중단했다. 이는 전체 실행 시간으로 비교하지 않는다. 초기 변환 실패나 진단용 실행도 보존했다.

동일 벤치마크에서 기존 방식은 142.66초였으므로 158.12초 후보는 약 10.8%, 182.15초 후보는 약 27.7% 더 느리다. 93.75초 후보의 약 34.3% 시간 감소는 품질 미통과 결과이므로 채택 가능한 개선이 아니다. 각각 한 번의 전체 실행이며 반복 측정에 의한 속도 보장은 아니다.

기존 방식의 이번 재실행도 엄격한 정지 검사는 통과하지 못했다. post-14s 옆면의 최종 위치 대비 최대 변위는 8.160 mm이고 최종 수면 진입은 15.150초였다. 이전 저장 결과의 0.261 mm와 차이가 커서 정지 거동의 반복 재현성 역시 미완료 상태다. 기존 방식의 시퀀스 15개 및 crease 13개 검사는 통과했다. 별도의 기존 회귀 검사 43개도 모두 통과했고, Python 문법 검사와 격리 환경 import 검사를 통과했다.

`articulated_proxy`는 79.6 mm 들어 올렸고, 약 155.5 mm까지 압착했으며, 12.817초에 지지된 수면 상태로 들어갔다. 하지만 압착 유지 중 턱의 되열림 누적 경로가 7.04 mm로 기존 검토 결과 1.17 mm보다 커졌다. 한 항목의 속도·정지만 보고 채택할 수 없다.

`articulated_proxy21`는 관절 연결 오차 1.941 mm로 시퀀스 15개를 통과했지만, 16초 시점에도 최대 속도 8.49 mm/s가 남았고 엄격한 정지 기준을 통과하지 못했다. 원래 시퀀스의 <10 mm/s 기준과 사용자가 요구한 정지 기준을 혼동하지 않는다.

마지막 `articulated_relaxed`부터 반력 진단에 VBD의 substep 시작 pose snapshot을 사용한다. 앞선 시험은 갱신된 pose history를 사용했으므로 과거 반력 수치는 잠정 진단값이다. 접촉력이 solver 사이로 전달되는 native 경로는 별개였지만, 보고값 및 정지 모드의 loaded 판정과 연관되므로 앞선 결과로 최종 적격성을 선언하지 않는다.

표의 71.56초나 93.75초를 적격한 속도 개선율로 제시해서는 안 된다. 정지 검사만 통과해도 파지에 실패했다면 전체 결과는 실패다. 모든 검사 기준을 유지했고, 불합격 후보를 기본 결과로 승격하지 않았다.

## 재현

실험 환경이 생성된 이 작업 공간에서 실행한다. 출력 폴더는 새 경로여야 하며 기존 결과는 덮어쓰지 않는다.

```bash
scripts/python_coupled_trial.sh scripts/benchmark_coupled_trial.py \
  --output outputs/coupled_trial/recheck \
  --duration 16 --proxy-joints --loop-time-constant .0021

scripts/python.sh scripts/benchmark_coupled_trial.py \
  --output outputs/coupled_trial/baseline_recheck \
  --duration 16 --baseline
```

각 결과에는 궤적, material state, CSV, 실행 시간, 설정과 소스 해시가 저장된다. GPU 최초 컴파일/초기화 비용은 반복 실행과 구분해야 한다. `performance.json`의 init_seconds는 생성 비용이고 wall_time_s는 advance/기록 구간이며 첫 frame의 eager warmup/graph 준비는 포함한다. GUI의 화면 전달 비용은 포함하지 않는다.

이번 구현은 고정된 기존 시나리오를 비교하기 위한 실험 경로다. GUI 편집, 임의의 강체 checkpoint 복원, 외부 pose teleport를 지원하는 범용 결합 backend로 검증한 것은 아니다. `partition=gripper`는 원인 분리용 진단 경로이며 채택 후보가 아니다.

이 결과는 이번 구현·설정의 한계다. 모든 MuJoCo/VBD 결합 방식이 느리다는 결론은 아니다. 현재의 강한 압착과 고강성 셸에서는 관절 처리뿐 아니라 종이 및 접촉의 수렴을 가속해야 한다. 재료를 더 부드럽게 만들거나 정지 판정 기준을 완화하는 방식으로 속도를 맞추지는 않았다.
