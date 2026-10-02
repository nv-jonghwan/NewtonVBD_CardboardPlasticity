# VBD 단계별 프로파일과 최적화 분석 — 2026-10-01

현재 primitive 그리퍼 장면을 새 상태에서 16초 끝까지 실행하고, CUDA graph 시간·단계별 커널·접촉 수를 측정했다. 결론은 **GPU 물리 계산이 병목이며, 낙하 후 supported 계산량과 강체 측 접촉 누적의 병렬화가 우선순위**라는 것이다. 강체당 접촉 worker를 4개에서 32개로 늘린 격리 시험은 상승/압착의 해당 커널 배치를 1.96/2.42배 가속했지만, 무접촉 구간에서는 24% 느려졌다. 따라서 고정 worker 증가를 기본 계산기에 적용하면 안 된다.

이번 산출물은 실제 프로파일, 격리 커널 실험, 구현/검증 계획이다. **전체 시뮬레이션 가속은 아직 입증하지 않았고 기본 solver에 후보를 적용하지 않았다.** 원본 장면·물성·메쉬·시간 간격·solver 설정과 공유 toolchain은 유지했다. 전체 궤적 검사는 기존과 같이 14/15 통과하며 안착 속도 기준은 실패했다.

## 1. 측정 범위와 신뢰도

| 항목 | 조건 |
|---|---|
| 장면 | `assets/demo_scene_robotiq_board10_primitive.usda` |
| 계산 | Newton 1.6.0 / Warp 1.17.0, 원본 full VBD 경로 |
| GPU | RTX 6000 Ada, `cuda:1` |
| 물리 모델 | 정점 3,458 / 삼각형 6,912 / 굽힘 hinge 10,368 / 두께 10 mm |
| 색 그룹 | particle 8개, dynamic rigid body 그룹 크기 2/4/2 |
| 시나리오 | home → 이동 → 하강 → 집기 → 상승 → 압착 → 놓기 → 안착, 16초 |
| 상세 샘플 | 시뮬레이션 약 0.5 / 2 / 6 / 8 / 11 / 13.5 / 15초 |

일반 프레임에서는 기존 CUDA graph 전후에 재사용 event를 기록했다. 상세 샘플은 동일한 `_integrate()`를 event가 들어간 graph로 캡처하여 그 프레임에 한 번 실행했다. 추가 물리 step은 실행하지 않았다. **커널별 attribution은 첫 substep만 측정**했고, 상세 샘플과 mode 진입 초기 2프레임은 일반 프레임 통계에서 제외했다. 최종 일반 통계는 945프레임이다.

Nsight Systems/Compute가 설치되어 있지 않아 이번에는 Warp/CUDA event와 설치된 커널 소스를 사용했다. achieved occupancy, DRAM bandwidth, warp stall 원인은 아직 측정하지 않았다. CUDA event를 작은 커널마다 삽입하면 측정 오버헤드와 시간 해상도가 결과에 영향을 주므로, 상세 attribution은 병목 후보를 찾는 용도로 사용해야 한다. [Warp profiling 문서](https://nvidia.github.io/warp/latest/user_guide/execution_and_performance/profiling.html)

시작 시 GPU1은 유휴 상태였으나 이후 다른 프로젝트 CUDA context가 관찰되었다. GPU 상태와 process snapshot을 기록했으며 다른 프로세스는 건드리지 않았다. 따라서 숫자는 **공유 GPU에서 얻은 관측치**이며 독점 GPU 반복 비교 결과는 아니다. 최신 문서는 원리 확인에 사용했고, 구현 분석은 설치된 1.6.0/1.17.0 소스를 기준으로 했다.

## 2. 실제로 시간이 드는 위치

![프레임 시간과 첫 substep 커널 비중](../outputs/vbd_profile/phase_profile.png)

일반 프레임 중앙값:

| 단계 | 프레임 수 | 전체 ms/frame | GPU graph ms/frame | 해석 |
|---|---:|---:|---:|---|
| 초기 home | 57 | 132.1 | 130.0 | active |
| 이동 | 117 | 13.3 | 11.8 | active 43 + sleep 74; 중앙값은 sleep 쪽 |
| 하강 | 118 | 13.4 | 12.0 | active 17 + sleep 101; 중앙값은 sleep 쪽 |
| 집기 | 119 | 143.1 | 141.1 | active |
| 상승 | 119 | 158.9 | 157.0 | active |
| 압착 | 179 | 171.2 | 169.3 | active |
| 놓기 | 58 | 478.9 | 477.0 | active 16 + supported 42 |
| 안착 | 178 | 476.6 | 474.2 | supported |

일반 프레임의 전체 시간 합에서 **GPU graph가 99.09%**, supported 프레임이 **54.44%**를 차지한다. 통계에 포함된 시간 합 192.745초는 상세 샘플 등을 제외한 값이며, 정상 실행의 16초 전체 벤치마크 시간으로 사용하면 안 된다.

프레임당 GPU→NumPy 호출은 17회, 반환 데이터는 374,448 bytes다. 그러나 `.numpy()`의 벽시계 시간에는 앞선 GPU 작업을 기다리는 시간이 포함된다. 이를 모두 복사 비용으로 해석하면 잘못된 병목을 고르게 된다. 이 측정에서 graph 바깥 시간을 전부 없애더라도 이상적인 가속 상한은 약 **1.009배**다. Python 최적화가 물리 계산 시간을 크게 줄일 가능성은 낮다.

표면 생성은 별도 측정에서 중앙값 25.8–36.0 ms였다. 이는 물리 graph 통계에 포함되지 않은 `PanelSurface` 평가 비용이며 USD 전송·RTX 렌더링·GUI 전체 FPS는 아니다. 인터랙티브 표시 지연을 개선할 때는 별도 목표로 다루어야 한다.

### supported 모드에서 계산량이 증가한다

| 모드 | substeps/frame | rigid iterations/substep | shell sweeps/substep | shell sweeps/frame | 계측된 `wp.launch` 호출/frame |
|---|---:|---:|---:|---:|---:|
| active | 16 | 48 | 17 | 272 | 22,880 |
| supported | 32 | 48 | 48 | 1,536 | 87,232 |
| sleep | 16 | 8 | 0 | 0 | 1,856 |

active는 particle solve interval 3, supported는 interval 1이다. 안착 후 shell 반복은 **5.65배**, 강체 반복은 **2배**가 된다. 마지막 약 3.7초의 시뮬레이션이 전체 측정 시간의 절반 이상을 쓰는 구조적 이유다. `wp.launch` 수는 memcpy/memset 등 내장 호출을 포함하지 않으므로 CUDA graph 전체 node 수와 같지 않다.

이번 full trajectory의 최종 속도는 **20.29 mm/s**로 기존 안착 기준 10 mm/s를 넘었다. 계산량이 늘었어도 정지 품질이 보장되지 않는다는 뜻이다. 단순히 반복을 줄이거나 sleep을 강제하기 전에, 물리적인 잔진동인지 contact/solver의 수렴 문제인지 분리해야 한다.

### 압착과 안착은 병목 구성이 다르다

아래는 event가 들어간 첫 substep의 커널 시간 비중이다. 전체 단계의 정확한 비용 비율로 일반화하지 않는다.

| 커널 계열 | 압착 약 11초 | 안착 약 15초 |
|---|---:|---:|
| 강체 측 접촉 힘/Hessian 누적 | 27.8% | 7.5% |
| 강체 solve | 19.0% | 13.2% |
| shell 탄성 solve | 8.5% | 17.2% |
| 자기접촉 이동 제한/truncation | 9.5% | 18.2% |
| 자기접촉 힘 누적 | 7.2% | 12.2% |
| 자기접촉 검출 | 8.2% | 4.9% |
| particle 측 rigid 접촉 | 5.4% | 10.6% |
| shell 전역 보정 | 5.2% | 9.6% |
| dual 갱신 | 7.7% | 5.4% |
| 소성 이력 갱신 | 0.04% | 0.03% |

압착에서는 소수의 강체가 접촉을 처리하는 경로가 크고, 안착에서는 반복되는 shell/자기접촉 계산이 크다. 소성 이력 계산을 생략하거나 줄이는 것은 이 프로파일에 맞는 최적화가 아니다.

## 3. 직접 시험한 최적화 후보

격리 microbenchmark는 새 시뮬레이션을 지정 시점까지 진행한 뒤, 해당 상태의 kernel 입력을 고정했다. 출력은 별도 버퍼를 쓰고 매 호출 전 복원했으며 입력/출력 alias도 보존했다. 128회 graph batch, warm-up 3회, 측정 9회의 중앙값을 사용했다. 시간에는 출력 복원 복사가 포함된다. 표의 시간은 모든 관련 색 그룹에 대한 중앙값의 합이다.

총 **158개 커널/설정 조합**에서 원본 출력과 `rtol=1e-5, atol=1e-5` 비교를 통과했다. 이는 고정 상태의 출력 동등성 검사이며, 158개의 전체 시뮬레이션 회귀 검사가 아니다.

### 후보 A: 강체 측 접촉 worker를 접촉 수에 맞춰 조절

`accumulate_body_particle_contacts_per_body`는 원본에서 강체당 worker 4개를 사용한다. dynamic body 색 그룹이 2/4/2개여서 한 그룹의 접촉 처리를 소수 스레드가 담당한다. 설치된 커널의 접촉법칙·force/Hessian 계산은 그대로 두고 worker 수만 인자로 바꾼 독립 커널을 만들었다.

![접촉 worker 실험](../outputs/vbd_profile/contact_workers.png)

| worker/body | 무접촉 0.5초 µs | 상승 8초 µs | 압착 11초 µs |
|---|---:|---:|---:|
| 원본 4 | 17.91 | 56.28 | 76.44 |
| 8 | 18.56 | 43.76 | 56.10 |
| 16 | 19.48 | 32.55 | 38.52 |
| 32 | 22.14 | 28.73 | 31.61 |
| 64 | 29.24 | 29.35 | 31.26 |

32개는 상승에서 **1.96배**, 압착에서 **2.42배** 빠르다. 출력별 최대 절대 차이를 해당 출력의 최대 성분 크기로 나눈 값은 최대 **2.54×10⁻⁷**이었다. 이 수치는 성분별 상대오차 또는 전체 궤적 오차가 아니다. 합산 순서 변화에 따른 작은 부동소수점 차이도 장시간 비선형 접촉에서는 궤적 차이로 커질 수 있다.

반대로 강체와 박스의 접촉이 없는 0.5초에는 32개가 **23.6% 느리고**, 64개는 **63.2% 느리다**. 이때에도 테이블과 박스의 soft contact는 존재하므로 “모든 접촉이 없는 장면”이라는 뜻은 아니다. 측정한 supported 시점에서도 dynamic body 측 접촉 수는 0이었다. 전체 모드에 worker 32를 일괄 적용하는 변경은 권장하지 않는다.

구현 우선안은 다음과 같다.

1. body별 contact count가 0이면 불필요한 계산과 누적을 건너뛰는 경로를 설계한다. force/Hessian 초기화 및 다른 접촉 항과의 누적 의미는 보존한다.
2. count가 적을 때는 소수 worker, 많을 때는 32개 정도를 사용한다. 분기 임계값은 접촉 수별 sweep으로 정하며 아직 확정하지 않았다.
3. CPU로 count를 읽어 graph를 바꾸지 않고, 고정 graph 내부의 GPU 분기로 구현 가능한지 검증한다. 고정 최대 launch에서 비활성 worker를 반환하는 방법도 launch 비용 자체는 남으므로 재측정이 필요하다.
4. 모든 접촉의 처리, torque/Hessian, friction, color별 Gauss–Seidel 순서를 보존하고 전체 16초 A/B 검증을 수행한다.

압착 상세 샘플에서 이 특정 body-particle 커널은 약 22.7%였다. 이 비중이 전체 압착 단계에도 같고 커널 가속이 2.42배 그대로 유지된다고 **가정하면**, Amdahl 식 `1 / (1 - 0.227 + 0.227 / 2.42)`은 약 **1.15배**다. 이는 조건부 추정이며 측정된 전체 가속 배수가 아니다. 전체 실행에서는 supported 비중과 무접촉 손해 때문에 효과가 더 작거나 사라질 수 있다.

### 후보 B: block size 및 contact capacity만 조절 — 효과가 작았다

| 시험 | 0.5초 | 11초 | 판단 |
|---|---:|---:|---|
| particle-rigid 접촉 block 256→32 | 42.92→40.03 µs | 42.43→40.14 µs | 해당 커널 배치 1.06–1.07배 |
| particle-rigid 접촉 실제 count만 launch | 42.92→39.76 µs | 42.43→40.16 µs | 1.06–1.08배, production graph 구현은 아님 |
| rigid solve block 256→32 | 45.37→47.15 µs | 44.06→44.71 µs | 개선 없음 |

soft contact capacity는 82,992이고 관측 count는 수백 개이므로 큰 낭비처럼 보이지만, count를 넘는 스레드는 일찍 반환한다. 실제 count launch 시험은 이 변경만으로 큰 가속을 얻기 어렵다는 증거다. host count read를 사용하는 oracle이므로 그대로 실시간 경로에 넣을 수도 없다.

`solve_rigid_body`는 compiled property상 255 registers/thread, local memory 32 bytes/thread였다. 이는 낮은 body 수 및 복잡한 solve와 함께 검토할 단서지만, 실제 occupancy나 메모리 병목을 입증하지 않는다. block size 축소만으로 body-level 병렬성이 늘지 않는다는 점은 실험과 일치한다.

또한 **총 soft contact capacity와 body별 contact buffer 256은 서로 다르다**. 압착 상세 샘플에서 한 body의 접촉 수는 최대 244였다. 메모리를 줄이겠다며 body별 buffer를 줄이면 접촉 손실 위험이 있다. 모든 substep의 high-water mark와 overflow diagnostics를 먼저 확보해야 한다. 샘플에서 256 미만이었다는 사실만으로 전체 구간의 overflow 부재를 주장할 수 없다.

## 4. 프로파일에 근거한 구현 순서

| 순위 | 작업 | 이유와 승인 기준 |
|---|---|---|
| 1 — 작은 범위의 실증 후보 | body 접촉 count 0 경로 + 적응형 worker | loaded 커널 가속은 측정됨. unloaded 손해 제거와 전체 A/B가 남음 |
| 1 — 큰 비용 구조 | supported 반복량과 안착 수렴 분석 | 측정 시간 54.4%. 정확도를 유지하는 반복 정책을 찾으면 영향 범위가 큼 |
| 2 | 색별 접촉 목록 및 안전한 kernel fusion | 22,880/87,232 호출과 반복 메모리 접근을 줄일 후보. 새 프로파일과 동등성 필요 |
| 3 | rigid solve 특화 / block 내부 협력 계산 | body 8개와 높은 register 사용에 맞는 실행 구조 검토. Nsight 근거 필요 |
| 후순위 | 단순 block 조절, capacity trim, NumPy 병합 | 이번 측정에서 개선 여지가 작거나 전체 기여가 작음 |
| 별도 목표 | 표면 생성과 표시 경로 | GUI 지연 개선용; 물리 solver 속도와 분리 측정 |

### supported 반복 정책의 구체적 분석 방법

먼저 낙하 충격 직후와 안정화 이후를 분리하고, substep/iteration별 위치 보정량·정규화 force residual·최대 침투·접촉 수·에너지/속도·소성 work를 함께 기록한다. residual 자체의 GPU reduction 비용도 측정해야 한다. `16/24/32` substeps 및 여러 iteration budget을 격리 시험하되 시간 간격과 반복 횟수를 동시에 바꾼 결과를 하나의 효과로 해석하지 않는다.

고정된 적은 iteration보다, 충격 직후 충분히 계산하고 수렴이 확인된 이후에만 반복을 줄이는 정책을 우선 검토한다. CUDA graph는 반복 budget별 graph를 미리 캡처하고 저빈도 정책 전환을 쓰거나 GPU 내부 수렴 판정을 검토할 수 있다. 여기에는 알고리즘/정확도 변경이 포함되므로 기존 full VBD trajectory 및 더 촘촘한 convergence reference와 비교해야 한다.

sleep 강제, 속도/변위 동결, 안착 threshold 완화, damping 임의 증가는 성능 개선의 근거로 사용하지 않는다. 현재 기준 자체가 실패하고 있으므로 빠르다는 이유만으로 전체 품질을 통과했다고 판정할 수 없다.

### fusion 및 실행 구조를 바꿀 때 지킬 조건

VBD는 인접한 vertex의 갱신 순서와 graph coloring에 의존한다. 같은 색의 독립 계산과 색 사이의 동기화 경계를 구분해야 한다. 색을 가로지르는 무조건적인 fusion이나 충돌 쌍 생략은 단순 성능 변경이 아니다. 색별 활성 접촉 목록을 만들어 반복적인 color reject를 줄일 수 있는지, 목록 구축 비용이 반복에서 회수되는지를 먼저 측정한다. [VBD 원논문](https://arxiv.org/html/2403.06321v4)

현재 이미 CUDA graph caching, compact self-contact 목록, sparse 표면 보간, 전역 shell 보정이 있다. 이를 신규 최적화처럼 중복 제안하지 않는다. graph가 적용되어도 GPU 측 개별 커널 실행과 데이터 이동은 남는다. NVIDIA가 설명하는 graph의 constant-time 개선은 반복 graph의 **CPU launch 비용**에 관한 것으로, 모든 GPU 커널이 일정한 총 시간에 실행된다는 뜻은 아니다. [CUDA graph 성능 설명](https://developer.nvidia.com/blog/constant-time-launch-for-straight-line-cuda-graphs-and-other-performance-enhancements)

강체 AVBD를 persistent/cooperative 형태로 바꾸는 것은 장기 후보다. 8개 body에서의 제약·friction·dual·행렬 solve를 함께 고려해야 하며, 많은 body를 사용하는 논문 결과를 이 장면의 가속 배수로 가져올 수 없다. [AVBD 연구 및 원논문](https://graphics.cs.utah.edu/research/projects/avbd/), [Newton SolverVBD 문서](https://newton-physics.github.io/newton/latest/api/_generated/newton.solvers.SolverVBD.html)

기존 7색 시험은 recoil 문제로, rank8 ROM 및 MuJoCo 결합 시험은 속도/정확도 기준 미달로 기본 적용되지 않았다. 이번 프로파일만으로 해당 후보를 다시 통과 처리하지 않는다.

## 5. 다음 최적화의 검증 규격

- 동일 GPU에서 fresh state로 baseline/candidate를 순차 ABBA 실행하고 최소 3회 반복한다. compilation/init, 상세 계측 시간, 물리 advance, 표면/표시 시간을 구분한다. 다른 프로젝트 작업을 중단하지 않고 GPU 사용 상태를 기록한다.
- 기본 조건뿐 아니라 기존 변경 command 조건에서도 16초 전체를 검증한다. 격리 커널 동등성이 전체 비선형 trajectory 동등성을 대신하지 않는다.
- finite state, 양손 접촉 및 lift, release/landing, joint anchor 2 mm, axis 2도, 소성 이력 보존 및 비음수 소성 work를 확인한다. 기존 안착 10 mm/s와 엄격한 post-14s 이동 0.1 mm 기준은 그대로 사용한다.
- baseline 자체의 기존 실패와 후보가 만든 새 실패를 분리한다. 기존 실패가 남으면 전체 품질 통과 또는 production 승격으로 기록하지 않는다.
- force/Hessian, 접촉 count/high-water/overflow, 최대 penetration, 전체 형상 오차를 함께 기록한다. event 없는 전체 시간의 중앙값/분산과 구간별 시간을 비교한다.
- Nsight가 사용 가능해지면 Systems로 graph/stream gap과 커널 빈도를 확인하고, Compute는 대표 접촉/rigid/shell 커널만 좁혀 occupancy·register/spill·memory throughput·stall을 측정한다. profiling replay와 계측 오버헤드는 일반 시간에서 제외한다. [Nsight Compute profiling guide](https://docs.nvidia.com/nsight-compute/ProfilingGuide/index.html)

## 6. 재현 및 산출물

각 명령의 output은 존재하지 않는 새 디렉터리여야 한다. 아래 `*_rerun` 디렉터리는 기존 증거를 덮어쓰지 않는다.

```bash
./scripts/python.sh scripts/profile_vbd_phases.py --device cuda:1 --duration 16 --output outputs/vbd_profile/full_rerun
./scripts/python.sh scripts/validate_scaled_robotiq.py --output outputs/vbd_profile/full_rerun
./scripts/python.sh scripts/benchmark_vbd_launches.py --device cuda:1 --times 0.5 11 --output outputs/vbd_profile/launch_bench_rerun
./scripts/python.sh scripts/benchmark_vbd_launches.py --device cuda:1 --parallel-contact --times 8 11 --output outputs/vbd_profile/contact_parallel_rerun
./scripts/python.sh scripts/benchmark_vbd_launches.py --device cuda:1 --parallel-contact --times 0.5 --output outputs/vbd_profile/contact_parallel_unloaded_rerun
```

요약 스크립트는 최초 측정의 고정 경로를 읽는다. 위 재실행 결과를 비교하려면 입력 경로를 명시적으로 변경한다.

```bash
./scripts/python.sh scripts/summarize_vbd_profile.py
```

- [프로파일 스크립트](../scripts/profile_vbd_phases.py), [격리 benchmark](../scripts/benchmark_vbd_launches.py), [worker 실험 커널](../scripts/vbd_contact_parallel_trial.py), [통계·그림 생성](../scripts/summarize_vbd_profile.py)
- [가공 통계 JSON](../outputs/vbd_profile/analysis.json), [원본 프로파일](../outputs/vbd_profile/full/profile.json), [장면/소스 hash 및 실행 조건](../outputs/vbd_profile/full/run_config.json), [전체 시나리오 검증](../outputs/vbd_profile/full/validation.json)
- [block/count 실험](../outputs/vbd_profile/launch_bench/benchmark.json), [loaded worker 실험](../outputs/vbd_profile/contact_parallel/benchmark.json), [unloaded 대조군](../outputs/vbd_profile/contact_parallel_unloaded/benchmark.json)
- `outputs/vbd_profile/full/`의 `state.csv`, `trajectory.npz`, `final_material_state.npz`, 상세 kernel JSON과 로그를 보존했다.

프로파일 수집 후 geometry-query 분류 이름을 보완했다. 기존 raw JSON은 유지하고 요약 스크립트에서 해당 항목을 self-contact detection으로 재분류했다. 따라서 현재 profiler 스크립트 hash는 최초 수집 시 hash와 다를 수 있으나 물리 source/scene hash는 별도로 검증한다. 전체 solver 코드 변경과 production 가속 주장은 없다.
