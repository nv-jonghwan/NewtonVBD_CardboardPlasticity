# 골판지 ROM/VBD 전환 실험과 종료 후 복구

현재 rank8 후보는 기본·변경 조건 모두 속도·형상 기준을 통과하지 못했다. 전체16초 새 계산은 기본 조건에서 기존212.20초/ROM290.01초, 변경 조건에서 기존235.52초/ROM273.07초였다. 단위·회귀 검사49개는 통과했지만 이것이 전체 궤적의 등가성을 뜻하지는 않는다. 후보는 별도 실험으로 유지한다.

## 범위와 구현

2026-10-01 종료 전 남아 있던 `rom_basis.py`, `rom_solver.py`, 학습 기저와 1초 시험을 복원하고 전체 동작 검증을 이어 진행했다. 기존 primitive 그리퍼, 상판 충돌, 10mm 재료, 상승 명령20cm 장면을 기준으로 한다. 물리 정점3,458개/삼각형6,912개, 접촉 표면과 각 힌지의 소성 상태를 유지한다. GUI 기본 계산기는 변경하지 않는다.

전체 좌표 자유도는10,374개다. 학습 데이터 `outputs/primitive_gripper/mesh_lift20/trajectory.npz`의 위치 증분에서 평균 이동을 빼고 POD를 수행했다. 병진3개와 변형5개의 고정 기저를 사용한다. 학습 증분의 중심 이동을 제외한 제곱합97.849%를 포착하지만, 이는 접촉력이나 새 동작의 정확도 보증이 아니다. 기저 SHA와 학습 파일 SHA는 `outputs/rom_trial/basis8.json`에 있다.

실행 중에는 현재 위치에서 모든 요소의 힘과 국소 Hessian을 계산하고 `(UᵀDU) Δz = Uᵀf`를 푼다. `D`는 VBD 정점별3×3 Hessian의 block diagonal이다. 전체 연성 Hessian의 정확한 Galerkin 투영과 다르다. 현재 전체 상태에 `UΔz`를 더하는 방식이므로 기존 소성 변형 상태를 기저에 강제로 투영하거나 학습 궤적을 시간에 맞춰 재생하지 않는다.

전체 공간 국소 보정의 기저 밖 성분, 유한성, 시험 보정 후 preconditioned residual을 검사한다. 기본 표현 오차 허용치는0.35, 보정 최대 길이는0.1mm다. 접촉 truncation 후 residual 제곱합이 이전 값의1.0001배(+1e-20)를 넘으면 보정을 거부하고 원래 위치에서 전체 VBD를 수행한다. 이 판정은 실제 에너지 감소나 장기 궤적 오차의 수학적 보증이 아니다. 첫/마지막 반복과 매4번째 활성 반복에도 전체 VBD를 실행한다. 승인된 ROM 반복에서는 기존 병진/회전 가속 보정도 건너뛰며, 전체 반복과 fallback에서는 그대로 수행한다.

여기서 적응적인 것은 **ROM 승인과 전체 계산 복귀의 선택**이다. 기저를 온라인으로 확장하거나, 접촉 주변만 전체 공간으로 분리하는 방법은 아직 구현하지 않았다. 모든 요소·자기접촉·소성 이력을 평가하므로 hyper-reduction도 아니다.

## 이론과의 관계

- [Vertex Block Descent, 2024](https://ankachan.github.io/Projects/VertexBlockDescent/index.html): 정점별 Gauss–Seidel 갱신이 기준 계산의 출발점이다. 현재 ROM 보정에 원 논문의 수렴 성질을 그대로 적용하지 않는다.
- [Skipping Steps, 2009](https://www.cs.cornell.edu/projects/skippingSteps/): 온라인 기저 구성과 전체/축소 모델 전환을 제시한다. 본 구현은 고정 POD 기저와 반복 단위 fallback 실험이며 해당 논문 전체 알고리즘의 재현이 아니다.
- [Subspace Condensation, 2015](https://www.tkim.graphics/CONDENSE/): 새 접촉이 요구하는 변형을 전체 공간 영역으로 처리하는 방향이다. 현재의 전역 fallback과 구분한다.
- [Embedded IPC, 2024](https://arxiv.org/html/2409.16385v1): 저차원 탄성과 고해상도 접촉 표면의 분리를 다룬다. 현재 구현은 IPC barrier 모델이 아니며, 논문의 비관통 보장이나 탄성 실험 결과를 골판지 소성에 대한 검증으로 사용하지 않는다.

## 검증 방법

`tests/test_rom.py`는 직교성/병진 표현, rest/topology digest, 독립 float64 선형계 대조, 비유한/기저 밖 보정 거부, residual 판정과 CUDA graph 안의 승인·두 종류 fallback 경로를 검사한다. fallback에서 원래 위치와 충돌 anchor 기준 변위가 복원되는지 확인한다.

각16초 실행은 새 물리를 계산하고30Hz 정점/강체 궤적,60Hz 동작 지표, 최종 힌지별 소성각·누적각·소산·손상 상태를 저장한다. 기존15개 동작 판정과 엄격한 안착 판정은 유지한다. `compare_rom.py`는 월드 좌표 정점 오차를 rigid alignment 없이 비교한다. 추가 비교 기준은 프레임 RMS2mm, 전체 최대 정점 오차10mm, 최종 소성 소산 차이10%, 상승/압착 간격 차이5mm, 관측 속도비1.1 초과다. 이는 공학적 선별 기준이며 실물 보정된 오차 한계가 아니다.

조건 변경 검증은 기저를 재학습하지 않고 상승20→18cm, 압착 목표 간격140→150mm, 가상 모터 상한4000→3600N을 적용한다. 원본 장면은 유지하고 `outputs/rom_trial/varied_scene_v2.usda`의 별도 USD 레이어로 재현한다. `build_rom_variation.py`가 기준 장면의Z-up/미터 단위를 명시적으로 복사하고 물리 메쉬를 대조한다. 최초 레이어는 stage 메타데이터를 누락하여 초기 IK에 실패했으며 `varied_fom.log`에 보존했다. 이는 ROM 실행 이전의 시험 장면 구성 오류다.

다른 프로젝트의 GPU 프로세스가 동시에 관측되었다. 따라서 실행 시간은 공유 장비에서의 관측값이며 확정적인 단독 성능 비교로 해석하지 않는다. 초기 준비 시간은 별도이고, 첫 프레임의 CUDA graph 준비는 실행 시간에 포함된다.

전체 VBD 결과도 수렴된 정답이나 실물 계측값은 아니다. 각 조건에서 한 쌍의 실행을 비교했으며 병렬 누적의 비결정성에 대한 반복 통계는 얻지 않았다. 아래 오차는 기준 실행으로부터의 차이이며, ROM 근사만의 인과 효과를 분리한 수치는 아니다. 현재 후보의 등가성·속도 향상을 입증하지 못했다는 판단에 사용한다.

## 재실행

```bash
./scripts/python.sh scripts/benchmark_rom.py --output outputs/rom_trial/new_fom
./scripts/python.sh scripts/benchmark_rom.py --basis outputs/rom_trial/basis8.npz --output outputs/rom_trial/new_rom
./scripts/python.sh scripts/validate_scaled_robotiq.py --output outputs/rom_trial/new_fom
./scripts/python.sh scripts/validate_scaled_robotiq.py --output outputs/rom_trial/new_rom
./scripts/python.sh scripts/compare_rom.py --baseline outputs/rom_trial/new_fom --candidate outputs/rom_trial/new_rom
```

새 출력 폴더만 생성하도록 기존 폴더가 있으면 실행을 거부한다. `progress.json`은 중단 위치를 확인하기 위한 기록이며 전체 물리 상태 checkpoint가 아니다. 전원 종료 후 bitwise 재시작을 지원한다고 해석하지 않는다.

## 기본 조건의 완료 결과

| 지표 | 기존 전체 VBD | rank8 ROM/VBD |
|---|---:|---:|
|16초 계산 시간, 준비 제외|212.195초|290.008초|
|전체 동작 판정|14/15|14/15|
|실제 상승|176.525mm|167.566mm|
|압착 종료 실제 간격|147.742mm|158.430mm|
|최종 소성 힌지|3,386|3,209|
|최종 최대 순간 속도|46.455mm/s|29.019mm/s|
|14초 이후 옆면 최대 이동|3.014mm|1.095mm|

두 실행 모두 유한 상태, 양쪽 접촉으로 집기/상승, 압착/해제/낙하, 영구 소성과 관절 연결 판정은 통과했다. 두 실행 모두 최종 속도10mm/s 이하와 엄격한 post-14s 이동0.1mm 기준을 통과하지 못했다. 따라서 ROM 이전부터 남아 있던 안착 문제도 해결되지 않았다.

추가 비교12개 중6개만 통과했다. 프레임별 월드 위치 RMS 최대52.629mm, 최종39.445mm, 개별 정점 최대93.118mm다. 강체 정렬을 적용한 **진단용** 형상 RMS도 최대11.911mm/최종8.833mm이므로 차이가 전체 위치 이동에만 있는 것은 아니다. 이 정렬 수치를 원래 월드 좌표 판정 대신 사용하지 않았다. 최종 소성 소산 차이는4.479%이며 힌지별 이력의 유한성·비음수·누적 조건은 통과했다.

ROM 시도352,288회 중213,365회 승인(60.57%), 표현 범위 부족35,450회, residual 판정 거부103,473회다. t10.017–11.017초에는11,520회 시도가 모두 표현 범위 부족으로 거부되었다. fallback이 실제 작동하지만, 앞서 승인한 근사 보정이 만든 궤적 차이를 되돌리지는 않는다.

![기본 조건 비교](../outputs/rom_trial/resumed_rom/comparison.png)

원시 근거: `outputs/rom_trial/resumed_fom/`, `outputs/rom_trial/resumed_rom/rom_comparison.json`, 각 폴더의 `validation.json`, `supported_rest_validation.json`, `shape_diagnostic.json`(ROM), `outputs/rom_trial/regression.log`. 종료 전 학습 데이터/기저 해시도 모두 일치했다(`resumed_integrity.json`).

## 조건 변경 완료 결과

같은 기저로 새 명령을 계산한 추가16초 비교도 모두 끝났다. 기존 결과의 재생이나 기저 재학습을 사용하지 않았다.

| 지표 | 기존 전체 VBD | rank8 ROM/VBD |
|---|---:|---:|
|16초 계산 시간, 준비 제외|235.521초|273.075초|
|전체 동작 판정|14/15|14/15|
|실제 상승|156.222mm|146.696mm|
|압착 종료 실제 간격|161.374mm|165.213mm|
|최종 소성 힌지|3,316|3,046|
|최종 최대 순간 속도|71.108mm/s|25.709mm/s|
|14초 이후 옆면 최대 이동|2.249mm|1.148mm|

추가 비교12개 중7개 통과다. 월드 위치 RMS 최대104.774mm/최종78.042mm, 개별 정점 최대199.432mm였다. 정렬한 진단용 형상 RMS는 최대9.623mm/최종7.793mm였다. 최종 소성 소산 차이는2.489%, 힌지별 소성 이력 조건은 통과했다. ROM 승인 비율은53.45%(188,309/352,288)다. 두 모델 모두 기존 안착 기준과 엄격한0.1mm 정지 기준은 실패했다.

![조건 변경 비교](../outputs/rom_trial/varied_rom_v2/comparison.png)

근거는 `outputs/rom_trial/varied_fom_v2/`, `outputs/rom_trial/varied_rom_v2/`이며 두 조건의 종합 판정은 `outputs/rom_trial/review.json`이다. 네 실행 모두16초까지 완료했으며 종료 전 학습 기저와 기준 primitive 장면은 변경하지 않았다.

## 판정과 후속 방향

현재 후보를 기본 계산기에 승격하지 않는다. 국소 선형계와 fallback 경로의 정상 동작, 소성 이력 보존은 확인했지만, 전체 움직임·형상 유지와 새 계산 가속은 별도 조건이며 충족하지 못했다. 반복마다 전 요소 힘을 평가하고 거부된 경우 전체 VBD도 수행하는 비용이 존재한다. 이 비용이 공유 GPU 경합과 독립적으로 얼마인지는 별도 계측이 필요하다.

다음 구현을 검토한다면 접촉 주변의 전체 공간 처리, 국소/온라인 기저 보강, 보수적인 전환 주기와 시간 구간별 오차 추적이 후보가 된다. 임계치를 느슨하게 해서 승인 횟수만 늘리거나, 기존 결과 재생을 새 계산 가속으로 취급하지 않는다. 이러한 후속 방법의 성능은 이번 실험으로 검증하지 않았다. 기존 전체 VBD의 안착 잔진동도 별도의 미해결 항목이다.
