# 구현 및 검증 결과

실제 실행 결과: 2026-09-30. 플랫폼: RTX 6000 Ada, standalone Newton 1.5.0 / Warp 1.16.0. 모든 아래 수치는 코드 실행 산출물에서 가져왔다.

## 최종 실험

동일한 원본 자산, 그리퍼, 11초 제어 시퀀스에 대해 소성 on/off를 비교했다. 60 Hz output, frame당 16 substeps, 기본 VBD 16 iterations다. 팔은 dynamic rigid bodies와 implicit joint PD drive, 손가락은 30 N으로 제한한 explicit motor force를 사용한다. 손가락과 상자의 접촉에 의해서만 변형을 유도한다. 상자 정점을 손가락에 강제로 붙이거나 mesh animation으로 찌그러뜨리지 않는다.

| 측정 | 소성 on | 탄성 대조 |
|---|---:|---:|
| 집기 전 1초 소성 힌지 수 | 0 | 0 |
| 최종 소성 힌지 수 | 129 | 0 |
| 누적 소성 유동 소산 | 0.19879 J | 0 J |
| 최종 형상 RMS / 초기형상, rigid alignment | 4.469 mm | 2.397 mm |
| 최대 손가락 접촉 반력 | 34.601 N | 35.098 N |
| 최종 양쪽 손가락 접촉 반력 | 0 N | 0 N |

두 모델의 최종 형상을 서로 rigid alignment한 차이는 **RMS 3.706 mm / 최대 11.324 mm**다. 초기형상 대비 RMS에는 중력과 테이블 접촉의 탄성 변형도 포함되므로, 그 수치 전체를 소성량이라고 해석하면 안 된다. 소성 모델 최종 최대 입자 속도는 **0.630 mm/s**, 저장된 최대 영구 힌지 각은 **0.558 rad**다.

실제 twist 명령은 torque feedback 때문에 약 **0.0321 rad (1.84°)**까지만 진행됐다. 설정된 0.12 rad를 모두 강제하지 않았다. 2 N m는 twist-axis supervisor threshold이며, 관측된 twist-axis 최대치는 **3.451 N m**, 전체 모멘트 norm 최대치는 **4.225 N m**다. 60 Hz 피드백의 overshoot가 있으므로 엄밀한 torque bound라고 부르지 않는다. 실로봇 safety controller 검증은 이 작업 범위에 포함되지 않는다.

최저 정점 z는 0.651158 m, 테이블 위면 z는 0.650 m, contact radius는 0.0015 m다. 이 범위에서 유한한 penalty penetration이 있고, 침투가 수학적으로 완전히 0이라고 주장하지 않는다. 별도의 전 프레임 triangle–triangle intersection proof는 실행하지 않았다.

## 검사 결과

- **구성식 3개 테스트 통과**: 항복 이하에서 소성 갱신 없음; yield return mapping과 양의 소산; 독립 scalar reference와 Warp GPU 결과 일치.
- **USD 계약 10개 검사 통과**: registered codeless API 5개, meter/Z-up, 닫힌 two-manifold, 비퇴화 삼각형, 바인딩의 인덱스/가중치/원형 복원, 실제 fixed/prismatic 연결, unscaled rigid body roots, 원본 SHA-256.
- **물리 시퀀스 8개 검사 통과**: 유한 상태, 초기 소성 0, 영구 이력 존재, 탄성 이력 0, 제하 후 무접촉, 소산 단조 증가, 최종 소성 모델 최대 속도 <1 mm/s, 테이블 접촉 허용오차.
- **Checkpoint 복원 통과**: 2,592개 힌지의 상태와 rest-angle 및 양쪽 solver state buffer 복원. 최종 render mesh도 물리 형상과 일치하게 저장.
- **Isaac Sim RTX**: 실제 USD time samples를 331개 프레임으로 렌더하고 MP4 생성. 렌더 도중 PhysX rigid-body actuation은 비활성화.
- **Live bridge smoke 통과**: 별도 Newton process → atomic NPZ → Isaac Sim USD 업데이트. 0.2 simulation seconds / 12 snapshots 수신. 전체 GUI 조작 UX/장시간 실행까지 검증한 것은 아니다.

`outputs/asset_validation.json`, `outputs/checkpoint_validation.json`, `outputs/validation_summary.json`, `outputs/unit-tests.log`, `outputs/live_bridge_test.log`가 근거다. NVIDIA 전체 SimReady validator suite를 통과했다는 의미는 아니다.

## 수치 민감도와 미완료 검증

| 지표 | 16 iterations | 32 iterations | 상대 변화 |
|---|---:|---:|---:|
| 잔류 형상 RMS | 4.469 mm | 3.225 mm | 27.84% |
| 소성 유동 소산 | 0.19879 J | 0.16397 J | 17.52% |
| 최대 손가락 접촉 반력 | 34.601 N | 33.885 N | 2.07% |
| 최종 소성 힌지 수 | 129 | 105 | 메시의 임계 항복 판정에 민감 |

따라서 **정성적인 permanent-crease 데모는 통과, 정량 수렴은 미달**로 기록했다. 32 iterations를 정답으로 간주하지도 않는다. feedback controller의 실제 명령 경로도 반력에 따라 달라지므로 이 비교는 solver-only 오차 분리 시험이 아니라 결합 시스템의 iteration sensitivity 시험이다. 향후 고정된 동일 경계조건의 coupon 실험에서 timestep/iteration/mesh convergence를 분리한 뒤 재료를 동정해야 한다.

현재 simulation mesh의 삼각형 변형률/좌굴 거동은 기존 Newton membrane, 소성은 굽힘 힌지에만 적용된다. 완전한 골판지의 직교이방성·면내 압축 소성·flute 두께 압궤·테이프·접착 파손은 구현하지 않았다. parameter fitting과 실제 박스 force-displacement 검증 역시 실측자료가 없어 수행하지 않았다.

## 재현 및 성능

최종 소성 런: physics 11초 / 약 52.1초 wall time. 다른 GPU 작업과 동시 실행한 탄성/32-iteration 런은 각각 약 81.1초/122.3초였다. 이를 전용 장비의 공정한 성능 비교로 해석하지 않는다. frame integration은 CUDA graph로 실행하지만 IK, USD 쓰기, diagnostics, viewer는 host 작업을 포함한다.

모든 실험의 초기 상태/재료/로봇 궤적은 기본 USD와 코드로 재생성 가능하며, 원본 바이너리 자산의 URL과 hash를 보존했다. 이전 tuning 실험은 `outputs/trial_*`와 `outputs/settle*`에 남아 있고, 최종 결과의 근거는 `outputs/plastic`, `outputs/elastic`, `outputs/convergence32`만 사용한다.
