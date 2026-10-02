# VBD 접촉 스케줄링 최적화 구현·검증

프로파일에 근거한 접촉 스케줄링 최적화를 실제 구현하고 primitive GUI의 기본 경로로 연결했다. 기본 조건 3회씩의 전체 시간 중앙값은 **201.01→191.56초(4.7% 감소)**, 변경 명령 조건은 **200.68→190.65초(5.0% 감소)**였다. 52개 회귀 검사와 GUI Play/Pause/Reset 검사를 통과했다. 기존 안착 및 엄격한 전체 궤적 재현성 문제는 남아 있다. 결과 수치는 `outputs/vbd_optimized/`의 새 실행 자료를 기준으로 한다.

## 구현한 변경

`src/cardboard/rigid_schedule.py`와 `scheduled_contact.py`에 Newton 1.6용 프로젝트 확장을 추가했다. 공유 Newton 설치는 수정하지 않는다. 원본 `baseline` 경로는 상위 solver를 그대로 호출하며 비교와 복귀에 사용할 수 있다.

GUI 검토 대상인 `guarded` 경로는 다음 세 가지를 바꾼다.

1. 강체 force, torque, Hessian 세 블록의 초기화 5회를 하나의 커널로 합친다.
2. body별 접촉 수가 0이거나 담당할 접촉이 없는 스레드는 기하 계산과 0 atomic-add 전에 반환한다. 이미 쌓인 다른 force/Hessian 항을 지우지 않는다.
3. 색 그룹별로 세 번 호출하던 body-particle 접촉 누적을 모든 dynamic body에 대해 한 번 호출한다. **강체당 4스레드, 각 스레드의 접촉 순서와 합산 방식은 유지**한다.

3번이 가능한 이유는 rigid iteration 안에서 particle 위치가 고정되고, 각 body의 pose는 그 body가 속한 색 그룹의 solve 전까지 바뀌지 않기 때문이다. body-particle 접촉은 해당 body와 particle에만 의존한다. 따라서 해당 힘을 색 그룹 반복 전에 계산할 수 있다. 다른 body의 갱신에 의존하는 **body-body 접촉, joint solve, 색 그룹별 강체 pose 갱신, dual update는 원래 순서**로 실행한다. body 색 그룹이 서로 겹치면 초기화 시 거부한다.

이 변경은 강체 iteration마다 초기화 4회와 body-particle 호출 2회를 줄인다. active의 768 rigid iterations/frame에서는 4,608회, supported의 1,536회에서는 9,216회의 해당 GPU 작업 호출 감소다. 이 계산은 제거한 명시적 커널/초기화 호출 수이며 전체 graph node 수를 계측한 것은 아니다.

## 적용하지 않은 후보

32스레드 적응형 후보도 실제 16초 실행했다. active 프레임 중앙값은 첫 원본 158.5 ms에서 140.3 ms로 줄었으나, 전체 시간은 원본 123.34초에 비해 189.48초였다. 원본은 이 실행에서 일찍 sleep에 들어갔고 후보는 들어가지 못했다. 원본 대비 최대 프레임 위치 RMS 19.17 mm로 이전 2 mm 비교 기준도 넘었다. 이 결과만으로 전체 가속 또는 궤적 동등성을 주장할 수 없어서 **GUI 선택지에서 제외**했다. 실험 코드와 결과는 보존한다.

supported substeps/iterations 감축, sleep 강제, 정지 기준 완화, 감쇠·물성 변경, 접촉 생략은 적용하지 않았다. 기존 mesh·collision·마찰·소성 이력·시간 간격·반복 예산을 유지하는 계산 스케줄 변경에 범위를 한정했다. 자기접촉 커널과 rigid solve 자체를 더 크게 재작성하는 후보도 이번 적용에 포함되지 않는다.

## 전체 시간 해석 시 주의점

새 원본 실행 자체가 반복 간에 다른 안착 결과를 냈다. 첫 baseline은 123.34초/안착 성공이었으나 두 번째 baseline은 안착 실패였다. 원본끼리의 최대 프레임 위치 RMS도 52.40 mm였다. 이는 최적화 없이도 이 비선형 접촉 장면의 반복 재현성이 충분하지 않다는 증거다.

따라서 총 실행 시간만으로 커널 가속을 판단하지 않는다. 원본 및 guarded 반복의 총 시간과 함께, active/supported/sleep 모드별 시간과 집기·상승·압착 시간을 비교한다. 이전 ROM 비교의 엄격한 world-space 위치 기준은 그대로 보고하되, 원본 반복도 실패하는 경우 그 한계를 명시한다. 동일 상태에서 한 번의 강체 반복을 비교하는 검사와 장면의 실제 물리 검사도 별도로 수행한다.

## 검증 구성

- 기본 조건의 원본과 guarded 성능 비교를 GPU1에서 순차 실행한다. GPU0에서는 동일 상태 출력 검사와 변경 명령 조건의 원본/guarded 순차 비교를 수행한다. 두 GPU의 시간을 하나의 A/B 비율로 섞지 않는다. 새 상태에서 16초를 끝까지 실행하고 궤적·소성 이력·로그·소스 hash·GPU 정보를 보존한다.
- 기본 장면 반복에 더해 상승 목표 20→18 cm, 압착 목표 간격 140→150 mm, 압착 force 명령 4,000→3,600 N인 기존 변경 명령 장면에서도 원본/후보를 비교한다. 실제 접촉력의 상한을 뜻하는 값은 아니다.
- 단위 검사에서 particle/edge/face 접촉, 접촉 수 0/1/3/31/32/65/244/256/300, buffer clamp, 미리 누적된 출력 보존, CUDA graph 재사용 중 contact count 변경을 검사한다. 강체 초기화 커널의 모든 성분도 검사한다.
- 전체 rigid iteration 비교는 같은 loaded 상태에서 force/torque/Hessian, pose, contact penalty/dual 및 joint dual 출력을 비교한다. 실험 사이의 상태는 복원한다.
- 기존 전체 회귀 검사도 `CARDBOARD_VBD_SCHEDULE=guarded`로 실행한다. 전체 시나리오의 finite state, 양손 접촉, lift, joint closure, 소성 이력, 안착 기준을 변경하지 않는다.

## 실행과 복귀

`robotiq-primitive` GUI의 기본 경로를 guarded로 연결했다. 다른 quality 모드의 기본 경로는 유지한다. 아래처럼 명시적으로 선택할 수도 있다. worker snapshot의 `vbd_schedule`로 실제 적용 여부를 확인할 수 있다. 지원하지 않는 solver에서 guarded가 요청되면 조용히 원본으로 실행하지 않고 오류를 낸다.

```bash
./scripts/python.sh scripts/live_isaac.py --quality robotiq-primitive --vbd-schedule guarded
```

원본 비교 GUI:

```bash
./scripts/python.sh scripts/live_isaac.py --quality robotiq-primitive --vbd-schedule baseline
```

전체 새 실행 및 검증 예시(output은 새로운 경로 사용):

```bash
./scripts/python.sh scripts/benchmark_vbd_optimized.py --schedule guarded --output outputs/vbd_optimized/manual_guarded
./scripts/python.sh scripts/validate_scaled_robotiq.py --output outputs/vbd_optimized/manual_guarded
CARDBOARD_VBD_SCHEDULE=guarded ./scripts/python.sh -m unittest discover -s tests -v
./scripts/python.sh scripts/check_rigid_iteration.py --output outputs/vbd_optimized/manual_rigid_iteration
```

GPU0의 압착 11초 상태에서 전체 강체 반복을 비교한 결과, 위 14개 출력 배열의 최대 절대 차이는 모두 0이었다. 이는 한 번의 동일 상태 비교 결과이며 전체 궤적의 bitwise 재현성을 뜻하지 않는다. 상세 결과: [동일 상태 출력 검사](../outputs/vbd_optimized/rigid_iteration_gpu0/result.json).

## 기본 조건 3회씩의 측정 결과

| 경로 | 전체 시간 3회, 초 | 중앙값 |
|---|---|---:|
| 원본 baseline B/C/D | 201.007 / 201.115 / 200.681 | 201.007초 |
| guarded A/B/C | 190.950 / 191.955 / 191.562 | 191.562초 |

전체 시간 중앙값은 **4.70% 감소, 약 1.049배 가속**이다. 초기 baseline A의 123.34초는 별도로 보존하며 이 3회 반복 표에는 포함하지 않았다. A는 원본 자체가 조기에 sleep에 들어간 실행이다. 따라서 모든 실행에서 총 시간이 반드시 줄어든다는 뜻이 아니다. GPU clock을 고정하지 않았고, 이 수치를 다른 장면이나 장치의 보장값으로 사용하지 않는다.

각 실행의 단계별 중앙값을 구한 뒤, 그 세 값의 중앙값을 비교했다.

| 단계 | 원본 ms/frame | guarded ms/frame | 시간 감소 |
|---|---:|---:|---:|
| 집기 | 143.96 | 134.09 | 약 6.9% |
| 상승 | 159.91 | 147.13 | 약 8.0% |
| 압착 | 177.88 | 164.54 | 약 7.5% |
| 안착 | 476.46 | 464.27 | 약 2.6% |

원본 B/C/D와 guarded A/B/C 모두 전체 시나리오 **14/15**를 통과했고, 미통과 항목은 기존 `settled`였다. guarded의 마지막 속도도 기존 10 mm/s 기준을 넘었으므로 안착 문제가 해결됐다고 기록하지 않는다.

원본 B 대비 C/D의 최대 프레임 위치 RMS는 34.73/70.36 mm였고, guarded A/B/C는 39.56/30.11/32.45 mm였다. 후보의 차이가 원본 반복에서 관찰한 범위 안에 있다는 점과, 엄격한 2 mm 궤적 기준을 만족하지 않는다는 점을 함께 기록한다. 이 소수 표본으로 통계적 물리 동등성을 입증한 것은 아니다. 후보의 소성 work 차이는 원본 B 대비 1.16–3.23%였고, 소성 이력 및 기존 lift/gap 검사에 새 실패는 없었다.

`CARDBOARD_VBD_SCHEDULE=guarded`로 **52개 회귀 검사 전체를 통과**했다. 원본의 강체·관절 색 순서 및 dual update tail은 AST 비교로도 보존을 확인했다. [회귀 로그](../outputs/vbd_optimized/regression_guarded.log), [소스 검토](../outputs/vbd_optimized/source_review.json).

## 변경 명령과 GUI 검증

변경 명령 조건은 GPU0에서 원본/guarded를 순차 실행했다. 전체 시간은 **200.680→190.646초, 5.00% 감소**했다. 양쪽 모두 14/15이며 안착만 미통과했다. 후보의 소성 work 차이는 0.50%, lift/gap 차이는 기존 5 mm 기준 이내이고 소성 이력도 유효했다. 최대 프레임 위치 RMS는 34.21 mm여서 엄격한 2 mm 전체 궤적 기준은 통과하지 않았다. [변경 조건 비교](../outputs/vbd_optimized/varied_guarded/comparison_varied_baseline.json).

기본 primitive 실행 명령에 별도의 최적화 flag를 주지 않은 실제 Isaac headless smoke에서 Play → Pause 유지 → Reset → Play를 검사했다. 0.25초 완료, finite 상태, 실제 worker의 `vbd_schedule=guarded`, 정상 종료를 확인했다. 이는 인터페이스 동작 검사이며 0.25초 결과를 전체 성능 수치로 사용하지 않았다. 이후 같은 기본 명령으로 데스크톱 GUI를 다시 열었다. [GUI 동작 검사](../outputs/vbd_optimized/gui_smoke/result.json).

![반복 실행 시간과 단계별 프레임 시간](../outputs/vbd_optimized/performance.png)

## 산출물 및 적용 범위

- [전체 집계](../outputs/vbd_optimized/summary.json), [검토 결과](../outputs/vbd_optimized/review.json), [GUI 실행 확인](../outputs/vbd_optimized/gui_verification.json)
- [강체 스케줄링](../src/cardboard/rigid_schedule.py), [접촉 커널](../src/cardboard/scheduled_contact.py), [경계조건 테스트](../tests/test_rigid_schedule.py)
- [전체 실행 도구](../scripts/benchmark_vbd_optimized.py), [궤적 비교](../scripts/compare_vbd_optimized.py), [동일 상태 강체 반복 검사](../scripts/check_rigid_iteration.py), [통계·그림 생성](../scripts/summarize_vbd_optimized.py)

검증한 계산 스케줄링을 primitive GUI에 적용했다. 모든 프로파일 후보를 적용한 것은 아니다. 32스레드 후보는 제외했고, supported 반복 예산과 자기접촉 알고리즘의 추가 변경은 정확도 검증이 더 필요하다. 현재 변경을 전체 장면의 물리 품질 통과나 실시간 성능 달성으로 해석하지 않는다.
