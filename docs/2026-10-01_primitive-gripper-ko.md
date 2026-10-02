# 그리퍼 충돌 단순화와 20cm 상승

사용자 요청에 따라 현재10mm/same-resolution 장면에서 그리퍼 충돌 형상과 상승 목표만 변경했다. 최종 상승 요청은20cm이며 중간10cm 요청을 대체한다.

- 양쪽 각각 Fingertip(접촉 패드), Finger4(패드 지지부), Finger2(바깥 손가락 링크), Finger3(안쪽 연결 링크)에 Cube primitive 하나씩: 총8개.
- 기존11개 메쉬 collider와2개 원통 collider는 모두 collisionEnabled=false로 하고 CollisionAPI/MeshCollisionAPI도 제거했다. 손바닥, 장착 어댑터, 뿌리 쪽 outer_knuckle에는 대체 collider를 만들지 않았다.
- 패드는 원본 메쉬 좌표축 기준 경계 상자로 평평한 접촉면 위치를 유지한다. 나머지 링크는 원본XY 평면의 최소 면적 경계 직사각형을Z 두께로 확장한다. 링크의 모서리·굴곡·패드 뒷면 테이퍼는 근사하므로 접촉 결과가 원본과 완전히 같지는 않다.
- 테이블 상판은 기존1.8m×1m×0.08m Cube를 유지하고 다리4개의 충돌 API를 제거했다. 바닥 collider는 유지한다.
- 비주얼 원본은 그대로이며 primitive는 guide/invisible이다. Newton에 실제로 들어간 collision flags와 BOX 타입을 검사했다. 초기 버전은 꺼진 메쉬도 모델 배열에 남았으나, 최종 버전에서는 API까지 제거해 물리 전용 importer가 이를 아예 생략한다.
- 질량·관성·무게중심·관절·모터·마찰값은 유지한다. 패드의 상위 prim에서 상속된 물리 재질까지 해석해 새 primitive에 연결한다.
- 박스3,458정점/6,912삼각형, 표시173,760삼각형,10mm 재료와VBD 반복 설정은 그대로다. 상승 목표는12→20cm. 이는 그리퍼 목표 이동량이며 실제 박스 상승량은 미끄러짐과 변형에 따라 다르다.

## 재현

```bash
./scripts/python.sh scripts/build_primitive_gripper.py
./scripts/python.sh scripts/validate_primitive_gripper.py
./scripts/python.sh scripts/live_isaac.py --quality robotiq-primitive
```

장면은 `assets/demo_scene_robotiq_board10_primitive.usda`, 기록 재생은 `outputs/primitive_gripper/final`다. 원래 support20 장면·기록은 보존한다. 기존 MuJoCo 실험을 적용하지 않았으며 원래 Newton1.6 VBD 경로다.

## 검증 및 비교 방법

자산 검증21개와 기존 회귀43개를 통과했다. 형상·재료·관절 속성과 관계는 요청한 collisionEnabled/liftHeight를 제외하고 원본과 같다. 실제 Newton import의 질량·관성 및 패드 마찰도 동일하다. 활성 그리퍼 충돌체는13→8개이고 모두BOX다.

두 장면 모두20cm 상승, 동일 benchmark harness/16초 동작/GPU1로 순차 측정한다. 비교 메쉬 장면은 `assets/demo_scene_robotiq_board10_lift20_baseline.usda`다. GPU 초기화는 측정에서 제외하되 첫 frame의 graph 준비는 포함한다. 한 번씩의 비교이며 속도 재현성이나 실물 정확도 검증을 뜻하지 않는다.

`boxes_lift20` 초기 진단은 패드의 상속된 마찰 재질을 누락했기 때문에 비교에서 제외했다. 마찰을 수정했지만 다리 충돌이 남아 있던 중간 결과는 `boxes_lift20_v2`다. 최종 상판-only 결과는 `final`이며 앞선 진단들은 비교에서 제외한다.

![충돌체와 원본 비교](../outputs/primitive_gripper/collider_overlay.png)

원본 외형은 회색, 유지한 충돌체는 색 선이다. 왼쪽 손가락의 초기 자세를XY 평면에 투영했다.

## 최종 결과

- 자산21/21 및 회귀43개 통과. 전체 동작14/15, 엄격 정지3/8, 기존 주름 비교9/13 통과. 실패 항목은 그대로 기록했다.
- 상승 목표20cm, 박스 최저점 기준 실제 상승17.67cm. 양측 파지·압착·영구 주름·해제·착지는 통과하고 관절 최대 오차0.987mm다.
- 최종 최대 입자 속도29.69mm/s,14초 이후 옆면 최대 위치 변화8.53mm로 정지 기준 미충족이다. 완전 안정화된 개선판으로 표시하지 않는다.
- 관측 전체 계산 시간은 기존 메쉬20cm 조건174.74초, 최종 primitive299.39초다. 단, 다른 프로젝트의 시뮬레이터가 같은GPU1을 동시에 사용했고 측정 도중 프로세스도 바뀌었다. 따라서 이 수치로 충돌체 변경만의 가속/감속 비율을 주장하지 않는다. **전체 속도 개선은 입증하지 못했다.** 비활성 충돌 메쉬 제거는 물리 모델의 형상 개수를33→24개로 줄였지만 안착까지의VBD 반복과GPU경합이 남아 있다.
- 요청한 충돌 구성·상승 변경은 별도 GUI 모드에 적용했다. 원본 및 기준 기록은 보존하고 기존 정지/주름 판정 기준은 바꾸지 않았다.

상세 판정은 `outputs/primitive_gripper/review.json`, `final/validation.json`, `final/supported_rest_validation.json`, `final/crease_comparison.json`에서 확인한다.

GUI의0.3초 새 물리 Play/Pause/Reset과16초 전체 기록 재생의 일시정지·재개·마지막 프레임을 검증했다. 압착/착지 화면도 확인했다. 최종 GUI PID323749는 `robotiq-primitive` 모드, Home/t0/paused의 유한3,458정점/17강체로 열었다.
