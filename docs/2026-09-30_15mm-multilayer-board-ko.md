# 15mm 등가 다층 골판지와 반폭 압착

기본 GUI는15mm 벽을 가진 상자를 잡아, 실제 손가락 간격을 원래 상자 너비280mm의 약 절반까지 줄이는 시나리오다. 이전5mm 물성을 약하게 만드는 대신 두께에 맞게 강성·면밀도·접촉 두께를 늘리고, 그에 맞는 강한 가상 그리퍼를 사용한다.

## 재료 가정과 이론

실제 골판지의 굽힘 강성은 표면 종이층, 골 형상, 접착 상태에 따라 달라진다. 특히 면내·굽힘 강성은 표면층의 영향을 크게 받고, 횡전단은 코어 및 접착 상태의 영향을 받는다. [Carlsson, Nordstrand & Westerlind (2001)](https://journals.sagepub.com/doi/10.1106/BKJF-N2TF-AQ97-H72R), [NACA TN2289 원문 기록](https://digital.library.unt.edu/ark:/67531/metadc55649/).

이번 구현은 **두께뿐 아니라 종이량도 증가하는 유사한 등가 다층 재료**라는 명시적 가정이다. 같은 표면 종이를 두고 골 높이만 늘리는 경우의 보편 법칙으로 해석하면 안 된다. 실측 물성은 제공되지 않았다.

두께비r=15/5=3에 대해 등가 면 강성A와 면밀도는r배, 굽힘 강성D는r³배로 잡았다. 바깥 섬유의 항복 변형률 ε_y ≈ t κ_y/2를 유지하므로 항복곡률κ_y는1/r배, 초기 항복 굽힘모멘트D κ_y는9배다.

| 항목 | 이전5mm | 새15mm |
|---|---:|---:|
| 두께 |5mm|15mm|
| 면밀도 |0.9kg/m²|2.7kg/m²|
| membraneShear / membraneArea |17640 /29400|52920 /88200|
| 유효 D_MD / D_CD |0.556 /0.278N·m|15 /7.5N·m|
| 항복곡률 |3m⁻¹|1m⁻¹|
| 굽힘 내부 감쇠 시간 |6ms|6ms|
| 소성 손상률 / 잔여 강성 하한 |0.15 /65%|동일|

NVIDIA 원본의 외형과UV는 유지했다. 중립면은 외면에서7.5mm 안쪽에 놓고, particle radius도7.5mm를 사용한다. self-contact 거리는15mm다. 바뀐 중립면에 대해 hinge dual width와 렌더 바인딩을 다시 계산했다. 원본 외형 크기를 유지하므로 중립면 면적이 줄어 총질량은 정확히3배가 아니며, **면밀도가3배**다. 표시 메쉬는 원본 외피이며 내부 골/종이층을 개별 기하로 만들지는 않았다.

## 높은 강성의 실시간 해법

386개 물리 정점,8반복×8substeps,60Hz를 유지한다. 높은 강성에서 정점별 VBD만 적게 반복하면 전체 이동 성분의 수렴이 느려져, 잡아 올리는 힘이 있어도 상자가 충분히 따라오지 못했다. 반복32회만 늘린 시험 및 접촉 두께 축소 시험도 적합한 해결이 아니었다.

`TranslationBlockVBD`는 각 VBD 반복 뒤에 모든 자유 입자의 **공통 병진3자유도**를 추가로 푼다. 내부 탄성·굽힘·소성·자기접촉 에너지는 공통 병진에 불변이므로, 이 부분 문제의 힘과 Hessian에는 관성 및 외부 접촉만 들어간다. Newton1.6의 동일한 접촉 힘/Hessian 함수를 사용한다. 0.8 완화와1mm trust-region 상한을 사용하며, 속도는 Newton의 최종 위치 차분으로 갱신된다. 붙잡기 제약, 형상 애니메이션, 속도 고정은 추가하지 않았다.

단일 연결된 자유 셸에만 허용하고 고정 입자/스프링은 거부한다. 자유비행 운동량·형상 보존과 그리퍼 힘 제한 회귀 시험을 추가했다. 회전 성분의 별도 블록이나 완전한 전역 수렴을 제공하지는 않는다. 이는 **프로젝트의 Newton1.6 확장**이며 upstream 기본 기능으로 표현하지 않는다.

## 그리퍼와 실제 압착량

| 항목 | 15mm 약한 압착 비교 | 최종 반폭 압착 |
|---|---:|---:|
| 집기 힘 상한 |120N/손가락|동일|
| 압착 모터 힘 상한 |600N/손가락|2000N/손가락|
| 손가락 위치 이득 |7200N/m|32000N/m|
| 손가락 속도 감쇠 |85N·s/m|180N·s/m|
| 압착 명령 간격 |90mm|68mm|
| 기록 실행에서 실제 간격 |264.4mm|144.5mm|

명령68mm와 실제 간격은 다르다. 재료 저항과 힘 제한 아래에서 **실제144.5mm(원래 너비의51.6%)**를 확인했다. 센서 위치에서의 개구이며, 접촉하지 않는 모서리까지 상자 전체 폭이 절반이라는 뜻은 아니다. 모터 상한은 접촉 반력의 절대 상한이 아니다. 관성·접촉 및 제한된 솔버 반복 때문에 기록된 반력은 상한을 일시적으로 넘을 수 있다.

이 손가락은 직선 구동 관절이므로 별도 회전 토크를 키우지 않았다. UR 팔은 처방 운동이고 손가락은 동역학이다. **2kN급 힘은 이 데모의 가상 맞춤형 그리퍼 설정이며 실제 UR10이나 특정 상용 그리퍼의 허용하중 검증 결과가 아니다.**

## 관측 결과와 검증 범위

- 약한 압착: 굽힘 후 표시 면의 국소 평탄 면적97.6%, 영구각15.8°. 두꺼운 판처럼 큰 면이 유지되는 기준 비교다.
- 반폭 압착: 12초 실제 개구144.5mm, 압착 직후 소성 이력907개 힌지. 큰 접힘이 생긴다.
- 반폭 압착 후16초 체적87.87% 유지,14→16초 체적변화−0.010%. 면적99.97% 유지.
- 낙하 후15→16초 면별 변형속도 평균0.108mm/s,16초 전체 최대 입자속도2.05mm/s. 큰 구김 후에도 지속적인 체적 붕괴는 관찰되지 않았다. 이는 무진동·완전 정지의 보장은 아니다.
- 반폭 압착에서는 표시 평탄 면적이 압착 시34.4%,낙하 후40.8%로 줄었다. 사용자가 요청한 큰 접힘 때문에 약한 압착의“90% 이상 평탄”기준을 동일하게 적용하지 않는다. 강성/소성 재료 값은 바꾸지 않았다.
- fresh live 별도 실행에서도 실제 개구142.745mm(51.0%),lift100.5mm,최종 최대속도2.40mm/s로12개 동작 검사를 통과했다.
- 실제 GUI의 새 물리 계산:16초 시뮬레이션에16.325초,0.980배속,화면 갱신38.3fps. 모델/IK/앱 준비 시간은 별도다.
-14개 단위 검사와10개 USD 계약 검사 통과. 별도 기록 및 fresh live 시나리오 검증,GUI play/pause/reset/recorded replay 검증은 각 JSON/log에 보관한다.

접촉과 큰 접힘은 제한된 해상도·반복 횟수의 근사다. 고압 구김의 면내 strain p95는약8.5%로, 정량 응력·손상 예측이나 실제 골판지 인증 시험을 대신하지 않는다. 골의 압궤, 층간 박리, 찢김, 습도 의존 물성, 완전 직교이방성은 미구현이다.

## 실행과 재생성

```bash
./scripts/python.sh scripts/live_isaac.py                     #15mm 반폭 압착
./scripts/python.sh scripts/live_isaac.py --quality board15-gentle
./scripts/python.sh scripts/live_isaac.py --quality realtime5
./scripts/python.sh scripts/live_isaac.py --quality reference  #기존5mm 정밀 모델

./scripts/python.sh scripts/build_multilayer_asset.py \
  --variant board15_half --preserve-outer-size --translation-block \
  --grasp-force 120 --crush-force 2000 --crush-gap .068 \
  --grasp-gap .255 --finger-kp 32000 --finger-kd 180
```

`CardboardMaterialAPI`의 두께·강성·소성 필드를 사용한다. 새 `CardboardDemoAPI` 필드는 `translationBlockSolve`(기본false), `fingerPositionStiffness`(기본1200), `fingerVelocityDamping`(기본12)이며 이전 자산 기본 동작을 보존한다.

- 최종 자산: `assets/cardboard_board15_half.usda`, `assets/demo_scene_board15_half.usda`
- 결과: `outputs/board15_half/report.json`, `live_sequence_validation.json`, `live_performance.json`
- 화면: `outputs/board15_half/gui_crush.png`, `gui_settled.png`
- 이전 실시간 소스 복원: `deliverables/realtime-r2-source-backup.zip` 및 r2 runtime binding. 소스가 공유 프로젝트에 있으므로 runtime symlink만 바꾸는 것으로 소스까지 복원되지는 않는다.
