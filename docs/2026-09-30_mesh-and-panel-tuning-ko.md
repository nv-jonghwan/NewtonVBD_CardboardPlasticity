# 고해상도 골판지 면과 주름 튜닝

현재 기본 GUI는 `assets/demo_scene_dense.usda`를 열며, 상자는 `assets/cardboard_dense.usda`를 참조한다. 이전 자산과 기록은 그대로 보존했다.

## 발견한 원인과 변경

기존 물리 메쉬는 면당 12×12, 866 vertices / 1,728 triangles였다. 표시용 26,592 vertices는 삼각형별로 중복된 정점이며, 넓은 면에는 최대 84.64 mm 길이의 삼각형이 남아 있었다. 전체 정점 수만으로 표시 해상도가 충분하다고 판단할 수 없었다.

물리 메쉬를 면당 24×24, **3,458 vertices / 6,912 triangles / 10,368 hinges**로 늘렸다. 표시용 메쉬는 원본 삼각형의 긴 변을 양쪽에서 일관되게 나누는 방식으로 세분화했다. **57,280 triangles / 171,840 face-corner vertices**, 최대 변 길이 약 **11.79 mm**이다. 원본 UV와 텍스처를 보존하고, 새로운 물리 메쉬에 다시 바인딩했다. 닫힌 테스트 메쉬의 체적·UV·공유 변 연결 보존 검사를 통과했다.

## 물성 및 실행 설정

| 항목 | 이전 +5% 강성 버전 | 현재 고해상도 버전 |
|---|---:|---:|
| 면내 shear / area 계수 | 12,600 / 21,000 | 17,640 / 29,400 |
| 굽힘 MD / CD 계수 | 0.042 / 0.02016 | 동일 |
| 항복곡률 | 12 | 8.4 |
| 경화 비율 | 0.02 | 0.01 |
| 손상 누적 계수 | 0.35 | 0.5 |
| 면내 / 굽힘 감쇠 | 0.0002 / 0.0002 | 0.02 / 0.0006 |
| 압착 모터 제한 | 70 N | 70 N |
| 압착 목표 개구 | 110 mm | 190 mm |
| VBD iterations / substeps | 16 / 16 | 64 / 16 |

면의 늘어남에는 더 저항하고, 접히는 곳에서는 항복과 손상이 더 쉽게 진행되도록 분리했다. 메쉬가 바뀌면 동일한 명령으로 생기는 구김도 달라진다. 110 mm 목표를 그대로 쓰면 지나치게 구겨지고 압착 중 미끄러져 내려가는 문제가 있어 고해상도 장면에서는 190 mm로 제한했다. 실제 종료 간격은 약 233.4 mm이다. 이전의 약 196.1 mm와 전체 구김 양이 정확히 같지는 않다.

기존 16 iterations를 고해상도에 그대로 적용하면 시작 시 붕괴했다. 반복 횟수를 64로 높이고 `/World/Physics`의 등록 스키마 값 `cardboard:gravityRampSeconds=0.75`를 사용했다. 첫 0.75초 초기화 동안 중력 하중을 부드럽게 증가시킨다. 이후 표준 중력이며, 접근 전까지 소성 이력 0을 확인했다. 이 초기화 구간은 실제 환경의 중력 변화로 해석하거나 학습용 실제 동작으로 취급하지 않는다.

주름의 영구 기준각과 누적 손상은 계속 보존된다. 한번 접힌 곳의 굽힘 강성이 감소하지만, 현재 항복 모멘트에는 양의 경화항도 있으므로 재항복 힘이 반드시 낮아진다고 보장하는 모델은 아니다. 이번에 그 법칙 자체를 변경하지는 않았다.

## 검증과 한계

- 재료/메쉬 테스트 4개, USD 계약 검사 10개, 전체 집기·들기·압착·낙하 검사 12개 통과.
- GUI Play/Pause/Reset/재시작 및 기록 재생 Pause/Resume/마지막 프레임 검사 통과. `gui_crush.png`는 실제 RTX 뷰포트 캡처이다.
- 압착 유지 구간 11.2–12초에서 강체 이동·회전을 제거한 내부 속도 RMS는 38.9→4.28 mm/s, 면내 변형률 RMS는 1.088→0.417%였다. 이 속도에는 계속 진행되는 소성변형도 포함되며 순수한 진동 지표는 아니다.
- 들기 후 바닥과의 실제 간격 약 67.5 mm. 압착 중 영구 주름이 증가하고, 낙하 후 최종 양손 접촉력은 0 N이다.
- 해상도에 따라 힌지 수가 달라지므로 원시 소성 힌지 개수를 직접 비교해 개선 비율로 해석하면 안 된다.
- 낙하 후 15–16초 내부 속도는 이전보다 높았고, 최종 최대 입자 속도도 0.157 m/s로 정적 평형은 아니다. 충돌 시점이 다르므로 같은 절대 시간 구간 비교에는 한계가 있다. 압착 중 면 반응 개선과 낙하 후 거동은 구분해야 한다.
- 실물 골판지 물성 동정, 완전 직교이방성, 상세 flute, 파괴/박리 및 메쉬·시간 간격 수렴 검증은 완료하지 않았다. 더 높은 반복 횟수는 초기 안정성을 개선했지만 정량 수렴 인증이 아니다.

모든 수치는 `outputs/panel_dense_final/{state.csv,panel_metrics.json,report.json,comparison.json,run_config.json}`에 있다. 비교 기준은 같은 측정기를 사용한 `outputs/panel_baseline`이다. 채택하지 않은 초기 후보들은 기본 자산에 적용하지 않았다.

## 실행과 재현

```bash
./scripts/python.sh scripts/live_isaac.py
# 빠른 확인: Recorded replay → Box detail
# 새 계산: Reset to Home → Play / Resume

./scripts/python.sh scripts/build_dense_asset.py
./scripts/python.sh scripts/measure_panel_response.py --profile panel_crease \
  --scene assets/demo_scene_dense.usda --output outputs/panel_dense_final
./scripts/python.sh scripts/validate_scenario.py --output outputs/panel_dense_final \
  --baseline outputs/panel_baseline --comparison-mode report
./scripts/python.sh scripts/validate_assets.py --asset assets/cardboard_dense.usda \
  --scene assets/demo_scene_dense.usda --report outputs/dense_asset_validation.json
```

기록 재생은 실제 Newton 계산 결과를 시간에 맞춰 표시한다. 새 물리 계산과 구분해 패널에 `Recorded`로 표시한다. 현재 장비에서 16초 물리 계산은 약 404초였으며 실시간 성능을 주장하지 않는다. 이전 버전 GUI는 `--scene assets/demo_scene.usda --replay-directory outputs/panel_baseline`으로 열 수 있다.
