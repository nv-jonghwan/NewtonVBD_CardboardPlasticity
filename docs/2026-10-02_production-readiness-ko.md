# 프로덕션 준비 — board4-crease45-rc1

2026-10-02, 현재 4 mm 종이박스 시뮬레이션을 재현 가능한 로컬 검토 후보로 정리했다. HMG/EForrest README의 시나리오·아키텍처·물리·설치·운영·검증·한계 구성에 맞춰 루트 README를 전면 재작성했다. 실제 로봇이나 재료의 생산 승인을 의미하지 않는다.

## 변경

- `config/production_candidate.json`: 기존 검증된 S16, 24회 혼합 ROM/VBD, rank 8, 접힘 저항 45, memory 0을 명시한다. 물리 알고리즘과 물성은 이번 정리에서 바꾸지 않았다.
- `assets/models/board4_rank8.npz`: 실험 outputs에 있던 basis를 내용 변경 없이 승격했다. SHA-256 `56a10990b18686d20e3b11e94c72614d053c95e9487eaf1b614199054397c70f`. JSON sidecar에 훈련 출처와 해시를 보존했다.
- `config/release-inputs.lock.json`: 프로파일, basis·출처, 선택된 USD 의존성과 텍스처 등 31개 입력을 고정한다. 공식 원본에는 URL을 기록한다.
- `scripts/python.sh`: 계산/Kit 환경 선택을 독립적으로 유지하고 개인 계정의 다른 프로젝트 Python fallback을 제거했다. 계산 환경 선택이 GUI 래퍼 존재 여부에 의존하던 문제도 제거했다.
- `run_candidate.py`, `run_headless.sh`, `live_crease_friction.sh`: 기본 실행을 실험 출력 폴더에서 분리한다. 라이브 기록은 기본 활성화하며 고유 출력 폴더를 사용한다. Replay는 사용자가 전체 기록을 생성한 뒤 연결한다.
- `check_release.py`: 파일 무결성, 패키지 버전, 활성 USD 참조, 두께·격자·ROM digest 및 실제 CUDA 할당/읽기를 검사한다.
- `fetch_release_assets.py`: 고정된 원본만 다운로드하고 저장 전 크기와 SHA-256을 확인한다. 기존 변조 파일을 덮어쓰지 않으며 offline 검사를 제공한다.
- `package_release.py`: 환경·캐시·실험 outputs를 제외한 검토 ZIP과 파일별 manifest, ZIP SHA-256 sidecar를 생성한다. 생성 장면은 과거 회귀와 연구 재현을 위해 포함한다.
- `docs/media/`, `docs/validation/`: 실험 결과 폴더 밖에 실제 궤적 렌더, 물리 형상 비교, 경로 독립적 요약을 보존한다.

## 검증

94개 회귀 검사와 31개 입력의 offline 검사가 통과했다. 새로운 진입점으로 GPU 1에서 20초를 다시 계산했다. 초기화 약 7.39초, 적분/기록 약 102.31초이며 모든 상태가 유한하고 소성 이력·손상·비음수 소성 일 검사를 통과했다. 하중 중 휴면은 없었다. 12→16초 부피 지표 +4.04%, 외곽 부피 +1.53%, 18.5초에 지지 휴면에 들어갔고 최종 속도는 0이었다. 12→20초 부피 지표는 +4.09%였다.

파지 검사 4/5는 이전 선택 결과와 동일하다. 전체 보유, 양쪽 접촉, 해제 전 테이블 복귀 없음, 양쪽 손끝 관여는 통과했지만 20 mm 재료점 이동 조건은 실패했다. 새 실행이 이전 궤적과 비트 단위로 같은 것은 아니며 공유 GPU 상태와 비선형 접촉의 민감도가 존재한다.

새 GUI 진입점에서도 짧은 Play/Pause/Reset 검사와 새 전체 기록의 Recorded replay Pause/Resume/최종 프레임 검사가 모두 통과했다. GUI 검사는 별도 headless Kit에서 수행했고 실제 표시 품질은 기존 라이브 상태를 Isaac로 렌더해 확인했다.

원본 증거는 `outputs/production_readiness/`에 보존한다. 공개용 요약은 `docs/validation/reference-results.json`이며, 라이브 기록에서 뽑은 12초 상태의 Isaac 렌더를 실제 확인했다. 이전에 열어둔 사용자 GUI는 종료하거나 바꾸지 않았다.

## 경로 독립 배포 검증

검토 ZIP을 `/tmp` 아래 새로운 경로에 압축 해제했다. 내부 manifest의 340개 파일 해시를 전부 비교한 뒤 원래 프로젝트 밖인 `/tmp`에서 실행했다. 공유 설치된 계산 환경만 명시적으로 지정했으며 `.workspace` 연결, 기존 outputs, 기존 Warp cache는 복사하지 않았다. 실제 import된 `cardboard.ROOT`가 압축 해제 경로인지 확인했다.

입력 lock 31개, 활성 USD/ROM/CUDA 점검, 새 캐시에서 0.3초 GPU smoke, 압축 해제된 소스와 자산으로 전체 94개 회귀 검사를 모두 통과했다. 이 시험은 설치된 계산 환경을 재사용하므로 새 OS에서의 패키지 설치 검증과 구분한다. 최종 ZIP에는 이 검증의 문서/요약을 추가하며, 실행 코드와 입력은 시험 묶음과 동일한 해시인지 다시 비교한다.

`README.md`의 36개 로컬 링크와 14개 목차/내부 링크, 140개 Python 파일의 구문과 shell 구문을 확인했다. 기본 실행은 실험 outputs가 없어도 초기화할 수 있고 Replay만 별도 전체 기록을 필요로 한다.

## 배포 전 남은 조건

실물 재료 식별, 엄격한 무슬립 기준, 격자·시간·반복 수 수렴, ROM 훈련 범위 밖 일반화, 목표 GPU의 성능 SLO, 새 OS 설치 및 단일 GPU 전체 회귀, 장기 운전/정확한 checkpoint, 프로젝트 코드 라이선스와 외부 자산 재배포 조건을 확인해야 한다. 현재 `production_qualified=false`를 유지한다.

원본 자산의 미선택 variant/thumbnail/원본 재료 참조는 전체 USD 의존성 탐색에서 경고를 낼 수 있다. 활성 장면의 USD 합성 오류는 없고 활성 파일 자산은 모두 해석된다. `OmniPBR.mdl`은 GUI Kit 런타임이 제공한다. 이 예외와 이유는 lock에 기록했으며, 미해결 활성 USD 참조를 무조건 무시하지 않는다.
