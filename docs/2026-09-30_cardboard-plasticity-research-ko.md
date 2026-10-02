# Newton 기반 골판지 상자의 소성변형: 조사, 모델 선택, 구현

작성/자료 확인: 2026-09-30. 실행 기준: Newton 1.5.0, Warp 1.16.0, OpenUSD 25.11. 최신 upstream 문서와 설치 버전은 구분한다.

## 결론

가능하다. 골판지는 “젤리와 천의 중간 탄성계수” 하나로 설명하기보다 **높은 면내 강성 + 상대적으로 낮은 굽힘 강성 + 좌굴 + 항복 이후의 영구 곡률 + 손상**으로 표현하는 것이 적절하다. 임계 하중을 넘으면 판 전체를 cloth로 바꾸는 것이 아니라, 일부 위치의 굽힘 기준 상태가 변하고 강성이 떨어지므로 주름이 남는다. 항복과 구조 좌굴은 다른 현상이다. 탄성 판도 좌굴할 수 있고, 하중을 제거하면 복원될 수 있다.

실시간 로봇 조작/SDG를 우선하는 이번 작업에서는 **Newton VBD 삼각형 셸 + GPU 소성 힌지 return mapping + USD 상태 저장**을 채택했다. 정량적인 골판지 파손 예측의 최종 목표는 실험으로 식별한 직교이방성 셸 구성식, 면내 압축 소성, 두께방향 코어 압궤, 접힘선/테이프/접착부 모델까지 포함하는 것이다. 현재 코드는 그 전체 모델의 검증 완료판이 아니라 굽힘 소성에 초점을 둔 실행 가능한 연구 데모다.

## 조사에서 확인한 사실

| 접근 | 실제 장점 | 이번 문제의 제약 | 판단 |
|---|---|---|---|
| Isaac Sim/PhysX volume deformable | 기존 Kit/센서와 통합하기 쉬움 | 얇은 중공 박스를 꽉 찬 tetrahedral 젤리로 모델링하면 물리가 달라짐. 문서상의 표면 탄성/감쇠 설정만으로 종이의 영구 주름이 구현되는 것은 아님 | 표준 deformable을 붙이는 것만으로 해결하지 않음 |
| PhysX surface deformable | 표면과 굽힘을 직접 표현 | 확인한 문서는 런타임 rest shape 변경을 지원하지 않는다고 명시. 스키마의 표현 범위와 구현 범위도 다름 | 이 데모의 rest-angle 소성 업데이트 경로로 선택하지 않음 |
| Newton VBD 셸 | 강성 셸과 강체 접촉, 관절을 GPU에서 함께 계산. edge rest angle과 강성 접근 가능 | 기본 굽힘은 탄성. 소성 구성식 추가 필요. 고강성/고해상도에서는 반복 수와 수렴 검증 필요 | 이번 구현 |
| Newton implicit MPM | 압력 의존 항복, 소성, hardening/softening, 큰 변형 | 일반 volumetric MPM의 plasticity가 곧 얇은 골판지 셸 모델은 아님. 얇은 층 두께를 격자로 분해하는 비용과 별도 shell formulation 문제 | 두꺼운 압궤재/충전재 또는 별도 shell-MPM 연구에 적합 |
| ARCSim 방식 | 종이의 굽힘 소성, weakening, 주름 정렬 remeshing 선행 연구 | Newton/로봇 GPU 생태계에 직접 들어맞지 않음. 배포 페이지의 비영리 이용 조건도 별도 고려 | 이론 참고; 코드를 복사하지 않음 |
| shell-MPM 연구 | 셸 굽힘과 마찰 접촉을 통합하는 선행 연구 존재 | 범용 Newton MPM 스위치 하나로 동일 모델이 구현되지는 않음 | 차기 대규모 접촉 대안 |
| 상세 flute/liner FEM | 골판 구조, 접착, 층별 압궤를 직접 연구 가능 | 로봇 반복 rollout에는 비쌈. 측정 재료 파라미터와 세부 형상 필요 | 오프라인 기준 모델/동정용 |

NVIDIA 문서도 표면 스키마가 구현보다 더 일반적일 수 있음을 분명히 구분한다. 따라서 `plasticity=true` 같은 임의 속성을 USD에 쓰고 엔진이 처리한다고 주장하지 않는다. 이번 custom API는 실제 등록되는 codeless schema이고, Python/Warp 런타임이 그 속성을 읽고 계산한다. [NVIDIA deformable schema](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/110.1/dev_guide/deformables/omniphysics_deformable_schema.html), [PhysX 구현 제한](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/deformables_beta/deformable_beta.html)

Newton 1.5 VBD에는 revolute/prismatic 관절과 drive가 있다. 예전 1.0 문서의 관절 지원 제한을 현재 설치 버전에 그대로 적용하면 안 된다. 반대로 1.5 VBD의 `joint_effort_limit`은 지원되지 않으므로 그리퍼 구동력은 코드에서 직접 clamp한다. 팔은 안정적인 implicit PD drive로 제어하며, UR 제조사 토크 제한을 검증한 산업용 컨트롤러 모델이라고 주장하지 않는다. [Newton 1.5 SolverVBD](https://newton-physics.github.io/newton/1.5.0/api/_generated/newton.solvers.SolverVBD.html)

## 이론과 실제 구현의 연결

### 물리적으로 더 완전한 골판지 셸

중간면의 막 변형률을 ε, 곡률 변화를 κ라고 하면 에너지는 다음과 같이 쓸 수 있다.

\[
E=\frac12\int_A(\epsilon-\epsilon^p)^T A(d)(\epsilon-\epsilon^p)
 +(\kappa-\kappa^p)^T D(d)(\kappa-\kappa^p)\,dA.
\]

여기서 A는 면내 강성 [N/m], D는 굽힘 강성 [N m], εᵖ·κᵖ는 소성 상태, d는 손상이다. MD(machine direction)/CD(cross direction)의 E, G, ν와 배향을 별도로 식별한다. 균질 등방판의 D=Et³/[12(1−ν²)]는 이해를 위한 식이며, corrugated sandwich에 단일 E와 두께만 넣어 측정된 A와 D를 동시에 맞출 수 있다고 가정해서는 안 된다.

압축과 인장의 항복은 비대칭일 수 있다. 코어 압궤는 두께방향 압축과 이력/치밀화를 갖는 별도 상태가 필요하다. 일반 von Mises 금속 모델이나 granular Drucker–Prager를 골판지 검증 없이 그대로 사용하는 것은 적절하지 않다. 상세 압궤 연구는 실험 전후 강성 저하도 다룬다. [골판지 압궤 실험/FEM](https://doi.org/10.3390/en14113203), [골판지 균질화 연구](https://arxiv.org/abs/1110.5417)

### 이번 데모의 이산 굽힘 소성

각 내부 변 e의 이면각 θₑ에 대해

\[
E_e=\frac12 k_e(d)(\theta_e-\theta^0_e-p_e)^2,
\quad k_e^0=D_e\frac{\ell_e}{h_e},\quad k_e(d)=(1-d_e)k_e^0.
\]

ℓ은 변 길이, h는 인접 두 삼각형 높이의 평균이다. Newton 1.5 VBD의 실제 kernel은 `edge_ke * edge_rest_length`를 각도 에너지 계수로 사용하므로 `edge_ke = D/h`로 대응시킨다. 면내 에너지는 Newton의 기존 isotropic membrane 모델을 사용한다. 방향에 따른 굽힘 강성 보간은 적용하지만, **완전한 직교이방성 막/굽힘 텐서 구현은 아니다**.

trial moment와 항복 함수는

\[
M^{tr}=k_e(\theta_e-\theta_e^0-p_e),\quad
f=|M^{tr}|-(M_y+H\alpha_e),\quad
M_y=k_e^0\kappa_y h_e.
\]

고정 각도 임계값 대신 곡률 임계값 κᵧ [1/m]를 h와 함께 사용해 메시 해상도 의존성을 줄인다. 이것이 완전한 mesh convergence를 보장하는 것은 아니다.

\[
\Delta\gamma=\max(0, f/(k_e+H)),\quad
p_e\leftarrow p_e+\operatorname{sign}(M^{tr})\Delta\gamma,\quad
\alpha_e\leftarrow\alpha_e+\Delta\gamma.
\]

`edge_rest_angle = reference_angle + plastic_angle`를 갱신하므로 힘을 제거해도 p가 남는다. 손상은 `d=min(1−residualStiffness, 1−exp(−damageRate*alpha))`로 표현한다. 기본 잔존강성은 25%이며 무한히 연해지는 것을 막는다. 기록하는 `plasticDissipation`은 MᵧΔγ를 누적한 **소성 유동의 소산**이다. 손상으로 방출되는 에너지까지 포함한 전체 열역학적 에너지 장부는 아니다.

VBD 한 substep → trial angle 평가 → local return mapping → 손상/기준각 갱신 순서의 operator split이다. 완전한 monolithic elastoplastic Newton iteration은 아니다. substep과 반복 수에 대한 민감도를 별도로 확인해야 한다. GPU 업데이트는 `src/cardboard/plasticity.py`에 있고 기존 Newton 소스는 수정하지 않는다.

종이의 주름과 weakening을 굽힘 소성으로 모델링하는 근거는 선행 연구에 있다. 다만 이번 고정 메시/힌지 모델은 그 논문의 adaptive remeshing과 plastic embedding을 재현한 것은 아니다. [Folding and Crumpling Adaptive Sheets](https://escholarship.org/uc/item/6p48z6mj), [ARCSim 공식 배포 설명](https://graphics.berkeley.edu/resources/ARCSim/)

### 접촉과 구동

상자는 속이 빈 6면 셸이다. 공유 정점으로 박스 가장자리를 연결하고, 최초 90° 접힘은 stress-free reference angle로 둔다. 현재 테이프, 개별 flap 분리, 접착 파손은 표현하지 않는다. 렌더 외형의 접힘선과 테이프 무늬는 원본 텍스처를 보존한다.

Newton VBD가 테이블·로봇·그리퍼와의 접촉, 마찰, 셸 자기접촉을 계산한다. 입자 반경은 두께의 절반인 1.5 mm다. 손가락 모터는 `clamp(kp*(q_target-q)-kd*qdot, ±Fmax)`로 구동된다. 이것은 **모터 구동력 제한**이며 충돌 반력의 정확한 상한은 아니다. 관성, 마찰, 손목 운동 때문에 실제 접촉력은 별도로 측정한다.

손목에 걸리는 상자 접촉 모멘트는 각 그리퍼 부품의 반력과 레버암으로 합산한다. torque feedback은 설정치를 넘으면 twist 목표를 되돌리는 supervisor이다. 60 Hz 관측/반응이므로 순간 토크의 엄밀한 제한이나 real-robot safety controller가 아니다. 팔 관절 힘/토크와 상자의 접촉력/모멘트는 다른 물리량이다.

## USD / SimReady 설계

원본은 공개 NVIDIA SimReady `cardbox_a1`이다. `assets/source/manifest.json`에 정확한 URL·크기·SHA-256을 남겼다. 원본의 SimReady 0.9.1 metadata가 실제로 존재한다. 원본 USD, 텍스처는 보존하고, 별도의 파생 USD에서 0.4배 외형과 셸을 만든다. 원본의 thumbnail-rig payload와 MDL dependency를 실행 asset에 끌고 오지 않으며, 원본 albedo/UV를 UsdPreviewSurface로 연결한다.

| USD 표현 | 용도 | 실행에서 소비하는 곳 |
|---|---|---|
| UsdGeom.Mesh, PrimvarsAPI | simulation/render geometry와 원본 UV | geometry loader, skinning |
| UsdShade.Material / MaterialBindingAPI | 이미지 기반 외관 | Isaac Sim / USD renderer |
| UsdPhysics.Scene, RigidBodyAPI, CollisionAPI, MassAPI | UR10, 그리퍼, 테이블의 물리 | Newton USD importer |
| UsdPhysics.RevoluteJoint, PrismaticJoint, FixedJoint, DriveAPI | 로봇과 2지 그리퍼 연결 | Newton importer/driver |
| CardboardMaterialAPI | 두께, 면밀도, 강성, 항복곡률, 손상, 마찰 | VBD model과 Warp return mapping |
| CardboardShellAPI | rest points, hinge topology, reference angles, dual width, source provenance | topology 검증 및 초기화 |
| CardboardPlasticStateAPI | 소성각, 누적각, 소산, 손상, 속도 | checkpoint 저장/복원 |
| CardboardBindingAPI | simulation→render 삼각형/가중치/offset | 원본 그래픽 표면 변형 |
| CardboardDemoAPI | 시간간격, 반복 수, 힘, 간격, twist, 대상 관계 | 데모 제어 |

Codeless API에는 `plugInfo.json`과 `generatedSchema.usda`가 있으며 schema registry에서 실제 적용/속성 기본값 조회를 검증한다. 별도의 C++ USD 빌드가 필요하지 않다. [OpenUSD schema 생성 공식 설명](https://openusd.org/24.08/tut_generating_new_schema.html)

새로운 AOUSD deformable proposal과 Newton 최신 deformable import는 진화 중이다. 설치된 1.5에서 보장되지 않는 이름을 마치 표준 확정 API처럼 쓰는 대신, 안정적인 standard USD와 명시적인 프로젝트 API를 조합했다. Newton의 지원 범위를 넘는 재료 정보를 자체 reader가 처리한다. [Newton USD schema resolver](https://newton-physics.github.io/newton/latest/concepts/usd_parsing.html)

**USD 스키마는 데이터 계약이지 solver 구현 자체가 아니다.** `demo_scene.usda`를 일반 Isaac Sim에서 열고 PhysX Play만 누르는 것으로 이 custom plasticity가 실행되지는 않는다. 제공하는 Newton runner/live bridge로 실행하거나 계산한 `replay.usdc`를 재생해야 한다. 파생 asset은 이 런타임에서 simulation-ready인 연구 확장 자산이며, NVIDIA 전체 SimReady conformance 인증을 받았다고 표시하지 않는다.

## 실제 재료로 맞추는 절차

1. 사용할 상자의 치수, 실제 질량, 단일/이중골, flute 방향, 두께, 습도, tape와 접착부를 측정한다.
2. MD/CD 인장·압축과 전단 coupon으로 면내 A를 식별한다.
3. MD/CD 3점 또는 4점 굽힘을 하중–제하까지 측정해 D, 항복곡률, 경화, 잔존강성을 맞춘다.
4. 평압 crush와 edge crush, box compression으로 두께압궤와 구조좌굴을 구분한다.
5. 실제 그리퍼 pad, 마찰, 힘 ramp로 squeeze–release–twist를 수행하고 F–변위, 모멘트–각도, 최종 3D 형상을 기록한다.
6. 먼저 탄성 해석의 반복 수/시간간격/메시 수렴을 확인한다. 수치적으로 부드러운 것을 재료가 부드럽다고 동정하지 않는다.
7. 소성/손상 파라미터를 훈련 데이터로 식별하고, 다른 box·load history·습도에서 검증한다. 이후 검증된 범위 안에서 SDG randomization을 한다.

측정 데이터가 없으므로 이번 파라미터는 모두 `illustrativeUncalibrated`다. 데모의 N/Nm 수치는 수치 모델의 결과이며 해당 실제 택배 박스의 파괴 하중 예측값이 아니다.

## 선행자료 목록

- [Newton VBD 1.5 API](https://newton-physics.github.io/newton/1.5.0/api/_generated/newton.solvers.SolverVBD.html): 설치 버전의 관절·구동 지원과 제한.
- [Vertex Block Descent 연구](https://graphics.cs.utah.edu/research/projects/vbd/): implicit energy minimization 방법. 설치 소스 docstring의 DOI: 10.1145/3658179.
- [Augmented VBD 연구](https://graphics.cs.utah.edu/research/projects/avbd/): 강체/관절 제약의 AL 확장.
- [Newton implicit MPM](https://newton-physics.github.io/newton/1.5.2/api/_generated/newton.solvers.SolverImplicitMPM.html): volumetric elastoplastic 대안과 재료 파라미터.
- [PhysX deformable limitation](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/deformables_beta/deformable_beta.html): rest-shape 업데이트/표면 재료 구현 제한.
- [Omni Physics deformable schema](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/110.1/dev_guide/deformables/omniphysics_deformable_schema.html): 표현과 구현의 차이.
- [NVIDIA SimReady asset packs](https://docs-prod.omniverse.nvidia.com/usd/latest/usd_content_samples/downloadable_packs.html): cardboard 포함 공개 콘텐츠.
- [SimReady FAQ](https://docs.omniverse.nvidia.com/simready/latest/simready-faq.html): 교환·검증 가능한 물리 정보의 역할.
- [ARCSim](https://graphics.berkeley.edu/resources/ARCSim/), [2013 논문](https://escholarship.org/uc/item/6p48z6mj): 굽힘 소성, weakening, 주름 적응 메시.
- [Thin shell MPM 원 논문](https://math.ucdavis.edu/~jteran/papers/GHFGTT18.pdf): 얇은 셸과 접촉을 위한 별도 MPM 구성식.
- [골판지 압궤 실험/수치 연구](https://doi.org/10.3390/en14113203): 실제 강성 저하와 정량 검증.
- [골판지 orthotropic homogenization](https://arxiv.org/abs/1110.5417): 균질화 접근.
- [OpenUSD codeless schema](https://openusd.org/24.08/tut_generating_new_schema.html): 등록 가능한 데이터 계약.

조사 범위는 공개 NVIDIA 문서·코드·자산과 외부 원 논문이다. NVIDIA 비공개 사내 문서까지 검색했다고 주장하지 않는다.
