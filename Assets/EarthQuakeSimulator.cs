using UnityEngine;
using System.Collections;

// ============================================================
//  ScientificSeismicSimulator - 졸업작품용 지진 시뮬레이터
// ============================================================
//
// ■ 지반 가속도 모델: Kanai-Tajimi (Kanai 1957, Tajimi 1960)
//   - 백색잡음(White Gaussian Noise)을 SDOF 지반 필터에 통과시켜
//     실제 지반 가속도 시계열을 생성
//   - ODE: x'' + 2ζg·ωg·x' + ωg²·x = -ωg²·w(t)
//   - Euler 수치적분 (매 FixedUpdate 1스텝)
//
// ■ 지반 파라미터: Clough & Penzien (1993)
//   암반(Rock):    ωg=15.0 rad/s,  ζg=0.60
//   경질토(Firm):  ωg=10.0 rad/s,  ζg=0.50  ← 기본값
//   연질토(Soft):  ωg= 5.0 rad/s,  ζg=0.40
//
// ■ 포락선 모델: Jennings, Housner, Tsai (1968)
//   - 구간1 P파:  [0, tS]              → 진폭 비율 0.2
//   - 구간2 전이: [tS, tS+tTrans]      → Smoothstep(3x²-2x³) 0.2→1.0
//   - 구간3 S파:  [tS+tTrans, 전체]    → 진폭 비율 1.0
//   - 전체 포락: 4t(1-t) (점증-최대-감쇠)
//
// ■ PGA 감쇠식: Esteva (1970)
//   PGA [gal] = 5600 · exp(0.8·M) / (R+40)²
//
// ■ 지반 변위 변환: Newmark & Hall (1982)
//   D [m] = (PGA [m/s²]) / ωg²
//   → 바닥(Floor)의 MovePosition 진폭에 사용
//
// ■ Unity 물리 적용 방식
//   · 바닥 (isKinematic Rigidbody):
//       MovePosition(initialPos + D·normalizedAcc)
//       → 시각적 지면 흔들림 재현
//   · 가구 (Rigidbody, mass 반영):
//       AddForce(acc_m_s2, ForceMode.Acceleration)
//       → ForceMode.Acceleration = 질량 무관 가속도 직접 적용
//       → 내부적으로 F = m·a 자동 처리 (Unity Docs)
//       → 무거운 가구(mass↑)는 덜 움직임, 가벼운 가구는 더 움직임
//
// ■ 참고문헌
//   Kanai, K. (1957). Semi-empirical formula for the seismic
//     characteristics of the ground. Bull. Earthquake Res. Inst., 35, 309-325.
//   Tajimi, H. (1960). A statistical method of determining the maximum
//     response of a building. Proc. 2nd WCEE, 781-798.
//   Clough, R.W. & Penzien, J. (1993). Dynamics of Structures, 2nd ed.
//   Esteva, L. (1970). Seismic risk and seismic design decisions.
//     Proc. 4th WCEE.
//   Jennings, P.C., Housner, G.W., Tsai, N.C. (1968). Simulated
//     Earthquake Motions. EERL Report, Caltech.
//   Newmark, N.M. & Hall, W.J. (1982). Earthquake Spectra and Design.
//     EERI Monograph.
//   Unity Technologies. Rigidbody.AddForce - ForceMode.Acceleration.
//     https://docs.unity3d.com/ScriptReference/Rigidbody.AddForce.html
// ============================================================

[RequireComponent(typeof(Rigidbody))]
public class ScientificSeismicSimulator : MonoBehaviour
{
    #region Inspector Parameters

    [Header("── 1. 지진 발생원 (Esteva 1970) ──────────────────")]
    [Tooltip("리히터 규모")]
    [Range(0f, 9f)]
    public float magnitude = 6.0f;

    [Tooltip("진앙 거리 (km)")]
    [Range(1f, 800f)]
    public float distance = 20.0f;

    [Tooltip("지진 지속 시간 (sec)")]
    public float duration = 15.0f;

    [Header("── 2. P/S파 전파 속도 ──────────────────────────")]
    [Tooltip("P파 속도 (km/s) - 일반 지각 평균값")]
    public float vP = 6.0f;
    [Tooltip("S파 속도 (km/s) - 일반 지각 평균값")]
    public float vS = 3.5f;

    [Header("── 3. Kanai-Tajimi 지반 파라미터 (Clough & Penzien 1993) ──")]
    [Tooltip("지반 고유 각진동수 (rad/s)\n암반=15 / 경질토=10 / 연질토=5")]
    public float omegaG = 10.0f;
    [Tooltip("지반 감쇠비\n암반=0.60 / 경질토=0.50 / 연질토=0.40")]
    public float zetaG = 0.50f;

    [Header("── 4. Jennings(1968) P→S파 전이 ─────────────────")]
    [Tooltip("P파→S파 진폭이 부드럽게 전환되는 시간 (sec)")]
    public float sWaveTransitionDuration = 1.5f;

    [Header("── 5. 가구 설정 ──────────────────────────────")]
    public string furnitureTag = "Furniture";

    #endregion

    #region Private State

    private Rigidbody _floorRb;
    private Rigidbody[] _furnitureRbs;
    private Vector3 _initialPos;
    private bool _isSimulating;

    // 계산된 지진 파라미터
    private float _pga_m_s2;      // PGA [m/s²]
    private float _dispAmp;       // 지반 변위 진폭 [m]  Newmark & Hall
    private float _sWaveLag;      // S파 도달 지연 [sec]

    // Kanai-Tajimi 필터 내부 상태 (X/Y/Z 독립)
    private float _xDisp, _xVel;
    private float _zDisp, _zVel;
    private float _yDisp, _yVel;

    #endregion

    // ──────────────────────────────────────────────────────────
    void Start()
    {
        _initialPos = transform.position;

        // 바닥: isKinematic → MovePosition으로만 제어
        _floorRb = GetComponent<Rigidbody>();
        _floorRb.isKinematic = true;
        _floorRb.interpolation = RigidbodyInterpolation.Interpolate;

        // 가구 Rigidbody 수집 및 설정
        GameObject[] objs = GameObject.FindGameObjectsWithTag(furnitureTag);
        _furnitureRbs = new Rigidbody[objs.Length];
        for (int i = 0; i < objs.Length; i++)
        {
            Rigidbody rb = objs[i].GetComponentInParent<Rigidbody>()
                        ?? objs[i].AddComponent<Rigidbody>();

            rb.isKinematic = false;
            rb.interpolation = RigidbodyInterpolation.Interpolate;
            rb.linearDamping = 0.1f;
            rb.angularDamping = 0.5f;
            rb.sleepThreshold = 0f;
            rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
            _furnitureRbs[i] = rb;
        }

        Debug.Log($"[Seismic] ✅ 가구 {_furnitureRbs.Length}개 등록 완료");
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space) && !_isSimulating)
            StartCoroutine(SimulateSequence());
    }

    // ──────────────────────────────────────────────────────────
    IEnumerator SimulateSequence()
    {
        _isSimulating = true;
        ComputeParameters();
        ResetFilter();

        Debug.Log(
            $"[Seismic] 🚨 시뮬레이션 시작\n" +
            $"  M={magnitude}  R={distance}km\n" +
            $"  PGA = {_pga_m_s2 * 100f:F2} gal  ({_pga_m_s2:F4} m/s²)  [Esteva 1970]\n" +
            $"  지반변위 = {_dispAmp * 100f:F3} cm  [Newmark & Hall 1982: D=PGA/ωg²]\n" +
            $"  S파 지연 = {_sWaveLag:F2} s  (vP={vP}, vS={vS} km/s)\n" +
            $"  ωg={omegaG} rad/s  ζg={zetaG}  [Clough & Penzien 1993]"
        );

        float t = 0f;
        while (t < duration)
        {
            // 1. Kanai-Tajimi → 정규화된 가속도 벡터 (크기 ≈1)
            Vector3 accNorm = ComputeNormalizedAcceleration(t);

            // 2. 포락선 + 위상 강도
            float env = 4f * (t / duration) * (1f - t / duration);   // Jennings 1968
            float phase = ComputePhaseIntensity(t);                      // Jennings 1968
            float scale = env * phase;

            // ── 바닥: 변위 기반 MovePosition (시각적 흔들림) ──────────
            // D = PGA / ωg²  [Newmark & Hall 1982]
            Vector3 floorDisp = accNorm * (_dispAmp * scale);
            _floorRb.MovePosition(_initialPos + floorDisp);

            // ── 가구: 가속도 기반 AddForce (관성력) ─────────────────
            // ForceMode.Acceleration → Unity가 내부적으로 F=m·a 처리
            // 무거운 가구(mass↑)는 같은 가속도라도 더 큰 F가 필요
            // → 실제 물리: 가속도는 동일하지만 마찰 저항(μ·m·g)이 커서 덜 움직임
            Vector3 groundAcc = accNorm * (_pga_m_s2 * scale);
            foreach (Rigidbody rb in _furnitureRbs)
            {
                if (rb == null) continue;
                rb.WakeUp();
                rb.AddForce(groundAcc, ForceMode.Acceleration);
            }

            t += Time.fixedDeltaTime;
            yield return new WaitForFixedUpdate();
        }

        // 종료: 바닥 원위치, 가구 속도 초기화
        _floorRb.MovePosition(_initialPos);
        foreach (Rigidbody rb in _furnitureRbs)
        {
            if (rb == null) continue;
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        _isSimulating = false;
        Debug.Log("[Seismic] ✅ 시뮬레이션 종료");
    }

    // ──────────────────────────────────────────────────────────
    // Esteva(1970) + Newmark & Hall(1982) 파라미터 계산
    void ComputeParameters()
    {
        // PGA [gal] = 5600·exp(0.8M) / (R+40)²
        float pga_gal = 5600f * Mathf.Exp(0.8f * magnitude)
                        / Mathf.Pow(distance + 40f, 2f);

        _pga_m_s2 = pga_gal / 100f;   // gal → m/s²

        // Newmark & Hall(1982): D = PGA[m/s²] / ωg²
        _dispAmp = _pga_m_s2 / (omegaG * omegaG);

        // P/S파 도달 시차 (Snell의 법칙)
        _sWaveLag = Mathf.Clamp((distance / vS) - (distance / vP), 0f, 5f);
    }

    void ResetFilter()
    {
        _xDisp = _xVel = 0f;
        _zDisp = _zVel = 0f;
        _yDisp = _yVel = 0f;
    }

    // ──────────────────────────────────────────────────────────
    // Kanai-Tajimi 필터 → 정규화 가속도 벡터
    // 출력은 방향 벡터(크기 ≈1), 실제 크기는 호출부에서 scale 곱
    Vector3 ComputeNormalizedAcceleration(float time)
    {
        float dt = Time.fixedDeltaTime;

        // Box-Muller 변환: 균등분포 → 표준정규분포 N(0,1)
        float wX = Mathf.Sqrt(-2f * Mathf.Log(Mathf.Clamp(Random.value, 1e-6f, 1f)))
                   * Mathf.Cos(2f * Mathf.PI * Random.value);
        float wZ = Mathf.Sqrt(-2f * Mathf.Log(Mathf.Clamp(Random.value, 1e-6f, 1f)))
                   * Mathf.Cos(2f * Mathf.PI * Random.value);
        float wY = Mathf.Sqrt(-2f * Mathf.Log(Mathf.Clamp(Random.value, 1e-6f, 1f)))
                   * Mathf.Cos(2f * Mathf.PI * Random.value);

        // Kanai-Tajimi ODE 수치적분 (Euler)
        float ax = KanaiTajimiStep(wX, ref _xDisp, ref _xVel, dt);
        float az = KanaiTajimiStep(wZ, ref _zDisp, ref _zVel, dt);
        float ay = KanaiTajimiStep(wY, ref _yDisp, ref _yVel, dt);

        // P파/S파 수평·수직 성분 비율 적용
        bool fullS = time > _sWaveLag + sWaveTransitionDuration;
        Vector3 raw;
        if (fullS)
            raw = new Vector3(ax, ay * 0.05f, az);   // S파(횡파): 수평 우세
        else
            raw = new Vector3(ax * 0.2f, ay * 0.4f, az * 0.2f);  // P파(종파): 수직 우세

        // 정규화 (0벡터 방지)
        return raw.magnitude > 1e-6f ? raw.normalized : Vector3.zero;
    }

    // Kanai-Tajimi 1스텝 Euler 적분
    // x'' = -ωg²·w - 2ζg·ωg·x' - ωg²·x
    float KanaiTajimiStep(float w, ref float disp, ref float vel, float dt)
    {
        float acc = -(omegaG * omegaG) * w
                    - 2f * zetaG * omegaG * vel
                    - (omegaG * omegaG) * disp;
        vel += acc * dt;
        disp += vel * dt;
        // ±5 클램프: Euler 누적 발산 방지 (PGA 유한값과 동등)
        vel = Mathf.Clamp(vel, -5f, 5f);
        disp = Mathf.Clamp(disp, -5f, 5f);
        return disp;
    }

    // Jennings et al.(1968) P→S파 위상 강도
    // Smoothstep f(x)=3x²-2x³: 양 끝 미분=0 → 충격 없는 연속 전환
    float ComputePhaseIntensity(float time)
    {
        if (time < _sWaveLag)
            return 0.2f;
        if (time < _sWaveLag + sWaveTransitionDuration)
        {
            float x = (time - _sWaveLag) / sWaveTransitionDuration;
            return Mathf.Lerp(0.2f, 1.0f, x * x * (3f - 2f * x));
        }
        return 1.0f;
    }
}