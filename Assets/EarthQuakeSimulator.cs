using UnityEngine;
using System.Collections;
using NUnit.Framework;

// [졸업작품용 지진 시뮬레이터 - 물리 엔진 연동 버전]
// 변경사항:
//   - 바닥: transform.position → Rigidbody.MovePosition (물리 엔진 인식)
//   - 가구: Rigidbody 자동 수집, isKinematic=false로 설정하여 물리 반응
//   - 시뮬레이션 종료 후 가구 velocity 초기화

[RequireComponent(typeof(Rigidbody))]
public class ScientificSeismicSimulator : MonoBehaviour
{
    #region 1. 사용자 설정 (User Settings)
    [Header("1. 지진 발생원 정보 (Source Parameters)")]
    [Tooltip("리히터 규모 (Richter Magnitude)")]
    [UnityEngine.Range(0f, 9.0f)]
    public float magnitude = 6.0f;

    [Tooltip("진앙지로부터의 거리 (Epicentral Distance, km)")]
    [UnityEngine.Range(0f, 800f)]
    public float distance = 20.0f;

    [Tooltip("지진 지속 시간 (Duration, sec)")]
    public float duration = 15.0f;

    [Header("2. 물리 상수 (Physics Constants)")]
    public float vP = 6.0f;
    public float vS = 3.5f;
    public float sensitivity = 0.015f;

    [Header("3. 가구 설정 (Furniture)")]
    [Tooltip("가구 태그 - 이 태그 달린 오브젝트 전부 물리 적용")]
    public string furnitureTag = "Furniture";
    #endregion

    #region 2. 내부 변수 (Internal Variables)
    private Rigidbody floorRb;          // 바닥 Rigidbody
    private Rigidbody[] furnitureRbs;   // 가구 Rigidbody 목록

    private Vector3 initialPosition;
    private bool isSimulating = false;

    private float calculatedPGA;
    private float sWaveLagTime;
    private float dominantFrequency;
    #endregion

    void Start()
    {
        initialPosition = transform.position;

        // ── 바닥 Rigidbody 설정 ──────────────────────────────────────────────
        floorRb = GetComponent<Rigidbody>();
        floorRb.isKinematic = true;     // 바닥은 중력/충돌 영향 안 받고 직접 제어
        floorRb.interpolation = RigidbodyInterpolation.Interpolate; // 부드러운 이동

        // ── 가구 Rigidbody 수집 ──────────────────────────────────────────────
        // "Furniture" 태그가 달린 오브젝트 전부 수집
        GameObject[] furnitureObjects = GameObject.FindGameObjectsWithTag(furnitureTag);
        furnitureRbs = new Rigidbody[furnitureObjects.Length];

        for (int i = 0; i < furnitureObjects.Length; i++)
        {
            Rigidbody rb = furnitureObjects[i].GetComponent<Rigidbody>();
            if (rb == null)
                rb = furnitureObjects[i].AddComponent<Rigidbody>();

            rb.isKinematic = false;
            rb.interpolation = RigidbodyInterpolation.Interpolate;
            furnitureRbs[i] = rb;
        }

        Debug.Log($"✅ 가구 {furnitureRbs.Length}개 물리 등록 완료");
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space) && !isSimulating)
        {
            StartCoroutine(SimulateSequence());
        }
    }

    // ------------------------------------------------------------------------
    // [메인 루틴] 전체 시뮬레이션 흐름 제어
    // ------------------------------------------------------------------------
    IEnumerator SimulateSequence()
    {
        isSimulating = true;
        CalculateEarthquakeParameters();

        float elapsedTime = 0.0f;
        Debug.Log($"🚨 시뮬레이션 시작! (PGA: {calculatedPGA:F4} gal, Lag: {sWaveLagTime:F2}s)");

        while (elapsedTime < duration)
        {
            Vector3 vibration = CalculateVibrationAtTime(elapsedTime);

            // ── 핵심 변경: MovePosition으로 물리 엔진에 이동 알림 ────────────
            // transform.position 직접 수정 시 가구가 바닥 움직임을 인식 못함
            // MovePosition은 Rigidbody에 충돌/마찰 계산을 유지하면서 이동
            floorRb.MovePosition(initialPosition + vibration);

            elapsedTime += Time.deltaTime;
            yield return new WaitForFixedUpdate(); // FixedUpdate 주기에 맞춤
        }

        // ── 종료: 바닥 원위치 + 가구 velocity 초기화 ────────────────────────
        floorRb.MovePosition(initialPosition);

        foreach (Rigidbody rb in furnitureRbs)
        {
            if (rb == null) continue;
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        isSimulating = false;
        Debug.Log("✅ 상황 종료");
    }

    // ------------------------------------------------------------------------
    // 물리 파라미터 사전 계산 (Esteva & Time Lag)
    // ------------------------------------------------------------------------
    void CalculateEarthquakeParameters()
    {
        float numerator = 5600f * Mathf.Exp(0.8f * magnitude);
        float denominator = Mathf.Pow((distance + 40f), 2.0f);
        calculatedPGA = numerator / denominator;

        float lag = (distance / vS) - (distance / vP);
        sWaveLagTime = Mathf.Clamp(lag, 0f, 5.0f);

        dominantFrequency = Mathf.Max(1.0f, 12.0f - magnitude);
    }

    // ------------------------------------------------------------------------
    // 특정 시간(t)의 진동 벡터 산출 (Perlin Noise & Envelope)
    // ------------------------------------------------------------------------
    Vector3 CalculateVibrationAtTime(float time)
    {
        float normalizedTime = time / duration;
        float envelope = 4.0f * normalizedTime * (1.0f - normalizedTime);

        bool isSWaveArrived = time > sWaveLagTime;
        float phaseIntensity = isSWaveArrived ? 1.0f : 0.2f;

        Vector3 noiseVector = GeneratePerlinNoiseVector(time);

        float power = calculatedPGA * sensitivity * envelope * phaseIntensity;

        if (isSWaveArrived)
        {
            // S파(횡파): 수평(X, Z) 진동 우세
            return new Vector3(noiseVector.x, noiseVector.y * 0.3f, noiseVector.z) * power;
        }
        else
        {
            // P파(종파): 수직(Y) 진동 우세
            return new Vector3(noiseVector.x * 0.2f, noiseVector.y * 0.8f, noiseVector.z * 0.2f) * power;
        }
    }

    // ------------------------------------------------------------------------
    // 펄린 노이즈 생성
    // ------------------------------------------------------------------------
    Vector3 GeneratePerlinNoiseVector(float time)
    {
        float x = (Mathf.PerlinNoise(time * dominantFrequency, 0f) - 0.5f) * 2f;
        float y = (Mathf.PerlinNoise(0f, time * dominantFrequency) - 0.5f) * 2f;
        float z = (Mathf.PerlinNoise(time * dominantFrequency, time * dominantFrequency) - 0.5f) * 2f;

        return new Vector3(x, y, z);
    }
}