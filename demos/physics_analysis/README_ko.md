# Waypoint 비행 및 물리·센서 공격 분석 데모

[English](README.md) | 한국어

기존 **ArduCopter 4.6.2 / QEMU / Rumoca FMU**를 이용해 10개 지그재그
waypoint를 비행하고 RTL로 복귀하는 선택 설치형 데모입니다. 무공격 대조군과
네 공격 케이스가 같은 미션을 사용합니다. 기록·공격 모델·미션 제어·분석·Three.js
가시화는 모두 이 디렉터리에 있으며 FastDyn 코어, 공용 Modelica 모델, 서브모듈은
수정하지 않습니다.

## 공통 미션과 공격 케이스

`scenarios/_mission.toml`에서 공통 미션을 정의합니다. 15m 이륙 후 홈 기준
`(북,동) = (10,-10), (20,10), …, (90,-10), (100,10)m`를 따라 비행하고 RTL합니다.
미션 실행기는 **GUIDED 위치 목표를 주기적으로 전송**합니다. AUTO 모드로
업로드하는 미션은 아닙니다. 제어기 추정 위치가 수평 반경 2m·고도 ±1m 안에서
1초간 유지되면 다음 waypoint로 이동합니다. 항로 비행 제한 시간은 시뮬레이션
360초이며, RTL 후 낮은 고도가 지속되는지 확인하고 일반 disarm 요청으로
착륙을 마무리합니다. 기존 초기화 파라미터와 `DISARM_DELAY=0`은 유지합니다.

| 시나리오 | 공통 미션에 추가되는 입력 |
|---|---|
| `baseline` | 무공격 대조군 |
| `magnetic` | 모델 전류에 비례하는 기체 FRD 자기장 간섭, 기본 배율 1 |
| `wind` | 서쪽(270°)에서 평균 5m/s + 최대 10m/s 돌풍, 최대 풍속 15m/s |
| `gps-spoof` | GPS 위치를 북쪽 +12m·동쪽 −8m·수직 0m 이동하고 변화율에 맞는 속도도 주입 |
| `gyro` | yaw 바이어스 +3°/s와 0.25Hz 허위 각속도: yaw ±8°/s, roll ±2°/s |

공격 시간은 이륙 후 경과 시간이 아닌 **FMU 절대 시간 140–200초**입니다.
시작·종료 각각 10초 동안 smoothstep으로 변화하며 150–190초에 전체 세기를
적용합니다. 검증 비행에서는 후반 waypoint와 RTL 초반에 걸쳐 적용됐습니다.
이륙 완료 전에 공격이 시작된 경우 실행기는 해당 비행을 실패로 기록합니다.
대조군에도 같은 비교 시간 구간을 두어 공격 전·중·후 지표를 계산합니다.
기존 hover·고도 진동·사각형 경로 시나리오는 대체했으며 Monte Carlo도 같은
지그재그 미션을 사용합니다.

개별 TOML의 `[attack]`을 수정하거나 사용자 TOML 경로를 넘길 수 있습니다.
사용자 파일은 공통 미션을 상속하고 필요한 미션 필드를 덮어씁니다. 자이로
바이어스와 허위 각속도는 독립된 파라미터이므로 한쪽을 0으로 설정하면 각각
시험할 수 있습니다. 설정 단위는 도·도/초이며, 제어기로 전달하는 자이로 출력은
rad/s로 변환합니다.

## 빌드와 실행

FastDyn 루트에서 실행합니다. 기존 FastDyn 환경, QEMU/플러그인, 펌웨어,
Rumoca 컴파일러 및 FMI3 헤더가 준비되어 있어야 합니다.

```bash
fastdyn-env/bin/python -m pip install -e 'demos/physics_analysis[test]'
fastdyn-env/bin/python demos/physics_analysis/demo.py build
fastdyn-env/bin/python demos/physics_analysis/demo.py run baseline \
  --output out/physics_analysis/my-baseline
fastdyn-env/bin/python demos/physics_analysis/demo.py campaign attacks \
  --output out/physics_analysis/my-attacks
```

`campaign attacks`는 대조군·자기장·바람·GPS·자이로 순서로 실행하고 실패도
보존하여 통합 재생 파일을 만듭니다. 개별 실행은 `run gps-spoof`처럼 지정합니다.
`run --duration`은 항로 비행 제한 시간, `--timeout`은 전체 실행의 실제 시간
watchdog(기본 900초)을 바꿉니다.

기본 빌드 위치는 `out/physics_analysis/waypoint-attacks/build/`이며
`pristine/`, `observed/`, `build.json`이 생성됩니다. 로컬
`third_party/common/rumoca/target/debug/rumoca`를 사용하고 다른 컴파일러는
`build --compiler PATH`로 선택합니다. 기록용 C 래퍼는 observed FMU에만
추가합니다. 이전 데모 FMU는 호환성 검사로 거부하며, 실행 중인 파일을 교체하지
않도록 기존 빌드·실행 디렉터리는 재사용하지 않습니다. 재빌드 시 새 `--output`과
실행용 `--build-dir`을 지정합니다. 원본 연구의 펌웨어·결과 파일은 복사하지 않습니다.

```bash
# 대조군 + seed 42로 추출한 바람 3개: 총 4회 비행
fastdyn-env/bin/python demos/physics_analysis/demo.py campaign monte-carlo \
  --seed 42 --count 3 --output out/physics_analysis/my-mc
# QEMU 실행 없이 설정만 생성
fastdyn-env/bin/python demos/physics_analysis/demo.py campaign attacks \
  --prepare-only --output out/physics_analysis/my-plan
```

RAM·QMP 소켓·포트·로그는 실행별로 분리됩니다. 병렬 실행 시 `--port-base`를
15550, 15650 등으로 구분합니다. 각 실행은 monitor/firmware/GCS/GPS용 연속
4개 포트를 사용합니다. Unix 소켓 경로 제한 때문에 출력 경로는 짧게 유지하세요.
고정 seed는 추출한 파라미터를 재현하며 비동기 QEMU/MAVLink 실행 결과까지
비트 단위로 같게 만들지는 않습니다. 소규모 실행은 통계적 신뢰성 검증이 아닙니다.

## 모델과 공격 주입 위치

`models/PhysicsAnalysis/Copter.mo`가 기존 `FastDyn.Copter`를 `vehicle`로
포함하고, `AttackInputs.mo`가 시간에 따른 공격 신호를 만듭니다. 바람은 상대
공기 속도에 따른 항력과 기존 무풍 항력의 차이를 외력으로 적용합니다. 무풍이면
추가 힘은 0입니다. 센서 공격은 외부 출력 `mag`, `gps`, `vel_ned`, `gyro`에만
적용하며 실제 기체 위치·자세·각속도를 직접 덮어쓰지 않습니다. GPS 고도와 수직
속도의 부호도 일관되게 처리하고 기압계는 독립된 실제 고도를 제공합니다.
제어기에 GPS 정답 방위각이 유입되지 않도록 데모의 `yaw_deg=0`, 즉
`GPS_INPUT`의 yaw 미제공 값을 사용합니다. 실제 방위각은 quaternion으로부터
계산하여 분석용으로만 보관합니다.

자기장 간섭은 원본 연구의 FRD 기울기
`[0.00489157, 0.00473074, -0.00704143] Gauss/A`를 참고했습니다.
모델 전류는 `max(0, 0.42 + 42.5908 * mean(omega_i² / omega_max²)) A`입니다.
이는 **배터리·전기 모델이나 실측 전류가 아닌 모터 부하 기반 대리값**이며
`model_current_a`로 별도 기록합니다. `current_a`·`voltage_v`는 계속 빈 값이고
JSON에서는 `null`입니다. 이 파라미터는 시험 입력이지 실제 기체의 교정값이 아닙니다.

공격은 모델 내부 센서 신호 변조 및 환경 외란으로 구현했습니다. 실제 전파 공격이나
외부 GPS 송신을 수행하지 않으며, 원본 연구의 CompassMot·전류 보상 결과를
재현한다고 주장하지 않습니다.

## Rumoca 실시간 뷰어와 오프라인 재생

여기서 온라인은 실행 중인 동일한 시뮬레이션의 로컬 실시간 관측을 뜻하며,
외부 Rumoca 호스팅 사이트에 데이터를 올리지 않습니다. 실제 Rumoca 서브모듈의
뷰어·quadrotor 장면을 제공하고 데모 전용 궤적·waypoint 표시를 덧붙입니다.

```bash
# 시뮬레이션보다 먼저 실행해도 truth 파일을 기다립니다.
fastdyn-env/bin/python demos/physics_analysis/demo.py serve \
  --truth out/physics_analysis/my-baseline/truth.csv
# 또는 저장한 다섯 케이스를 재생합니다.
fastdyn-env/bin/python demos/physics_analysis/demo.py serve \
  --replay out/physics_analysis/my-attacks/replay.json
```

실시간: <http://127.0.0.1:8080/>, 재생: <http://127.0.0.1:8080/replay/>.
HTTP/WS 포트는 `--port`, `--ws-port`로 변경하며 기본값은 8080/8081입니다.
`--truth`와 `--replay`를 함께 지정해도 됩니다. 서버는 localhost에만 바인딩하며
뷰어에서 보낸 비행 조작은 무시합니다. 미션 실행기만 비행을 제어하고, FMU는 CSV를
기록하므로 느린 브라우저를 기다리지 않습니다.

재생 화면은 두 케이스 비교, 재생/정지, 속도, 시간 탐색, 카메라, 궤적 길이를
지원합니다. 번호가 있는 원은 도달 여부와 무관하게 전체 계획 waypoint를,
회색 경로는 RTL을 나타냅니다. **기체와 실선은 실제 상태**, **노란색 표식·선은
제어기 추정 상태**입니다. 고도·방위각 오차·위치 추정 오차·구간 추종 오차를
그래프로 표시하고 공격 시간은 음영으로 표시합니다. 주입값과 모델 전류 대리값도
별도로 보여줍니다.

`--replay`를 바꾸거나 선택한 파일을 교체하면 열려 있는 재생 탭을 새로고침하세요.
재생 데이터는 페이지를 열 때 읽으므로 서버만 다시 시작해도 브라우저에 이미
읽어 둔 데이터가 자동으로 교체되지는 않습니다. 서버 시작 로그에 전체 파일 경로를,
화면의 `Source:`에 폴더·파일명을 표시합니다. 재생 응답은 브라우저 캐시를 사용하지
않으며, 존재하지 않는 재생 파일은 서버 시작 시 거부합니다. 실시간 `--truth`
파일은 서버 실행 후 생성해도 됩니다.

Three.js·기체·텍스처는 로컬 파일로 제공되어 인터넷/CDN 없이 재생할 수 있습니다.
`file://` 대신 HTTP 서버를 사용해야 합니다. 기체 표시 배율과 프로펠러 회전은
가시화용이며 충돌 형상과 같다는 의미가 아닙니다.

```bash
fastdyn-env/bin/python demos/physics_analysis/demo.py export \
  out/physics_analysis/run-a out/physics_analysis/run-b \
  --output out/physics_analysis/comparison.json
# 기존 연구의 truth CSV도 가져올 수 있습니다.
fastdyn-env/bin/python demos/physics_analysis/demo.py export /path/to/truth.csv \
  --output out/physics_analysis/imported.json
```

외부 CSV 필수 열은 `sim_time_s,north_m,west_m,alt_m,qw,qx,qy,qz`입니다.
나머지는 선택 사항이며 telemetry·목표·완료 정보가 없으면 해당 지표는 비워 둡니다.

## 지표와 재현성

기록이 있는 실행은 실패한 경우도 `replay.json`, `replay.metrics.json`,
`replay.metrics.csv`를 생성합니다.

- `completed`는 **제어기 기준 미션·RTL 착륙 완료**이며 스푸핑 상태에서 실제
  위치가 모든 목표를 정확히 통과했다는 의미가 아닙니다.
- `waypoints`는 현재 명령한 지점에 대한 실제 위치를 검증합니다.
  `truth_waypoints_reached`는 같은 수평 2m·고도 ±1m 범위에서 1초 연속 체류해야
  인정하며 20Hz 경계 표본 오차 0.051초를 허용합니다. 최소 거리와 제어기 도착
  보고 시 실제 거리도 기록합니다. 미래 항로점을 미리 지나갔다고 도착 처리하지 않습니다.
- 구간 RMS는 현재 명령 구간까지의 수평 거리이며, 전체 항로 중 가장 가까운
  구간까지의 거리가 아닙니다. 목표 거리와 고도 오차도 별도 계산합니다.
- 방위각 오차는 제어기 yaw에서 실제 quaternion 기반 방위각을 뺀 뒤 ±180°로
  정규화합니다. 위치 추정 오차는 제어기와 실제 위치의 수평 차이입니다.
  RMS·절댓값 P95·최댓값을 제공합니다.
- 공격 전·중·후 지표는 **waypoint 비행 구간**을 대상으로 계산합니다. RTL은
  추종 지표에서 제외하고 최종 실제 홈 수평 오차를 따로 보고합니다. 주입 최댓값과
  관측한 공격 시간은 RTL을 포함한 전체 기록 기준입니다.
- waypoint 비행 중 고도 2m를 넘긴 기체가 실제 고도 0.3m 이하가 되면 접촉
  가능성을 기록하고 이후 표본을 제외합니다. 접촉 힘 기반 추락 검출기는 아닙니다.
- 약 20Hz로 기록하며 위치는 NWU, quaternion은 wxyz 순서의 body FLU → world NWU입니다.
  화면 좌표는 `[-W,U,N]`, 자세 보간은 최단 경로 SLERP, 위치 보간은 선형입니다.
  기록 밖으로 외삽하거나 0.5초보다 긴 누락 구간을 보간하지 않습니다.
- `truth_sim_s = MAVLink boot_s + boot_offset_s`이며 기본 오프셋은 0입니다.
  펌웨어를 변경하면 `--boot-offset-s`를 검증해야 합니다. 케이스별 시간을
  임의로 이동하지 않고 절대 시뮬레이션 시간 및 공통 유효 구간으로 비교합니다.
- 통계는 표본 가중이며 불규칙한 외부 기록을 시간 가중하지 않습니다. 없는 값은
  0으로 대체하지 않습니다. 실제 주입 관측·전체 공격 구간 기록 여부·비행 완료를
  각각 별도로 보고합니다.

각 실행에 시나리오·초기 파라미터·격리된 FastDyn 설정·truth/telemetry/명령 CSV·
이벤트·미션/결과 JSON·MAVLink tlog·실행 로그를 보존합니다. `run.json`은
펌웨어/FMU/소스 해시와 좌표·시간 규칙을 기록하며, 내보낸 데이터에는 해당 분석에
사용한 소스 해시도 포함합니다.

## 검증과 선택적 동영상 출력

```bash
PHYSICS_DEMO_TEST_BUILD=out/physics_analysis/waypoint-attacks/build \
  fastdyn-env/bin/python -m pytest demos/physics_analysis/tests -q
node --test demos/physics_analysis/web/math.test.js
# 실행 중인 서버와 선택적 동영상 의존성 필요
fastdyn-env/bin/python -m pip install -e 'demos/physics_analysis[video]'
fastdyn-env/bin/python -m playwright install chromium
fastdyn-env/bin/python demos/physics_analysis/tests/browser_check.py \
  --output out/physics_analysis/browser-check.png
# ffmpeg도 설치되어 있어야 합니다.
fastdyn-env/bin/python demos/physics_analysis/demo.py video \
  --start 160 --end 165 --fps 24 --output out/physics_analysis/flight.mp4
```

[실제 다섯 비행 검증 결과](VALIDATION.md)와 [출처·라이선스](CREDITS.md)를
참고하세요. 실행 검증 통과는 공격 차단이나 항법 정확성 유지의 증명이 아닙니다.
