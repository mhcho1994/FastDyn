# ArduRover Rehosting Setup

## FMU rover mission and trajectory plots

현재 `configs/rover462.toml`은 부팅 후 health check만 실행합니다. 사각형
웨이포인트를 실제로 주행하고 궤적을 저장하려면 FastDyn 루트에서 실행합니다.

```bash
source fastdyn-env/bin/activate
# C 런타임 수정 후 한 번 실행
ninja -C build
bash utils/run_rover_trajectory.sh
```

실행마다 `out/rover-trajectory/<UTC timestamp>/`에 `mission.tlog`,
`console.log`, 사용한 설정/미션/파라미터 사본, `trajectory.csv`,
`trajectory.json`, `trajectory_2d.{png,svg,pdf}`, `trajectory_3d.{png,svg,pdf}`를
저장합니다. 완료 판정은 마지막 `MISSION_ITEM_REACHED` 수신입니다.
`MISSION_CURRENT`가 마지막 항목으로 바뀐 것만으로 완료 처리하지 않습니다.
기본 미션은 약 16 m × 17 m 사각형이며 목표 속도는 2 m/s입니다.
기본 MAVLink 포트를 공유하므로 다른 차량 시뮬레이션과 동시에 실행하지 않습니다.

특정 구간을 다시 그리려면 실제 실행 폴더를 지정합니다. 예를 들어 30–60초는
아래와 같으며, 90초 이상 기록한 미션에서는 `--end 90`을 사용할 수 있습니다.

```bash
run_dir=out/rover-trajectory/<UTC-timestamp>
python utils/plot_rover_trajectory.py \
  --log "$run_dir/mission.tlog" \
  --mission "$run_dir/mission.waypoints" \
  --output "$run_dir/trajectory_30_60" \
  --start 30 --end 60 --require-complete
```

시간은 **armed AUTO 진입 후 첫 위치 샘플을 0초로 한 펌웨어 시뮬레이션 시간**입니다.
그래프는 `GLOBAL_POSITION_INT`의 위치 추정값을 사용하며, 미션 홈 좌표에서
North/East 미터 좌표로 변환합니다. 파란색은 궤적, 초록색은 구간 시작점,
빨간색은 종료점, 검은색과 번호는 미션 웨이포인트입니다.
지상 rover이므로 3D 그래프의 높이는 거의 일정합니다. 높이는 AUTO 직전
2초간의 정지 고도 중앙값을 빼서 표시합니다. `relative_alt`가 약 149 m로
보고되는 실행도 있지만, 이는 고도 기준점 문제이며 지상고 149 m를 뜻하지 않습니다.
CSV에는 원래의 절대/상대 고도와 변환한 높이를 모두 보존합니다.

FMU 런타임은 모델에 선언된 PWM 초기값을 사용하고, 비활성 채널의 0 PWM도
그 초기값으로 처리합니다. Rover의 중립값은 1500이고 Copter는 1000입니다.
FMU의 FRD 자력계에는 Gazebo용 FLU→FRD 변환을 중복 적용하지 않습니다.
IMU FIFO는 가상 시간 기준 8 kHz 샘플 수와 요청된 읽기 길이를 사용하며,
rover의 `AP_InertialSensor::wait_for_sample()` 대기를 유지해 자세 적분 시간을 맞춥니다.
ChibiOS의 `stGetCounter()`도 동일한 1 MHz 가상 마이크로초 카운터를 사용합니다.

## Legacy Gazebo setup

This guide covers the legacy ArduRover/Courbet Gazebo setup. The maintained default FastDyn path uses the FMU/Rumoca backend; use this flow when you specifically need the older Gazebo backend with `LIBGZ=true` and `LIBHW=true`.

## Docker Build

The easiest way to use the legacy ArduRover/Courbet backend is via Docker. A `Dockerfile` is provided that automatically handles all QEMU, Gazebo Harmonic, Rust, and `libhw` dependencies.

To build the image, run this from your **parent workspace directory** (the folder containing `FastDyn/`, `qemu/`, etc.):

```bash
cd ..
docker build -f FastDyn/Dockerfile -t fastdyn-env .
```

Then run the container interactively:

```bash
docker run -it fastdyn-env
```

You will drop into a shell inside `/workspace/FastDyn` with the environment ready. **Note: All phases below (Static Analysis, Probe Run, Trace Analyze, etc.) should be run inside this container!**

## Local Build Instructions

### Prerequisites
The trace analysis pipeline requires `universal-ctags` or `exuberant-ctags` for fallback source extraction. Please install it using your package manager (e.g., `sudo apt-get install universal-ctags`).

Gazebo Harmonic is required. If the Gazebo pkg-config modules (`gz-transport13`, `gz-msgs10`, `protobuf`) are not available, install them via:
```bash
bash virtuals/physics/flight_controllers/courbet/utils/install_gazebo_harmonic.sh
```
`LIBHW=true` also requires a buildable `libhw` checkout and `libstlink-dev`.

### Local Build
To initialize submodules, build Gazebo dependencies, and build FastDyn, run from the FastDyn root:
```bash
source ./setup.sh --clone-qemu --build-qemu --build-gazebo --skip-optifuzz
```
Omit `--clone-qemu` when QEMU sources are already present. Omit both QEMU
options if the patched QEMU is already built and needs no rebuild.

Then, build the `libhw` repo, by first running the setup file in libhw folder
```bash
cd ../libhw
make
```

Then, to build FastDyn correctly for this legacy backend, run:
```bash
cd ../Fastdyn
make PROBE=true DEV=true LIBHW=true LIBGZ=true FLIGHT_CONTROLLERS=true DEBUG_PRINT=true LIBFUZZ=true
```

## 1. Static Analysis Phase

Before running certain analyses or traces, execute the static analysis frontend to extract binary metadata, SVD structures, source maps, and callgraphs.

Run the static analyzer using your configuration (make sure to update the paths in the `configs/automatic_ardurover462.toml`):
```bash
fastdyn static-analyze -c configs/automatic_ardurover462.toml -s third_party/common/cmsis-svd-data --force
```
This extracts the analysis artifacts into the cache directory specified in your TOML configuration under `[Rehosting.directories]`.

## 2. Probe Run Phase

This phase runs the binary on QEMU using the FastDyn plugin and consumes artifacts from the static analysis phase (e.g., `probe_faults.json`).

Run the command using your configuration:
```bash
fastdyn probe-run -c configs/automatic_ardurover462.toml -o fastdyn_recent_run
```
This will generate the artifacts under the `fastdyn_recent_run`

## 3. Trace Analyze Phase

After a probe run completes (or hangs), the trace analyzer parses the execution artifacts (`io.log`, `BBL_counts`) and correlates them with the static analysis data to produce a structured, deterministic LLM prompt. It avoids running QEMU or directly calling the LLM.

Run the command:
```bash
fastdyn trace-analyze -c configs/automatic_ardurover462.toml -o fastdyn_work --latest-run-dir fastdyn_recent_run
```
This will generate `fastdyn_work/prompt.txt` containing the hardware configuration, stateful analysis, tight loops, and full source context, ready to be passed to an LLM.

If an LLM later writes `fastdyn_work/routing.json` and recommends creating new model files or adding TOML routing entries, rerun trace analysis with:
```bash
fastdyn trace-analyze -c configs/automatic_ardurover462.toml -o fastdyn_work --latest-run-dir fastdyn_recent_run --apply-routing
```
That mode asks before applying each file/config change, then generates the routed implementation prompt.

## 4. LLM Prompt Phase

After `trace-analyze` creates `fastdyn_work/prompt.txt`, send that prompt to the configured LLM provider:
```bash
fastdyn llm -d fastdyn_work --compile --model gpt-5.4 --reasoning-effort medium --evaluate
```

The `llm` command reads `fastdyn_work/prompt.txt`, saves the prompt and response under `fastdyn_llm_history/`, and auto-detects the target model file from `fastdyn_work/analysis.json` when `-o/--output` is omitted. For known-device implementation prompts, it expects SEARCH/REPLACE output and patches the detected model file. With `--compile`, it builds the BoardRunner SDK model after applying the patch. With `--evaluate`, it appends per-call metrics to `fastdyn_llm_history/metrics.jsonl`.

If the LLM returns a VETO or routing-only JSON response, `fastdyn llm` writes `fastdyn_work/routing.json` but does not create files or edit TOML. Rerun trace analysis to consume that routing decision:
```bash
fastdyn trace-analyze -c configs/automatic_ardurover462.toml -o fastdyn_work --latest-run-dir fastdyn_recent_run
```

If that routing decision requires new files or TOML entries, rerun with `--apply-routing` to review and apply those changes interactively before generating the next implementation prompt.
