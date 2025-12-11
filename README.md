# Pure Pursuit Controller (Namgyu)

## Modified Files
주요 수정 파일 위치

| 구분 | 파일 경로 | 설명 |
| :--- | :--- | :--- |
| **Logic** | `race/src/pure_pursuit/src/pure_pursuit.cpp` | Lookahead distance 계산 및 조향각 제어 로직 구현 |
| **Config** | `race/src/pure_pursuit/config/pure_pursuit.yaml` | 주행 속도, 게인 값 등 핵심 파라미터 설정 |

---

## How to Run
터미널 2개를 사용하여 cbr, sis로 빌드와 환경설정 완료 후

### Terminal 1: Simulator 실행
~/race$ ros2 launch racecar_simulator simulator.launch.py 

### Terminal 2: 주행 노드 실행
~/race$ ros2 run pure_pursuit pure_pursuit   --ros-args --params-file ~/race/src/pure_pursuit/config/pure_pursuit.yaml

<br/>
결과확인하면서 파라미터 수정해서 최대한 시간단축하고 벽에 안닿게 수정

수정파일_race/src/pure_puresuit/config/pure_pursuit.yaml(namgyu) & race/src/pure_puresuit/src/pure_pursuit.cpp
