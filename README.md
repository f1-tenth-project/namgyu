# Advanced Pure Pursuit & Obstacle Avoidance (Namgyu)

## Key Features

1.  **Dynamic Pure Pursuit:**
    * 차량의 속도와 경로의 곡률(Curvature)에 따라 `Lookahead Distance`를 동적으로 조절
2.  **Physics-based Speed Control:**
    * 경로의 곡률을 미리 분석하여(Preview), 횡방향 가속도($a_y$) 한계 내에서 **최대 진입 속도**를 계산
3.  **Robust Obstacle Detection:**
    * **RANSAC Algorithm:** LiDAR 데이터에서 트랙 벽(Wall)을 직선으로 추정하여 제거
    * **Clustering:** 벽이 제거된 데이터에서 장애물(Obstacle) 군집만을 정확히 추출
4.  **Active Avoidance (FTG + PID):**
    * 장애물이 감지되면 경로 추종 조향각에 FTG 기반의 회피 조향각을 PID로 합성하여 부드럽게 회피

---

## Topics

| Type | Topic Name | Description |
| :--- | :--- | :--- |
| **Sub** | `/scan0` | 장애물 및 벽 인식을 위한 LiDAR 데이터 |
| **Sub** | `/odom0` | 차량 위치 및 속도 (Odometry) |
| **Sub** | `/center_path` | 글로벌 레퍼런스 경로 (Raceline) |
| **Pub** | `/ackermann_cmd0` | 최종 계산된 조향 및 가속도 명령 |
| **Pub** | `/pp_debug/walls` | RANSAC으로 검출된 벽 시각화 (Rviz) |
| **Pub** | `/obstacle_clusters` | 인식된 장애물 군집 시각화 (Rviz) |

---

## Parameters Guide

정밀한 튜닝을 위해 다양한 파라미터를 제공며 `config/pure_pursuit.yaml`에서 수정 가능

### 1. Driving Performance (주행 성능)
| Parameter | Default | Description |
| :--- | :--- | :--- |
| `speed_max` | **20.0** | 직선 구간 목표 최대 속도 (m/s) |
| `ay_max` | **3.0** | 코너링 시 허용할 최대 횡가속도 (높을수록 코너를 빠르게 돎) |
| `lookahead` | **0.8** | 기본 전방 주시 거리 (m) |
| `k_accel` | **2.5** | 가속도 제어 P-gain |

### 2. Obstacle & Wall Detection (인식)
| Parameter | Default | Description |
| :--- | :--- | :--- |
| `wall_ransac_iters` | **80** | 벽 검출을 위한 RANSAC 반복 횟수 (높을수록 정확하나 연산량 증가) |
| `wall_inlier_thresh` | **0.07** | 벽으로 판단할 직선과의 거리 오차 허용 범위 (m) |
| `obs_cluster_dist` | **0.35** | 점(Point)들이 하나의 장애물로 묶이는 거리 기준 (m) |
| `track_wall_margin` | **0.4** | 트랙 벽 근처의 데이터를 무시할 마진 (m) |

### 3. Avoidance Strategy (회피)
| Parameter | Default | Description |
| :--- | :--- | :--- |
| `obstacle_slow_dist` | **5.0** | 장애물 감지 시 감속을 시작하는 거리 (m) |
| `steer_ftg_max` | **0.7** | 회피 시 허용할 최대 조향각 (rad) |
| `steer_lpf_alpha` | **0.6** | 조향값 스무딩 필터 계수 (0.0~1.0, 클수록 최신값 반영) |

---

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

<img width="399" height="432" alt="스크린샷 2025-12-11 230157" src="https://github.com/user-attachments/assets/f251fd54-b70b-4fb8-964d-8fb496d3baf1" />
