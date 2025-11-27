# namgyu


프로그래밍 프로젝트 전체 파일
race/src/pure_pursuit/config의 pure_pursuit.yaml에서 파라미터 수정 후

다른 터미널 두개 준비, 빌드와 환경설정 마친 후

터미널 1
~/race$ ros2 launch racecar_simulator simulator.launch.py 

터미널 2
~/race$ ros2 run pure_pursuit pure_pursuit   --ros-args --params-file ~/race/src/pure_pursuit/config/pure_pursuit.yaml

결과확인하면서 파라미터 수정해서 최대한 시간단축하고 벽에 안닿게 수정

수정파일_race/src/pure_puresuit/config/pure_pursuit.yaml(namgyu) & race/src/pure_puresuit/src/pure_pursuit.cpp
