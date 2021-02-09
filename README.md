# vip_gui

**catkin workspace 새로 만들거나 기존 사용중인 workspace 사용해도 무방**  
```
cd catkin_workspace/src
git clone https://github.com/kka-na/vip_gui.git
catkin_make
```

**파일 경로 수정**  
rviz_array.py, d2v.py 코드에서 경로 수정 ( /home/kanakim/catkin_ws/src/gui 부분들 전부 )


**소스 파일**  
icon folder : GUI에 사용하는 아이콘 이미지 모음  
map folder : 송도 캠퍼스 맵 csv 파일 모음  
d2v.py : GUI및 노드 subscribe 하여 display 하기 위한 코드  
d2v.ui : GUI 파일  
default.rviz : 송도 맵 rviz 파일   
rviz에서 추가하는 visuaslizer들이 추가되어있음.   
-	사용하지 않는 파일 (삭제해도 무방)  
map2 folder   
array_marker.py  
poset_test.py  
test.rviz  


**실행방법**  
```
terminal #1 ) roscore
```
```
terminal #2 ) python rviz_array.py   
```
rviz 2D NAV GOAL사용해서 목적지 찍으면 정보 확인 가능   
```
terminal #3) python d2v.py  
```
GUI가 열림.   
Ready : widget 초기화, rviz 및 node 세팅  
Start : 모든 node 들로부터 subscribe  
+) map 만 띄우고 확인하고싶으면 rviz 실행 후 default.rviz 열면 됨.   

**실행 화면**
[![VIP – D2V GUI test](http://img.youtube.com/vi/m_2QwbOrvU0/0.jpg)](https://youtu.be/m_2QwbOrvU0?t=0s) 

