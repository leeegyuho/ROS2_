# DetectedObject.msg
string label          # 객체 클래스 이름 (e.g., "vehicle", "bigvehicle","bike", "human", "animal","obstacle")
float32 distance      # 추정 거리 (미터)
string zone           # 객체가 속한 ROI 구역 ("red", "yellow", "none")
int32 x_min           # 바운딩 박스 x_min
int32 y_min           # 바운딩 박스 y_min
int32 width           # 바운딩 박스 폭
int32 height          # 바운딩 박스 높이