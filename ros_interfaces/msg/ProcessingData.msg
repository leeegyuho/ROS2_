# ProcessingData.msg
std_msgs/Header header

# 이미지 폭/높이 (UI 노드에서 ROI 좌표를 이미지 크기에 맞춰 해석할 수 있도록)
int32 image_width
int32 image_height

# 최종 경고 레벨: "NONE", "WARNING", "DANGER"
string alert_level

# 감지된 객체 정보 리스트
DetectedObject[] detected_objects

# Warning/Danger ROI 다각형 정보 (시각화용)
# UI 노드에서 별도로 ROI를 그릴 필요가 없다면 이 필드는 제거 가능
# 하지만, 서버 노드에서 그리는 것을 디버깅하고 클라이언트에서 참조할 필요가 있다면 유지
# Warning/Danger ROI가 복잡한 형태일 수 있으므로 x, y 좌표 리스트로 정의
bool danger_roi_available
int32[] danger_roi_x_coords
int32[] danger_roi_y_coords

bool warning_roi_available
int32[] warning_roi_x_coords
int32[] warning_roi_y_coords