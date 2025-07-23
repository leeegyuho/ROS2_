import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ros_interfaces.msg import ProcessingData, DetectedObject # 💡 메시지 임포트 경로 변경
from cv_bridge import CvBridge
import cv2
import numpy as np

class UIDisplayNode(Node):
    def __init__(self):
        super().__init__('ui_display_node')
        self.bridge = CvBridge()

        self.image_subscription = self.create_subscription(
            Image,
            '/processing/visualized_image',
            self.image_callback,
            10
        )

        self.data_subscription = self.create_subscription(
            ProcessingData,
            '/processing/data_output',
            self.data_callback,
            10
        )

        self.latest_processed_data = None
        self.get_logger().info('UI Display Node가 시작되었습니다.')

        cv2.namedWindow("Client View", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Client View", 640, 480)

    def data_callback(self, msg: ProcessingData):
        self.latest_processed_data = msg

    def image_callback(self, img_msg: Image):
        if self.latest_processed_data is None:
            self.get_logger().warn("알림 레벨 데이터가 아직 수신되지 않았습니다. 대기 중...")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

            alert_text = self.latest_processed_data.alert_level
            alert_color = (255, 255, 255)
            if alert_text == "WARNING":
                alert_color = (0, 255, 255)
            elif alert_text == "DANGER":
                alert_color = (0, 0, 255)

            if alert_text != "NONE":
                font_scale = 1.5
                thickness = 3
                text_size, baseline = cv2.getTextSize(alert_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)

                text_x = int((cv_image.shape[1] - text_size[0]) / 2)
                text_y = text_size[1] + 20

                cv2.putText(cv_image, alert_text, (text_x + 2, text_y + 2), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), thickness + 2)
                cv2.putText(cv_image, alert_text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, alert_color, thickness)

            current_y_offset = 70
            line_height = 75
            box_width = 250
            box_x = 0

            

            for i, obj in enumerate(self.latest_processed_data.detected_objects):
                label = obj.label
                dist = obj.distance
                zone = obj.zone

                y_top = current_y_offset + i * line_height
                x_center = box_x + box_width // 2
                
                
                if zone == "red":
                    triangle_pts = np.array([
                        [x_center, y_top + 10],
                        [box_x + 10, y_top + line_height - 10],
                        [box_x + box_width - 10, y_top + line_height - 10]
                    ], np.int32)
                    cv2.fillPoly(cv_image, [triangle_pts], color=(0, 0, 255))

                    danger_text = "DANGER"
                    dz_size, _ = cv2.getTextSize(danger_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    dz_x = x_center - dz_size[0] // 2
                    dz_y = triangle_pts[0][1] - 8
                    cv2.putText(cv_image, danger_text, (dz_x, dz_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                    content_text = f"{label} {dist:.2f}m"
                    ct_size, _ = cv2.getTextSize(content_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    ct_x = x_center - ct_size[0] // 2
                    ct_y = triangle_pts[2][1] - 10
                    cv2.putText(cv_image, content_text, (ct_x, ct_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                elif zone == "yellow":
                    y_center_diamond = y_top + line_height // 2
                    diamond_pts = np.array([
                        [x_center, y_top + 10],
                        [box_x + box_width - 10, y_center_diamond],
                        [x_center, y_top + line_height - 10],
                        [box_x + 10, y_center_diamond]
                    ], np.int32)
                    cv2.fillPoly(cv_image, [diamond_pts], color=(0, 255, 255))

                    warn_text = "WARNING"
                    wz_size, _ = cv2.getTextSize(warn_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    wz_x = x_center - wz_size[0] // 2
                    wz_y = diamond_pts[0][1] - 8
                    cv2.putText(cv_image, warn_text, (wz_x, wz_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

                    content_text = f"{label} {dist:.2f}m"
                    ct_size, _ = cv2.getTextSize(content_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    ct_x = x_center - ct_size[0] // 2
                    ct_y = y_center_diamond + ct_size[1] // 2
                    cv2.putText(cv_image, content_text, (ct_x, ct_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
                    

                    

            cv2.imshow("Client View", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"UI Node 이미지 처리 오류: {e}")

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UIDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()