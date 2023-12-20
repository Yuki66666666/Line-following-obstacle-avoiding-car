import cv2
import numpy as np

cap = cv2.VideoCapture(0)
IMG_WIDTH = 640
IMG_HEIGHT = 480
MIN_AREA = 1000  # 最小三角形面積，可以根據需要調整

while True:
    ret, img = cap.read()
    if not ret:
        break

    img = cv2.resize(img, (IMG_WIDTH, IMG_HEIGHT))
    rows_to_keep = int(IMG_HEIGHT * 0.7)
    img = img[IMG_HEIGHT-rows_to_keep:, :]
    
    # 顏色篩選範圍
    LB = np.array([0, 0, 0])
    UB = np.array([180, 55, 81])
    kernelOpen = np.ones((5, 5))
    kernelClose = np.ones((20, 20))
    
    mask = cv2.inRange(img, LB, UB)
    maskOpen = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernelOpen)
    maskClose = cv2.morphologyEx(maskOpen, cv2.MORPH_CLOSE, kernelClose)

    # 邊緣檢測
    edges = cv2.Canny(maskClose, 30, 100)

    # 找尋輪廓
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        # 輪廓近似
        epsilon = 0.02 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        # 檢查是不是三角形
        if len(approx) == 3:
            # 計算三角形面積
            area = cv2.contourArea(cnt)
            if area > MIN_AREA:
                # 在原始圖像上畫出三角形
                cv2.drawContours(img, [approx], 0, (0, 0, 0), 2)  # 黑色輪廓

                # 頂點坐標
                points = approx.reshape(-1, 2)
                
                # 在每個頂點畫上彩色圓圈
                cv2.circle(img, tuple(points[0]), 5, (0, 0, 255), -1)  # 紅色
                cv2.circle(img, tuple(points[1]), 5, (0, 255, 255), -1)  # 黃色
                cv2.circle(img, tuple(points[2]), 5, (255, 0, 0), -1)  # 藍色

                # 根據 Y 值排序頂點
                points = sorted(points, key=lambda p: p[1])

                # 找出中間頂點並比較 X 值
                middle_point = points[1]
                if middle_point[0] < points[0][0] or middle_point[0] < points[2][0]:
                    print("三角形頂點在左邊")
                else:
                    print("三角形頂點在右邊")

    # 顯示圖像
    cv2.imshow('img with Triangles', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()