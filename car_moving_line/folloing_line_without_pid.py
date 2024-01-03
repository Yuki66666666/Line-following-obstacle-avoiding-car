import cv2
import numpy as np
import time
import serial

def extract_polygon(img, slice_num=16):
    IMG_HEIGHT, IMG_WIDTH, _ = img.shape
    X_DIV = int(IMG_HEIGHT / float(slice_num))
    
    avg_centroids = []
    all_contours = []
    centroids = []
    left = []
    right = []
    img = cv2.blur(img, (5, 5)) 
    for i in range(slice_num):
        sliced_img = img[X_DIV * i:X_DIV * (i + 1), :]
        gray = cv2.cvtColor(sliced_img, cv2.COLOR_BGR2GRAY)
        _, thresholded = cv2.threshold(gray, 43, 255, cv2.THRESH_BINARY_INV)  # 阈值可能需要调整

        conts, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(conts) >= 2:
            sorted_conts = sorted(conts, key=cv2.contourArea, reverse=True)[:2]
            centroids = []
            for c in sorted_conts:
                M = cv2.moments(c)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00']) + X_DIV * i
                    centroids.append((cx, cy))
            
            if len(centroids) == 2:
                avg_cx = int((centroids[0][0] + centroids[1][0]) / 2)
                avg_cy = int((centroids[0][1] + centroids[1][1]) / 2)
                avg_centroids.append((avg_cx, avg_cy))
                # 將最大的兩個輪廓添加到列表中
                ccx1 = int((centroids[0][0]))
                ccy1 = int((centroids[1][1]))
                ccx2 = int((centroids[1][0]))
                ccy2 = int((centroids[0][1]))
                if (ccx1 < 320) and (ccx2 > 320):
                    left.append((ccx1, ccy1))
                    right.append((ccx2, ccy2))
                elif (ccx1 > 320) and (ccx2 < 320):
                    right.append((ccx1, ccy1))
                    left.append((ccx2, ccy2))
        elif len(conts) == 1:
            centroids = []
            for c in conts:
                M = cv2.moments(c)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00']) + X_DIV * i
                    centroids.append((cx, cy))
                if len(centroids) == 1:
                # 將最大的兩個輪廓添加到列表中
                    ccx1 = int((centroids[0][0]))
                    ccy1 = int((centroids[0][1]))
                    left.append((ccx1, ccy1))
                    

    return avg_centroids, left, right




def detect_direction(path):
    if not path:
        return "No line detected"
    #get_mid = (path[-1][0] + path[0][0]) / 2  # Change in x
    get_mid = sum(point[0] for point in path) / len(path)
    #dy = (path[-1][1] + path[0][1]) / 2  # Change in y
    print(get_mid)
    if get_mid == 0:  # Prevent division by zero
        return "dont move"  # or handle vertical line case 
    elif get_mid >= 350: #get_mid >= 340:
        return "right turn"
    elif get_mid <= 290: #get_mid <= 300:
        return "left turn"
    else:
        return "go straight"
    
def detect_direction_one_line(path):
    x1, y1 = path[0]
    x2, y2 = path[-1]
    if y1 > y2:
        tmpx = x1
        tmpy = y1
        x1 = x2
        y1 = y2
        x2 = tmpx
        y2 = tmpy
        
    if x2 == x1:  # 避免除以零的情况
        return "No turn" 
    else:
        slope = (y2 - y1) / (x2 - x1)
        if slope > 0:
            #return "right turn"
            return "left turn"
        else:
            #return "left turn"
            return "right turn"

    
    
cap = cv2.VideoCapture(0)
IMG_WIDTH = 640
IMG_HEIGHT = 480
ser = serial.Serial('/dev/ttyUSB0', 57600)
ser.flush()

time.sleep(1)
count = 0
prev_direction = ""
while True:
    
    # 延时一小段时间，以避免过于频繁的读取
    print (ser.name)
    input_char = 'd'
    if ser.in_waiting < 0:
        input_char = 'd'
    elif ser.in_waiting > 0:
        print("ser inwaiting")
        #input_char = ser.read().decode('utf-8')
        #input_char = ser.read()
        input_char = ser.read().decode('utf-8')
        print(input_char)
        #print("Read input: " + input_char)
    
    ret, img = cap.read()
    if not ret:
        break
    
    img = cv2.resize(img, (IMG_WIDTH, IMG_HEIGHT))
    img2 = img
    get_triangle = 0
    MIN_AREA = 1000
    if (input_char == 'c'):
        get_triangle = 1
    get_triangle_once = 0
    if (get_triangle != 1):
        get_triangle_once = 0
        rows_to_keep = int(IMG_HEIGHT * 0.5)
        #rows_to_keep = int(IMG_HEIGHT * 0.5)
        
        rows_to_keep2 = int(IMG_HEIGHT * 0.8)
        #img = img[:rows_to_keep2, :]
        
        img = img[IMG_HEIGHT-rows_to_keep:, :]
        #avg_path, left, right = extract_polygon(img, 16)
        avg_path, left, right = extract_polygon(img, 8)
        
        if not (avg_path):
            if (left):
                direction = detect_direction_one_line(left)
                print("one line")
                #time.sleep(5)
            else:
                direction = "No line detected"
        elif (avg_path):
            direction = detect_direction(avg_path)
            print("two_lines")
        
        print(direction)
        
        
        #time.sleep(0.15)
        if (direction == "go straight"):
            ser.write(b'G')
            #time.sleep(1)
        elif (direction == "left turn"):
            ser.write(b'L')
            #time.sleep(1)
        elif (direction == "right turn"):
            ser.write(b'R')
            #time.sleep(1)
        elif (direction == "No line detected"):
            if (prev_direction == "go straight"):
                ser.write(b'G')
                #time.sleep(1)
            elif (prev_direction == "left turn"):
                ser.write(b'L')
                #time.sleep(1)
            elif (prev_direction == "right turn"):
                ser.write(b'R')
            #ser.write(b'H')
            #time.sleep(1)
        prev_direction = direction
    else:
        print("collide")
        ser.write(b'H')
        triangle_find = 0
        count = 0
        while(triangle_find == 0 and count <= 20 and get_triangle_once == 0):
            triangle_direction = "left"
            # 顏色篩選範圍
            LB = np.array([0, 0, 0])
            UB = np.array([180, 55, 81])
            kernelOpen = np.ones((5, 5))
            kernelClose = np.ones((20, 20))
            
            mask = cv2.inRange(img2, LB, UB)
            maskOpen = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernelOpen)
            maskClose = cv2.morphologyEx(maskOpen, cv2.MORPH_CLOSE, kernelClose)

            # 邊緣檢測
            edges = cv2.Canny(maskClose, 30, 100)

            # 找尋輪廓
            contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            count = count + 1
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
                        #cv2.drawContours(img, [approx], 0, (0, 0, 0), 2)  # 黑色輪廓

                        # 頂點坐標
                        points = approx.reshape(-1, 2)
                        
                        # 在每個頂點畫上彩色圓圈
                        #cv2.circle(img, tuple(points[0]), 5, (0, 0, 255), -1)  # 紅色
                        #cv2.circle(img, tuple(points[1]), 5, (0, 255, 255), -1)  # 黃色
                        #cv2.circle(img, tuple(points[2]), 5, (255, 0, 0), -1)  # 藍色

                        # 根據 Y 值排序頂點
                        points = sorted(points, key=lambda p: p[1])

                        # 找出中間頂點並比較 X 值
                        middle_point = points[1]
                        if middle_point[0] < points[0][0] or middle_point[0] < points[2][0]:
                            print("三角形頂點在左邊")
                        else:
                            print("三角形頂點在右邊")
                            triangle_direction = "right"
                            
                        if triangle_direction == "left" and get_triangle_once != 1:
                            ser.write(b'L')
                            get_triangle_once = 1
                        elif triangle_direction == "right" and get_triangle_once != 1:
                            ser.write(b'R')
                            get_triangle_once = 1
                        triangle_find = 1
                        time.sleep(1.6)
        get_triangle_once = 1
           
                        
                        

    
# 繪製計算出的平均中心點的路徑
    if __debug__:
        for i in range(len(avg_path) - 1):
            cv2.line(img, avg_path[i], avg_path[i + 1], (0, 255, 0), 5)
        for i in range(len(left) - 1):
            cv2.line(img, left[i], left[i + 1], (255, 0, 0), 5)
        for i in range(len(right) - 1):
            cv2.line(img, right[i], right[i + 1], (0, 0, 255), 5)
    
    # 繪製輪廓
    
    # 顯示方向
    cv2.putText(img, direction, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

    cv2.imshow("cam", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
ser.close()
cap.release()
cv2.destroyAllWindows()


