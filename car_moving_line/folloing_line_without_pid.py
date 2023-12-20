import cv2
import numpy as np
import time
import serial

def extract_polygon(img, slice_num=16, LB=np.array([0,0,0]), UB=np.array([180,55,81])):
    IMG_HEIGHT, IMG_WIDTH, _ = img.shape
    X_DIV = int(IMG_HEIGHT / float(slice_num))
    kernelOpen = np.ones((5, 5))
    kernelClose = np.ones((20, 20))
    
    avg_centroids = []
    # 新增一個列表來保存每個切片中最大的兩個輪廓
    all_contours = []
    centroids = []
    left = []
    right = []
    for i in range(slice_num):
        sliced_img = img[X_DIV * i:X_DIV * (i + 1), :]
        blur = cv2.GaussianBlur(sliced_img, (5, 5), 0)
        imgHSV = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(imgHSV, LB, UB)
        maskOpen = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernelOpen)
        maskClose = cv2.morphologyEx(maskOpen, cv2.MORPH_CLOSE, kernelClose)
        conts, _ = cv2.findContours(maskClose, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
    get_mid = (path[-1][0] + path[0][0]) / 2  # Change in x
    
    #dy = (path[-1][1] + path[0][1]) / 2  # Change in y
    print(get_mid)
    if get_mid == 0:  # Prevent division by zero
        return "dont move"  # or handle vertical line case 
    elif get_mid >= 340:
        return "right turn"
    elif get_mid <= 300:
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
            return "right turn"
        else:
            return "left turn"

    
    
cap = cv2.VideoCapture(0)
IMG_WIDTH = 640
IMG_HEIGHT = 480
ser = serial.Serial('/dev/ttyUSB1', 9600)
time.sleep(1)
count = 0
prev_dir = ""
while True:
    
    ret, img = cap.read()
    if not ret:
        break

    img = cv2.resize(img, (IMG_WIDTH, IMG_HEIGHT))
    rows_to_keep = int(IMG_HEIGHT * 0.7)
    #rows_to_keep = int(IMG_HEIGHT * 0.5)
    img = img[IMG_HEIGHT-rows_to_keep:, :]
    #avg_path, left, right = extract_polygon(img, 16)
    avg_path, left, right = extract_polygon(img, 32)
    
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
    
    if (count == 0):
        count = count + 1
        prev_dir = direction
    elif (direction == prev_dir):
        count = count + 1
    else:
        prev_dir = direction
        count = 1
        
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
        ser.write(b'H')
        #time.sleep(1)

    # 繪製原始的中心點和平均中心點
    
    
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