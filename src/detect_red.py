import cv2
import numpy as np
#import serial
import pyautogui
import time

frame_interval = 0.05
last_frame_time = 0
prev_p_x = 0
prev_p_y = 0 
ser = None
frame_width, frame_height = None, None
'''def serial_write(ser, point_x, point_y):
    global prev_p_x
    global prev_p_y

    if point_x != prev_p_x or point_y != prev_p_y:
        point_bytes = (str(map_the_input(point_x, 1)) + "x" + str(map_the_input(point_y, 2)) + "\n").encode()
        print(point_bytes)
        try:
            ser.write(point_bytes)
            ser.flush() 
        except Exception as e:
            print(e)
            try:
                ser.close()
                ser.open()
            except:
                pass
        
        prev_p_x = point_x
        prev_p_y = point_y

def serial_init():
    for i in range(20):
        try:
            ser_ = serial.Serial(
                port=f'COM{i}', 
                baudrate=9600,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1 
            )
            print("device found in port", ser_.port)
            return ser_
        except Exception as e: 
            print("No device in port ", i, ".", e)
            
    return None

def map_the_input(input_val, i):
    if i == 1:
        output = 180 - int((input_val / frame_width) * 180)
        return output
    else:
        output = 180 - int((input_val / frame_height) * 180)
        return output
'''
def main():
    global last_frame_time
    global frame_height
    global frame_width
    #ser = serial_init()
    # if ser == None:
    #    return
 
    url = "http://10.48.134.26:8080/video"
    cap = cv2.VideoCapture(2, cv2.CAP_V4L2)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
    # Increase brightness & contrast




    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    print("Press 'q' to exit the program.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame.")
            break
            
        current_time = time.time()
        if current_time - last_frame_time < frame_interval:
            continue
        last_frame_time = current_time
        

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_width, frame_height, _ = frame.shape
        print(frame_height, frame_width)
# Original
        lower_red1 = np.array([0, 120, 10])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([175, 120, 10])
        upper_red2 = np.array([180, 255, 255])

# Low-light tuning
        '''lower_red1 = np.array([0, 80, 50])      # lower S and V
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([170, 80, 50])
        upper_red2 = np.array([180, 255, 255])
'''

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        
        mask = mask2 + mask1

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)

            if cv2.contourArea(largest_contour) > 200:
                x, y, w, h = cv2.boundingRect(largest_contour)
                
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                M = cv2.moments(largest_contour)
                
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
                    
                    text = f"Center: ({cX}, {cY})"
                    cv2.putText(frame, text, (cX - 20, cY - 20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    
                    cv2.line(frame, (cX, cY), (int(frame_height/2), int(frame_width/2)), (0, 255, 0), 2)
                    
                    #serial_write(ser, cX, cY)

        cv2.imshow("Red Object Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()