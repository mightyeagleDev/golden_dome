import cv2
import numpy as np
import serial.tools.list_ports
import pyautogui
import time
from pid import PID 



frame_interval = 0.02
last_frame_time = 0
prev_p_x = 0
prev_p_y = 0 
ser= None
frame_width ,frame_height  = None , None
center = 90
servo_position_x = None
servo_position_y = None
left_edge = 5
right_edge = 175
k=1
pid_x = PID(kp=0.03, ki=0.0, kd=0.006, output_min=-10, output_max=10)
pid_y = PID(kp=0.03, ki=0.0, kd=0.006, output_min=-10, output_max=10)



def clamp(min_, max_, n):
    return max(min_, min(max_, n))


def isInRange(val , min = 5 , max = 175):
    if val>=min and val<=max:
        return True
    return False


DEADBAND = 2  
def calculate_movement_pid(cX, cY, frame_center_x, frame_center_y, dt = frame_interval):

    error_x = frame_center_x - cX
    error_y = frame_center_y - cY

    cmd_x = pid_x.update(error_x, dt)
    cmd_y = pid_y.update(error_y, dt)

    return cmd_x, cmd_y

SMOOTH = 1

    
def move_servo(cmd_x,cmd_y,ser):
    global servo_position_x,servo_position_y

    servo_new_position_x = servo_position_x + (cmd_x * SMOOTH)
    servo_new_position_y = servo_position_y + (cmd_y * SMOOTH)
    if not isInRange(servo_new_position_x) :##or not isInRange(servo_new_position_y):
        print("Object out of range")
        return
    
    serial_write(ser,servo_new_position_x , servo_new_position_y)
    servo_position_x=servo_new_position_x
    servo_position_y=servo_new_position_y
    



def move_to_center(ser):
    global servo_position_x , servo_position_y
    serial_write( ser,90 , 90)
    servo_position_x = servo_position_y = 90

def serial_write(ser ,point_x ,point_y  ):
    global prev_p_x
    global prev_p_y    
    if point_x != prev_p_x or  point_y != prev_p_y:
     
     
        #print(map_the_input(mouse_point[0]))
        #point_bytes = (str(map_the_input(point_x ,1)) +"x"+str(map_the_input(point_y ,2))+"\n").encode()
        point_bytes = (str(point_x) +"x"+str(point_y)+"\n").encode()
        print(point_bytes)
        try:
            (ser.write(point_bytes))  
            ser.flush() 
        except Exception as e:
            print(e)
            
        
        
            try:
                ser.close()
                ser.open()
            except:
                    pass
        
        
        prev_p_x =point_x
        prev_p_y =point_y


def serial_init():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        try:
            ser = serial.Serial(p.device, 9600, timeout=1)
            print("Connected:", p.device)
            return ser
        except:
            pass
    print("No serial device found.")
    return None

    

def map_the_input(input , i):
    if i == 1:
        output = 180 -int((input/frame_width)*180)
        return output
    else:
        output = 180 - int((input/frame_height)*180)
        return output



def main():
    global last_frame_time
    global frame_height
    global frame_width

    ser = serial_init()
    move_to_center(ser)
    
    if ser ==None:
        return
 
    url = "http://10.81.205.144/stream"
    cap = cv2.VideoCapture(1)

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return
    #cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    #cap.set(cv2.CAP_PROP_EXPOSURE, -6)
    #cap.set(cv2.CAP_PROP_BRIGHTNESS, 50)


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
        frame_height, frame_width = frame.shape[:2]

        
  
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])

       
        lower_red2 = np.array([175, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        
        mask =  mask2+mask1

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
                    frame_center_y= int(frame_height/2)
                    frame_center_x =  int(frame_width/2)
                    
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    
                    cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
                    
                    
                    text = f"Center: ({cX}, {cY})"
                    cv2.putText(frame, text, (cX - 20, cY - 20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    
                    cv2.line(frame , (cX,cY),( frame_center_x,frame_center_y)   , (0,255,0) , 2)
                    
                    #serial_write(ser , cX,cY )
                    cmd_x , cmd_y = calculate_movement_pid(cX,cY ,frame_center_x,frame_center_y)
                    move_servo(cmd_x , cmd_y , ser)

                    
        cv2.imshow("Red Object Detection", frame)
        # cv2.imshow("Mask", mask) # Uncomment to see what the computer 'sees'

       
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release resources
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()



