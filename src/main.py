import cv2
import numpy as np
import serial
import time

class KalmanFilter:
    def __init__(self, Q=0.01, R=1, initial_value=0):
        """
        Q = process noise (how much we trust model)
        R = measurement noise (how noisy sensor is)
        """
        self.Q = Q
        self.R = R

        self.x = initial_value   # filtered value (estimate)
        self.P = 1.0             # estimation error covariance

    def update(self, measurement):
        # Prediction step
        self.P = self.P + self.Q

        # Kalman Gain
        K = self.P / (self.P + self.R)

        # Update estimate with measurement
        self.x = self.x + K * (measurement - self.x)

        # Update error covariance
        self.P = (1 - K) * self.P

        return self.x

    def set_noise(self, Q, R):
        self.Q = Q
        self.R = R

    def reset(self, value=0):
        self.x = value
        self.P = 1.0

class PID:
    def __init__(self, kp, ki, kd, output_min=0, output_max=180):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.output_min = output_min
        self.output_max = output_max

        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self,error):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0:
            dt = 1e-6  # avoid zero division

        

        # Integral term
        self.integral += error * dt

        # Derivative term
        derivative = (error - self.prev_error) / dt

        # PID output
        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        # Clamp output
        if output > self.output_max:
            output = self.output_max
        elif output < self.output_min:
            output = self.output_min

        # Save state
        self.prev_error = error
        self.last_time = now

        return output

    def set_tunings(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
    
    def reset(self):
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()



frame_interval = 0.02  # ~20 FPS
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
k=2
pidx = PID(kp=0.8, ki=0, kd=0.01, output_min=0, output_max=180)
pidy = PID(kp=0.8, ki=0, kd=0.01, output_min=0, output_max=180)
kfx = KalmanFilter(Q=0.1, R=0.1)
kfy = KalmanFilter(Q=0.1, R=0.1)

def clamp(min_ , max_ , n):
    if n<0:
       n = max(min_ , n)
    elif n>0:
       n= min(n ,max_)
    return  n

def isInRange(val , min = 5 , max = 175):
    if val>=min and val<=max:
        return True
    return False


def calculate_movemonet(cX,cY ,frame_center_x,frame_center_y):
    x_shift = -(frame_center_x-cX )/50
    y_shift =-( frame_center_y-cY)/50
    
    x_sign = 1
    y_sign =1
    if x_shift<0:x_sign=-1
    if y_shift<0:y_sign=-1
    #cmd_x = x_shift/90 * k
    cmd_x = pidx.compute(abs(x_shift))*x_sign
    
    cmd_x = clamp(-5,5,cmd_x)
    cmd_y = pidy.compute(abs(y_shift)) *y_sign
    cmd_y = clamp(-5,5,cmd_y)

    return (cmd_x) , (cmd_y)


    
def move_servo(cmd_x,cmd_y,ser , shoot):
    global servo_position_x,servo_position_y

    servo_new_position_x = servo_position_x +cmd_x
    servo_new_position_y = servo_position_y-cmd_y
    if not isInRange(servo_new_position_x) or not isInRange(servo_new_position_y+5):
        print("Object out of range")
        return
    
    serial_write(ser,servo_new_position_x , servo_new_position_y , shoot)
    servo_position_x=servo_new_position_x
    servo_position_y=servo_new_position_y
    



def move_to_center(ser):
    global servo_position_x , servo_position_y
    serial_write( ser,90 , 90,False)
    servo_position_x = servo_position_y = 90

def serial_write(ser ,point_x ,point_y  ,shoot):
    global prev_p_x
    global prev_p_y    
    if point_x != prev_p_x or  point_y != prev_p_y:
     
     
        #print(map_the_input(mouse_point[0]))
        i='l'
        #point_bytes = (str(map_the_input(point_x ,1)) +"x"+str(map_the_input(point_y ,2))+"\n").encode()
        if shoot:
                i = 's'
        point_bytes = (str(point_x) +"x"+str(point_y)+i+"\n").encode()
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
    
    for i in range(20):
        try:
            ser_ = serial.Serial(
                    port=f'/dev/ttyACM{i}', 
                    baudrate=9600,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=1 
                )
            
            print("device found in port" , ser_.port)  
            return ser_
        except Exception as e : 
            print("No device in port ",i ,"." ,e)
            
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
 
    url = "http://10.48.134.26:8080/video"
    cap = cv2.VideoCapture(2, cv2.CAP_V4L2)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 840)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))

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
        frame_height,frame_width ,_ = frame.shape
        
  
        lower_red1 = np.array([0, 120, 10])
        upper_red1 = np.array([10, 255, 255])

       
        lower_red2 = np.array([175, 120, 10])
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

   
            if cv2.contourArea(largest_contour) > 1000:
               
                x, y, w, h = cv2.boundingRect(largest_contour)
                
                
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                
                M = cv2.moments(largest_contour)
                
                if M["m00"] != 0:
                    frame_center_y= int(frame_height/2)
                    frame_center_x =  int(frame_width/2)
                    
                    cX = (M["m10"] / M["m00"])
                    cY = (M["m01"] / M["m00"])
                    shift_X = frame_center_x- cX
                    shift_y = frame_center_y-cY
                    if abs(shift_X)<15:
                        cX= (kfx.update(cX))
                    if abs(shift_y)<15:
                        cY=(kfy.update(cY))
                    cX=int(cX)
                    cY=int(cY)
                    
                    cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
                    
                    
                    text = f"Center: ({cX}, {cY})"
                    cv2.putText(frame, text, (cX - 20, cY - 20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    
                    cv2.line(frame , (cX,cY),( frame_center_x,frame_center_y)   , (0,255,0) , 2)
                    
                    shoot = False
                    if( y <= (frame_center_y)and frame_center_y <=(y+ h)) and ( x <= (frame_center_x)and frame_center_x <=(x+ w)):
                        shoot =True
                    print(abs((y)), (h))
                    cmd_x , cmd_y = calculate_movemonet(cX,cY ,frame_center_x,frame_center_y)
                    if abs(cmd_x)<0.0025*w:
                        cmd_x=0
             
                        pidx.reset()
                    if abs(cmd_y)<0.0025*h:
                        cmd_y=0
              
                        pidy.reset()
                    
                    move_servo(cmd_x *1.8, cmd_y*1.8 , ser , shoot)

                    
        cv2.imshow("Red Object Detection", frame)
        # cv2.imshow("Mask", mask) # Uncomment to see what the computer 'sees'

       
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release resources
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()



