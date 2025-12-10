import serial
import pyautogui
import time

for i in range(20):
    try:
          ser = serial.Serial(
                port=f'COM{i}', 
                baudrate=9600,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1 
            )
          break
    except: 
         print("No device in port ",i ,".")

print("device found in port" , ser.port)

width , height = pyautogui.size()
def map_the_input(input , i):
    if i == 1:
        output = 180 -int((input/width)*180)
        return output
    else:
        output = 180 - int((input/height)*180)
        return output

prev_p_x = 0
prev_p_y = 0 
while True:
    mouse_point_x= pyautogui.position()[0]
    mouse_point_y= pyautogui.position()[1]
    time.sleep(0.05)
    
    if mouse_point_x != prev_p_x or  mouse_point_y != prev_p_y:
     
     
        #print(map_the_input(mouse_point[0]))
        mouse_point_bytes = (str(map_the_input(mouse_point_x ,1)) +"x"+str(map_the_input(mouse_point_y ,2))+"\n").encode()
        print(mouse_point_bytes)
        try:
            (ser.write(mouse_point_bytes))  
            ser.flush() 
        except Exception as e:
            print(e)
            
        
        
            try:
                ser.close()
                ser.open()
            except:
                    pass
        
        time.sleep(0.05)
        prev_p_x =mouse_point_x
        prev_p_y =mouse_point_y


    
    


