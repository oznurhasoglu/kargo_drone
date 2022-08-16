from dronekit import connect, VehicleMode, LocationGlobalRelative
#from pymavlink import mavutil
import time
#servo için
import RPi.GPIO as GPIO
#görüntü işleme için
import cv2
import mediapipe

#RP VE PİXHAWK BAĞLANTISI
connection_string="/dev/ttyACM0"    
iha=connect(connection_string,wait_ready=True,baud= 115200, timeout=100)    
print("bağlantı sağlandı")

drawingModule = mediapipe.solutions.drawing_utils
handsModule = mediapipe.solutions.hands

kontrol=0
iha.airspeed=3

GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
servo1 = GPIO.PWM(11, 50)
servo2 = GPIO.PWM(12, 50)

#DİREKT BELİRLENEN KOORDİNATA GİT
hedef_lat= 38.819057
hedef_lon= 35.443039
hedef_alt= 1

while True:
    if (iha.mode=="AUTO"):
        hedef = LocationGlobalRelative(hedef_lat,hedef_lon,4)
        iha.simple_goto(hedef)
        time.sleep(50)
        hedef2 = LocationGlobalRelative(iha.location.global_relative_frame.lat,iha.location.global_relative_frame.lon,1)
        iha.simple_goto(hedef2)
        time.sleep(10)
print("iha konuma geldi.")

try:
    cap = cv2.VideoCapture(0)
    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
    with handsModule.Hands(static_image_mode=False, min_detection_confidence=0.7, min_tracking_confidence=0.7, max_num_hands=2) as hands:
        while True:
            ret, frame = cap.read()
            flipped = cv2.flip(frame, flipCode = 1)
            frame1 = cv2.resize(flipped, (640, 480))
            results = hands.process(cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB))
            if results.multi_hand_landmarks != None:
                for handLandmarks in results.multi_hand_landmarks:
                    drawingModule.draw_landmarks(frame1, handLandmarks, handsModule.HAND_CONNECTIONS)
                    
                    x, y = handLandmarks.landmark[0].x, handLandmarks.landmark[0].y
                    x1, y1 = handLandmarks.landmark[4].x, handLandmarks.landmark[4].y
                    if (y1 <= y):
                        print("servo calisir")
                        #cv2.putText(frame1, "servolar calisiyor", (10, 50), font, 4, (0, 0, 0), 3)
                        servo1.start(2) 
                        servo2.start(2)
                        time.sleep(4)
                        kontrol+=1
            cv2.imshow("Frame", frame1); #kaldır
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or kontrol == 1:
                break 
        cap.release()
        cv2.destroyAllWindows()
except:
    print("hata aldım")
    print("servo calisir")
    servo1.start(2) 
    servo2.start(2)
print("görev bitti, eve dön.")
iha.mode=VehicleMode("RTL")

