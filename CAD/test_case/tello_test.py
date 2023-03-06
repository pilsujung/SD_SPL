import socket #소켓 통신을 위한 라이브러리
import threading #멀티 스레딩을 위한 라이브러리
import tkinter #GUI용 라이브러리
import time
import numpy as np #H.264 영상 규격 처리 보조를 위한 수치계산 라이브러리
import cv2
import sys
# import opencv.YOLO as YOLO
sys.path.append("D:\\LUC\\jupyter\\2022_software_Development\\SoftwareDevelopment2022\\CAD\\Decoder\\h264_36")
import h264decoder
from PIL import Image, ImageTk
import platform
# import torch

import time
import sys

#Tello와 상호작용하기 위한 클래스
class Tello:

    def __init__(self):
        print("[Tello] 시작")
        
        
        #1) 클래스 변수 선언
        
        #텔로의 IP주소, cmd용 port
        self.tello_address = ('192.168.10.1',8889) 
        
        #ToF 센서의 값 (mm)
        self.tof = 8190 
        
        #Tello가 이동할 기본 거리(mm)
        self.distance = 400  
        
        #Tello가 회전할 기본 각도(degree)
        self.degree = 30  
        
        #tof 측정값이 tof_threshold (mm) 이하인 경우 forward 명령 무시
        self.tof_threshold = 800 
        
        #send용 lock
        self.lock = threading.Lock()
        
        #
        self.panel = None
        
        #영상 frame
        self.frame = None  # numpy array BGR -- current camera output frame
        
        #영상 frame 디코딩을 위한 디코더
        self.decoder = h264decoder.H264Decoder() #H.264 동영상 스트림 디코딩을 위한 객체
        
        #Tello가 이륙했는지
        self.is_takeoff = False
        
        # self.model = self.load_model()
        # self.model.conf = 0.4 # set inference threshold at 0.3
        # self.model.iou = 0.3 # set inference IOU threshold at 0.3
        # self.model.classes = [0] # set model to only detect "Person" class
        # self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        
        self.YOLO_net = cv2.dnn.readNet("CAD/ObjectDetector/yolov2-tiny.weights", "CAD/ObjectDetector/yolov2-tiny.cfg")

        # YOLO NETWORK 재구성
        self.classes = []  #객체 이름을 저장하는 배열

        with open("CAD/ObjectDetector/yolo.names", "r") as f:   # 객체 이름들이 저장된 'yolo.names' open
            self.classes = [line.strip() for line in f.readlines()]   # yolo.names 파일

        self.layer_names = self.YOLO_net.getLayerNames()

        self.output_layers = [self.layer_names[i[0] - 1] for i in self.YOLO_net.getUnconnectedOutLayers()]
  
        
        

        #2) GUI화면 설정
        self.root = tkinter.Tk()  # GUI 화면 객체 생성
        self.root.wm_title("TELLO Controller_TEST") #GUI 화면의 title 설정  
        self.root.wm_protocol("WM_DELETE_WINDOW", self.onClose) #창을 나갈 경우에 대한 실행사항

        #화면에 문구 설정
        self.text0 = tkinter.Label(self.root, text= "ToF: {} mm".format(self.tof), font='Helvetica 10 bold')
        self.text0.pack(side='top')

        self.text1 = tkinter.Label(self.root, justify="left", text="""
        W - Move Tello Up\t\t\tArrow Up - Move Tello Forward
        S - Move Tello Down\t\t\tArrow Down - Move Tello Backward
        A - Rotate Tello Counter-Clockwise\t\tArrow Left - Move Tello Left
        D - Rotate Tello Clockwise\t\tArrow Right - Move Tello Right
        """)
        self.text1.pack(side="top")

        #착륙 버튼
        self.btn_landing = tkinter.Button(self.root, text="Land", relief="raised", command=self.land)
        self.btn_landing.pack(side="bottom", fill="both", expand="yes", padx=10, pady=5)

        #이륙 버튼
        self.btn_takeoff = tkinter.Button(self.root, text="Takeoff", relief="raised", command=self.takeoff)
        self.btn_takeoff.pack(side="bottom", fill="both", expand="yes", padx=10, pady=5)

        #키보드 버튼들과 Tello 동작을 바인딩
        self.tmp_f = tkinter.Frame(self.root, width=100, height=2)
        self.tmp_f.bind('<KeyPress-w>', self.on_keypress_w)
        self.tmp_f.bind('<KeyPress-s>', self.on_keypress_s)
        self.tmp_f.bind('<KeyPress-a>', self.on_keypress_a)
        self.tmp_f.bind('<KeyPress-d>', self.on_keypress_d)
        self.tmp_f.bind('<KeyPress-Up>', self.on_keypress_up)
        self.tmp_f.bind('<KeyPress-Down>', self.on_keypress_down)
        self.tmp_f.bind('<KeyPress-Left>', self.on_keypress_left)
        self.tmp_f.bind('<KeyPress-Right>', self.on_keypress_right)
        self.tmp_f.pack(side="bottom")
        self.tmp_f.focus_set()

        # #이동거리를 변경할 바
        # self.distance_bar = tkinter.Scale(self.root, from_=0.2, to=5, tickinterval=0.1, digits=3, label='Distance(m)', resolution=0.01)
        # self.distance_bar.set(self.distance)
        # self.distance_bar.pack(side="left")

        # #각도를 변경할 바
        # self.degree_bar = tkinter.Scale(self.root, from_=1, to=360, tickinterval=10, label='Degree')
        # self.degree_bar.set(self.degree)
        # self.degree_bar.pack(side="right")
        
        # #이동거리를 적용하는 버튼
        # self.btn_distance = tkinter.Button(self.root, text="Distance 적용", relief="raised",command=self.updateDistancebar)
        # self.btn_distance.pack(side="left", fill="both", expand="yes", padx=10, pady=5)
        
        # #각도를 적용하는 버튼
        # self.btn_distance = tkinter.Button(self.root, text="Degree 적용", relief="raised", command=self.updateDegreebar)
        # self.btn_distance.pack(side="right", fill="both", expand="yes", padx=10, pady=5)
        
        #3) 종료를 처리할 이벤트 생성
        self.stop_event = threading.Event()
        
        #4) 스레드 생성 및 실행
        self.thread_connect = threading.Thread(target=self._thread_connect, daemon=True)
        
        self.thread_tof = threading.Thread(target=self._thread_tof, daemon=True)
        self.get_GUI_Image_thread = threading.Thread(target = self._getGUIImage, daemon=True)
        self.receive_video_thread = threading.Thread(target=self._receive_video_thread, daemon=True)
        # self.run_yolo_thread = threading.Thread(target=self._run_yolo_thread,daemon=True)
        
        # self.thread_connect.start()
        self.socket_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # IPv4, UDP 통신 소켓 객체를 생성(command용)
        self.socket_cmd.bind(('', 8889)) #소켓 객체를 텔로와 바인딩(8889 포트)

        self.socket_state = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # IPv4, UDP 통신 소켓 객체를 생성(state용)
        self.socket_state.bind(('',8890)) #소켓 객체를 텔로와 바인딩(8890 포트)
        
        self.socket_camera = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # IPv4, UDP 통신 소켓 객체를 생성(camera용)
        self.socket_camera.bind(('', 11111)) #소켓 객체를 텔로와 바인딩(11111 포트)
        
        self.send_command('command')   #SDK mode에 진입하도록 command 명령어를 전송
        self.send_command('streamon')  #동영상 스트림을 보내오도록 streamon 명령어를 전송
        self.send_command('speed 100') #Tello의 속도를 최대로 지정
        
        
        print("[Tello] 소켓 연결 완료")
        self.thread_tof.start() #tof값을 받아오는 스레드
        self.receive_video_thread.start() #frame을 저장하는 스레드
        self.get_GUI_Image_thread.start() #이미지 편집 + 화면 이미지를 갱신하는 스레드
        
        print("[Tello] GUI 시작")
        self.root.mainloop()
        
        
        
        
#스레드에서 실행될 함수============================================================================
    
    #Tello와의 소켓을 생성하는 함수
    def _thread_connect(self):
        self.socket_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # IPv4, UDP 통신 소켓 객체를 생성(command용)
        self.socket_cmd.bind(('', 8889)) #소켓 객체를 텔로와 바인딩(8889 포트)

        self.socket_state = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # IPv4, UDP 통신 소켓 객체를 생성(state용)
        self.socket_state.bind(('',8890)) #소켓 객체를 텔로와 바인딩(8890 포트)
        
        self.socket_camera = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # IPv4, UDP 통신 소켓 객체를 생성(camera용)
        self.socket_camera.bind(('', 11111)) #소켓 객체를 텔로와 바인딩(11111 포트)
        
        self.send_command('command')   #SDK mode에 진입하도록 command 명령어를 전송
        self.send_command('streamon')  #동영상 스트림을 보내오도록 streamon 명령어를 전송
        self.send_command('speed 100') #Tello의 속도를 최대로 지정
        
        
        print("[Tello] 소켓 연결 완료")
        self.thread_tof.start() #tof값을 받아오는 스레드
        self.receive_video_thread.start() #frame을 저장하는 스레드
        self.get_GUI_Image_thread.start() #이미지 편집 + 화면 이미지를 갱신하는 스레드
        # self.run_yolo_thread.start()
    
    #ToF 값을 업데이트하는 함수
    def _thread_tof(self):
        print("[Tello] ToF 시작")
        while not self.stop_event.is_set():
            try:
                return_msg = self.send_command('EXT tof?')
                if return_msg[0]!="e": 
                    self.tof = int(return_msg[4:])
                else: 
                    self.tof = 8190
                    
                tmp = self.tof_threshold - self.tof #mm
                
                if self.is_takeoff and tmp>0:
                    if tmp<200: tmp = 200
                    self.move('back',tmp)
                self.text0.config(text = "ToF: {} m".format(self.tof))
            except Exception as e:
                print("[TOF]",e, "/[Error msg]: ", return_msg)
                
            time.sleep(0.1)


    #1) Tello가 보낸 영상 frame을 받아서, self.frame에 저장하는 스레드
    def _receive_video_thread(self):
        
        #TEST
        packet_data = bytes()
        
        #TEST1 START
        self.__decoder = h264decoder.H264Decoder()
        packet_data = bytes()
        while not self.stop_event.is_set():
            res_string = self.socket_camera.recv(2048)
            packet_data += res_string
            
            if len(res_string) != 1460: # frame의 끝이 아니면,
                for frame in self.h264_decode(packet_data): 
                    self.frame = frame
                packet_data = bytes()
                    

            
        #TEST
        
        
        #ORIGIN
        # print("[Tello] receive_video_thread 시작")
        # err_cnt = 0 #에러 발생을 카운트하는 변수 (에러가 10번 이상 발생 시 함수 종료)
        # packet_data = bytes()
        
        # while not self.stop_event.is_set():
        #     try:
        #         err_cnt = 0
        #         res_string = self.socket_camera.recv(2048)
        #         packet_data += res_string
                
        #         if len(res_string) != 1460: # frame의 끝이 아니면,
        #             for frame in self.h264_decode(packet_data): 
        #                 self.frame = frame

        #             packet_data = bytes()

        #     except Exception as e:
        #         err_cnt += 1
        #         print ("[tello] tello_receive_video_thread socket.error 발생: {} (err_cnt: {})".format(e,err_cnt))
                
        #         if err_cnt == 5: break
        #ORIGIN

        print("[tello] tello_run: receive_video_thread 종료")
    

    #frame을 디코딩하는 함수
    def h264_decode(self, packet_data):
        # H.264: 2003년에 발표된 동영상 표준 규격으로, 텔로에서도 사용
        # packet_data: raw H.264 data에 대한 배열

        res_frame_list = []
        frames = self.decoder.decode(packet_data) #입력받은 raw H.264 data 배열을 디코딩

        for framedata in frames: #framedata는 4개 요소의 튜플로 구성
            frame, width, height, linesize = framedata

            if frame is not None:
                frame = np.fromstring(frame, dtype=np.ubyte, count=len(frame), sep='') #UTF-8 인코딩을 통해 문자열을 바이트로 변환
                frame = frame.reshape((height, int(linesize / 3), 3)) #바이트 배열을 화면 크기에 맞게 변환
                frame = frame[:, :width, :]
                res_frame_list.append(frame) #frame을 변환 후 res_frame_list에 추가

        return res_frame_list

    
    
    def _getGUIImage(self):

        system = platform.system()  #현재 운영체제 확인
        
        try:
            while True: #종료 상태가 아니면,
                frame = self.frame
                if frame is None or frame.size == 0: 
                    continue #만약 받아온 frame이 없다면 pass
                
                h, w, c = frame.shape

                # YOLO 입력
                blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0),True, crop=False)
                self.YOLO_net.setInput(blob)
                outs = self.YOLO_net.forward(self.output_layers)

                class_ids = []
                confidences = []
                boxes = []

                for out in outs:

                    for detection in out:

                        scores = detection[5:]
                        class_id = np.argmax(scores)
                        confidence = scores[class_id]

                        if confidence > 0.5:
                            # Object detected
                            center_x = int(detection[0] * w)
                            center_y = int(detection[1] * h)
                            dw = int(detection[2] * w)
                            dh = int(detection[3] * h)
                            # Rectangle coordinate
                            x = int(center_x - dw / 2)
                            y = int(center_y - dh / 2)
                            boxes.append([x, y, dw, dh])
                            confidences.append(float(confidence))
                            class_ids.append(class_id)

                indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.45, 0.4)


                for i in range(len(boxes)):
                    if i in indexes:
                        x, y, w, h = boxes[i]
                        label = str(self.classes[class_ids[i]])
                        score = confidences[i]

                        # 경계상자와 클래스 정보 이미지에 입력
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 5)
                        cv2.putText(frame, label, (x, y - 20), cv2.FONT_ITALIC, 0.5, (255, 255, 255), 1)
                
                image = Image.fromarray(frame)  # frame을 이미지로 변환 

                if system =="Windows" or system =="Linux": 
                    self._updateGUIImage(image)
                    
                else: #mac OS의 경우 에러가 발생하는 것이 확인되어, 이를 방지하기 위해 새로운 thread로 실행
                    thread_tmp = threading.Thread(target=self._updateGUIImage,args=(image,))
                    thread_tmp.start()
                    time.sleep(0.03)    

        except Exception as e: print("[telloUI] _getGUIImage RuntimeError 발생: {}".format(e))
        print("[telloUI] _getGUIImage thread 종료")                    
    
    # #화면을 새 frame으로 갱신하는 함수
    def _updateGUIImage(self,image):
        image = ImageTk.PhotoImage(image) #이미지를 imagetk 형식으로 변환

        if self.panel is None: #panel이 없으면 생성
            self.panel = tkinter.Label(image=image)
            self.panel.image = image
            self.panel.pack(side="left", padx=10, pady=10)
        
        else:
            self.panel.configure(image=image)
            self.panel.image = image



#기타 함수=========================================================================================
    
    #Tello에게 command를 전송하는 함수
    def send_command(self, command):
        self.lock.acquire()
        response = None
        try:
            #Tello에게 command를 전송
            self.socket_cmd.sendto(command.encode('utf-8'), self.tello_address)
            response = (self.socket_cmd.recv(1024)).decode('utf-8')
            if response[0] != "t":
                print("[Tello] return: {} / ORIGIN: {}".format(response,command))
            
        except Exception as e:print(e)
            
        self.lock.release()
        
        return response

    #GUI 종료시 실행되는 함수
    def onClose(self):
        try:
            self.stop_event.set() #종료를 알리는 버튼을 켜기
            self.root.quit() #화면 종료
            self.land() #텔로 착륙
            self.socket_cmd.close() #command용 소켓 닫기
            self.socket_state.close() #state용 소켓 닫기
            self.socket_camera.close() #camera용 소켓 닫기
        except: pass
        print("[Tello] 종료")
        exit()


    # #이동거리 적용을 위한 함수
    # def updateDistancebar(self):
    #     self.distance = self.distance_bar.get()


    # #각도 적용을 위한 함수
    # def updateDegreebar(self):
    #     self.degree = self.degree_bar.get()



#Tello 명령어 송신 함수======================================================================================

    def set_speed(self, speed):
        #speed: 10~100 cm/s
        speed = int(round(speed))
        return self.send_command('speed {}'.format(speed))


    def land(self): #return: Tello의 receive 'OK' or 'FALSE'
        self.is_takeoff = False
        return self.send_command('land')


    def takeoff(self): #return: Tello의 receive 'OK' or 'FALSE'
        self.is_takeoff = True
        return self.send_command('takeoff')


    def move(self, direction, distance): #return: Tello의 receive 'OK' or 'FALSE'
        # distance: 20 ~ 500 centimeters.
        # direction: 'up, down, forward', 'back', 'right', 'left'
        # Metric: .02 to 5 meters

        send_distance = int(round(distance)/10) #cm

        return self.send_command("{} {}".format(direction, send_distance))
    

    def rotate_cw(self, degrees): #return: Tello의 receive 'OK' or 'FALSE'
        return self.send_command('cw {}'.format(degrees))


    def rotate_ccw(self, degrees): #return: Tello의 receive 'OK' or 'FALSE'
        return self.send_command('ccw {}'.format(degrees))
    


#Tello의 동작에 대한 키보드 링크 함수=============================================================================================== 

    def on_keypress_w(self, event):
        print("[telloUI] on_keypress_w: up {} m".format(self.distance))
        self.move('up',self.distance)

    def on_keypress_s(self, event):
        print("[telloUI] on_keypress_s: down {} m".format(self.distance))
        self.move('down',self.distance)

    def on_keypress_a(self, event):
        print("[telloUI] on_keypress_a: ccw {} degree".format(self.degree))
        self.rotate_ccw(self.degree)

    def on_keypress_d(self, event):
        print("[telloUI] on_keypress_d: cw {} degree".format(self.degree))
        self.rotate_cw(self.degree)

    def on_keypress_up(self, event):
        print("[Tello] tof: {}".format(self.tof))
        tmp = self.tof - self.tof_threshold #tmp: 뒤로 이동해야 할 거리
        if tmp>0: #뒤로 이동해야 할 거리가 존재
            if tmp> self.distance:
                print("[telloUI] on_keypress_up: forward {} m".format(self.distance))
                self.move('forward',self.distance)
            elif tmp>200:
                print("[telloUI] on_keypress_up: forward {} m".format(tmp))
                self.move('forward',tmp)
            else:
                print("[telloUI] 장애물과의 거리가 가깝습니다. 명령을 거부합니다.")
        else:
            print("[telloUI] 장애물과의 거리가 가깝습니다. 명령을 거부합니다.")

    def on_keypress_down(self, event):
        print("[telloUI] on_keypress_down: backward {} m".format(self.distance))
        self.move('back',self.distance)

    def on_keypress_left(self, event):
        print("[telloUI] on_keypress_left: left {} m".format(self.distance))
        self.move('left',self.distance)

    def on_keypress_right(self, event):
        print("[telloUI] on_keypress_right: right {} m".format(self.distance))
        self.move('right',self.distance)
        
        


#YOLO========================================================================================================
    # def load_model(self):
    #     """
    #     Function loads the yolo5 model from PyTorch Hub.
    #     """
    #     model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
    #     return model

    # def score_frame(self, frame):
    #     """
    #     function scores each frame of the video and returns results.
    #     :param frame: frame to be infered.
    #     :return: labels and coordinates of objects found.
    #     """
    #     self.model.to(self.device)
    #     results = self.model([frame])
    #     labels, cord = results.xyxyn[0][:, -1].to('cpu').numpy(), results.xyxyn[0][:, :-1].to('cpu').numpy()
    #     return labels, cord

    # def plot_boxes(self, results, frame):
    #     """
    #     plots boxes and labels on frame.
    #     :param results: inferences made by model
    #     :param frame: frame on which to  make the plots
    #     :return: new frame with boxes and labels plotted.
    #     """
    #     labels, cord = results
    #     n = len(labels)
    #     x_shape, y_shape = frame.shape[1], frame.shape[0]
    #     for i in range(n):
    #         row = cord[i]
    #         x1, y1, x2, y2 = int(row[0]*x_shape), int(row[1]*y_shape), int(row[2]*x_shape), int(row[3]*y_shape)
    #         bgr = (0, 0, 255)
    #         cv2.rectangle(frame, (x1, y1), (x2, y2), bgr, 1)
    #         label = f"{int(row[4]*100)}"
    #         cv2.putText(frame, label, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
    #         cv2.putText(frame, f"Total Targets: {n}", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    #     return frame


#실행부=======================================================================================================  
tello = Tello()