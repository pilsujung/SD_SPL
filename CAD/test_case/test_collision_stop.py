import socket #소켓 통신을 위한 라이브러리
import threading #멀티 스레딩을 위한 라이브러리
import tkinter #GUI용 라이브러리
import time



#Tello와 상호작용하기 위한 클래스
class Tello:

#클래스 생성 시 실행할 동작 (magic method)========================================================================
    def __init__(self):
        
        print("[Tello] 시작")
        
        #1) 클래스 변수 생성
        self.tello_address = ('192.168.10.1',8889) #텔로에게 접속했을 때, 텔로의 IP주소
        self.tof = 8190 #ToF 센서의 값 (mm)
        self.distance = 0.5  #Tello를 키보드로 이동 시, 이동할 기본 거리(초기값)
        self.degree = 30  #Tello를 회전 시, 회전할 기본 각도(초기값)
        self.tof_threshold = 1000 # tof 측정값이 tof_threshold mm 이하인 경우 forward 명령 무시
        self.lock = threading.Lock()
        
        
        #2) GUI화면 설정,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
        self.root = tkinter.Tk()  # 화면 객체 생성
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

        #이동거리를 변경할 바
        self.distance_bar = tkinter.Scale(self.root, from_=0.2, to=5, tickinterval=0.1, digits=3, label='Distance(m)', resolution=0.01)
        self.distance_bar.set(self.distance)
        self.distance_bar.pack(side="left")

        #각도를 변경할 바
        self.degree_bar = tkinter.Scale(self.root, from_=1, to=360, tickinterval=10, label='Degree')
        self.degree_bar.set(self.degree)
        self.degree_bar.pack(side="right")
        
        #이동거리를 적용하는 버튼
        self.btn_distance = tkinter.Button(self.root, text="Distance 적용", relief="raised",command=self.updateDistancebar)
        self.btn_distance.pack(side="left", fill="both", expand="yes", padx=10, pady=5)
        
        #각도를 적용하는 버튼
        self.btn_distance = tkinter.Button(self.root, text="Degree 적용", relief="raised", command=self.updateDegreebar)
        self.btn_distance.pack(side="right", fill="both", expand="yes", padx=10, pady=5)
        
        
        
        
        #3) 종료를 처리할 이벤트 생성
        self.stop_event = threading.Event()
        
        
        #4) 스레드 생성 및 실행
        self.thread_connect = threading.Thread(target=self._thread_connect, daemon=True)
        self.thread_tof = threading.Thread(target=self._thread_tof, daemon=True)
        self.thread_connect.start()
        
        print("[Tello] GUI 시작")
        self.root.mainloop()
        
        
        
        
#스레드에서 실행될 함수===========================================================================
    
    #Tello와의 소켓을 생성하는 함수
    def _thread_connect(self):
        self.socket_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # IPv4, UDP 통신 소켓 객체를 생성(command용)
        self.socket_cmd.bind(('', 8889)) #소켓 객체를 텔로와 바인딩(8889 포트)

        self.socket_state = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # IPv4, UDP 통신 소켓 객체를 생성(state용)
        self.socket_state.bind(('',8890)) #소켓 객체를 텔로와 바인딩(8890 포트)
        
        self.socket_camera = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # IPv4, UDP 통신 소켓 객체를 생성(camera용)
        self.socket_camera.bind(('', 11111)) #소켓 객체를 텔로와 바인딩(11111 포트)
        
        self.send_command('command') # SDK mode에 진입하도록 command 명령어를 전송
        self.send_command('speed 100')
        
        print("[Tello] 소켓 연결 완료")
        self.thread_tof.start()
    
    
    #ToF 값을 업데이트하는 함수
    def _thread_tof(self):
        print("[Tello] ToF 시작")
        while not self.stop_event.is_set():
            try:
                self.tof = int(self.send_command('EXT tof?')[4:])
                self.text0.config(text = "ToF: {} m".format(self.tof))
            except Exception as e:
                print(e)
                
            time.sleep(0.5)

    
    
#기타 함수========================================================================================
    
    #Tello에게 command를 전송하는 함수
    def send_command(self, command):
        
        self.lock.acquire()
        response = None
        try:
            #Tello에게 command를 전송
            self.socket_cmd.sendto(command.encode('utf-8'), self.tello_address)
            response = (self.socket_cmd.recv(1024)).decode('utf-8')
            if response[0] != "t":
                print("[Tello] return: {}".format(response))
            
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


    #이동거리 적용을 위한 함수
    def updateDistancebar(self):
        self.distance = self.distance_bar.get()


    #각도 적용을 위한 함수
    def updateDegreebar(self):
        self.degree = self.degree_bar.get()



#Tello 명령어 송신 함수======================================================================================

    def set_speed(self, speed):
        #speed: 10~100 cm/s
        speed = int(round(speed))
        return self.send_command('speed {}'.format(speed))


    def land(self): #return: Tello의 receive 'OK' or 'FALSE'
        return self.send_command('land')


    def takeoff(self): #return: Tello의 receive 'OK' or 'FALSE'
        return self.send_command('takeoff')


    def move(self, direction, distance): #return: Tello의 receive 'OK' or 'FALSE'
        # distance: 20 ~ 500 centimeters.
        # direction: 'up, down, forward', 'back', 'right', 'left'
        # Metric: .02 to 5 meters

        distance = int(round(distance * 100))

        return self.send_command("{} {}".format(direction, distance))
    

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
        if self.tof > self.tof_threshold:
            print("[telloUI] on_keypress_up: forward {} m".format(self.distance))
            self.move('forward',self.distance)
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
        
        
        
#실행부=======================================================================================================  
tello = Tello()