from PIL import Image
from PIL import ImageTk #이미지 처리를 위한 라이브러리
import tkinter
from tkinter import Toplevel, Scale #GUI용 라이브러리
import threading #멀티 스레딩을 위한 라이브러리
import datetime #현재 날짜를 얻기 위한 라이브러리
import cv2 #opencv를 위한 라이브러리
import os #windows용 cmd 명령을 위한 라이브러리
import time #시간에 대한 라이브러리
import platform #현재 사용중인 시스템 환경에 대한 라이브러리
from tello import Tello


#GUI에 대한 클래스
class TelloUI:


    #클래스 생성 시 실행할 동작 (magic method)
    def __init__(self,tello,outputpath):
    
        #1) 인스턴스 변수 생성
        self.tello = tello # Tello 객체
        self.outputPath = outputpath # 스냅샵을 저장할 디렉터리

        self.frame = None  # h264decoder의 frame을 받아오기 위한 변수
        self.distance = 0.1  # Tello를 키보드로 이동 시, 이동할 기본 거리(초기값)
        self.motion_distance = 0.5 #Tello를 모션으로 이동 시, 이동할 거리
        self.degree = 30  # Tello를 회전 시, 회전할 기본 각도(초기값)
        
        self.pose_mode = False #포즈 인식 모드 설정을 위한 변수
        self.quit_waiting_flag = False #지연시간 확인을 위한 변수
        self.draw_skeleton_flag = False #skeleton을 그렸는지 확인을 위한 변수
        
        self.points = [] #skeleton의 노드 좌표 저장을 위한 리스트
        
        #가능한 모든 skeleton 노드 좌표값        
        self.POSE_PAIRS = [[0,1], [1,2], [2,3], [3,4], [1,5], [5,6], [6,7], [1,14], [14,8], [8,9], [9,10], [14,11], [11,12], [12,13] ]

        
        #2) GUI화면 설정
        self.root = tkinter.Tk()  # 화면 객체 생성
        self.root.wm_title("TELLO Controller") #GUI 화면의 title 설정
        self.root.wm_protocol("WM_DELETE_WINDOW", self.onClose) #창을 나갈 경우에 대한 실행사항
        self.panel = None

        self.btn_snapshot = tkinter.Button(self.root, text="Snapshot!", command=self.takeSnapshot) #snapshot 버튼 생성
        self.btn_snapshot.pack(side="bottom", fill="both", expand="yes", padx=10, pady=5) #snapshot 버튼의 위치 설정

        self.btn_pose = tkinter.Button(self.root, text="Pose Recognition Status: Off", command=self.setPoseMode)
        self.btn_pose.pack(side="bottom", fill="both", expand="yes", padx=10, pady=5)
        
        self.btn_pause = tkinter.Button(self.root, text="Pause", relief="raised", command=self.pauseVideo) #pause 버튼 생성
        self.btn_pause.pack(side="bottom", fill="both", expand="yes", padx=10, pady=5) #pause 버튼의 위치 설정

        self.btn_commandPanel = tkinter.Button(self.root, text="Open Command Panel", relief="raised", command=self.openCmdWindow) #landing 버튼 생성
        self.btn_commandPanel.pack(side="bottom", fill="both", expand="yes", padx=10, pady=5) #landing 버튼의 위치 설정
        
        #3) 종료를 처리할 이벤트 생성
        self.stopEvent = threading.Event()
        
        #4) Tello의 동영상 스트림을 받아올 thread 실행
        self.thread = threading.Thread(target=self._videoLoop)
        self.thread.start()
        
        #5) GUI 이미지를 받아올 thread 선언
        self.get_GUI_Image_thread = threading.Thread(target = self._getGUIImage)
        
        #6) Tello와의 연결을 지속하기 위한 thread 선언
        self.sending_command_thread = threading.Thread(target = self._sendingCommand)
        
        # 참고) thread들의 실행 순서
        # _videoLoop(받아온 프레임 처리) -> _getGUIImage(프레임 받아오기)
        # _autoTakeoff(자동이륙)
        
          
 #=========thread에서 실행할 함수=============================================================================================== 
    
    
    #Tello의 동영상 스트림을 보고, 모션에 따른 동작을 수행하도록 하는 함수(모션인식을 위한 함수)
    def _videoLoop(self):
        
        print("[telloUI] _videoLoop thread 시작")
        time.sleep(0.5)
        self.get_GUI_Image_thread.start()
        self.sending_command_thread.start()
        
        try:
            #종료 버튼이 눌러진 상태가 아니면
            while not self.stopEvent.is_set():                
                
                self.frame = self.tello.read_frame() #frame을 tello로부터 받아오기
                if self.frame is None or self.frame.size == 0: continue #만약 받아온 frame이 없다면 pass

                self.frame = cv2.bilateralFilter(self.frame, 5, 50, 100)  #부드러운 필터 적용
                
                cmd = ''
                self.points.append(None) #구분을 위한 공백 추가
                
                if self.pose_mode: #pose mode가 켜진 상태이면
                    cmd,self.draw_skeleton_flag,self.points = self.my_tello_pose.detect(self.frame)
                    if cmd != "":
                        print("[telloUI] cmd: {}".format(cmd))
                    
                #pose를 읽어서 cmd에 명령이 추가되었다면,
                if cmd == 'moveback': 
                    print("[telloUI] motion recognition: back {} m".format(self.motion_distance))
                    self.tello.move('back',self.motion_distance)
                    
                elif cmd == 'moveforward':
                    print("[telloUI] motion recognition: forward {} m".format(self.motion_distance))
                    self.tello.move('forward',self.motion_distance)
                    
                elif cmd == 'land':
                    print("[telloUI] motion recognition: land")
                    self.tello.land()
                  
        except Exception as e: print("[telloUI] _videoLoop RuntimeError 발생: {}".format(e))
        print("[telloUI] _videoLoop thread 종료")
     
    
    def _sendingCommand(self):
        print("[telloUI] _sendingCommand thread 시작")
        try:
            while not self.stopEvent.is_set():
                self.tello._send_command('command')        
                time.sleep(5)    
        except: pass
        print("[telloUI] _sendingCommand thread 종료")
    
    
    #Tello의 동영상 스트림을 받아오는 함수
    def _getGUIImage(self):

        system = platform.system()  #현재 운영체제 확인
        
        try:
            while not self.stopEvent.is_set(): #종료 상태가 아니면,
                frame = self.tello.read_frame()
                if frame is None or frame.size == 0: continue #만약 받아온 frame이 없다면 pass
                
                if self.pose_mode: #pose mode가 켜진 상태이면
                    # 감지된 skeleton points 그리기
                    for i in range(15):
                        if self.draw_skeleton_flag == True:
                            cv2.circle(frame, self.points[i], 8, (0, 255, 255), thickness=-1, lineType=cv2.FILLED)
                            cv2.putText(frame, "{}".format(i), self.points[i], cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2,lineType=cv2.LINE_AA)       
                    
                    # Skeleton 그리기
                    for pair in self.POSE_PAIRS:
                        partA = pair[0]
                        partB = pair[1]
                        if self.points[partA] and self.points[partB]: #노드가 존재하면 선으로 연결
                            cv2.line(frame, self.points[partA], self.points[partB], (0, 255, 255), 2)
                            cv2.circle(frame, self.points[partA], 8, (0, 0, 255), thickness=-1, lineType=cv2.FILLED)
                
                image = Image.fromarray(frame)  # frame을 이미지로 변환 

                if system =="Windows" or system =="Linux": self._updateGUIImage(image)
                else: #mac OS의 경우 에러가 발생하는 것이 확인되어, 이를 방지하기 위해 새로운 thread로 실행
                    thread_tmp = threading.Thread(target=self._updateGUIImage,args=(image,))
                    thread_tmp.start()
                    time.sleep(0.03)    
        except Exception as e: print("[telloUI] _getGUIImage RuntimeError 발생: {}".format(e))
        print("[telloUI] _getGUIImage thread 종료")
                    
    
    #화면의 이미지를 새 frame 이미지로 갱신하는 함수
    def _updateGUIImage(self,image):
        image = ImageTk.PhotoImage(image) #이미지를 imagetk 형식으로 변환

        if self.panel is None: #panel이 없으면 생성
            self.panel = tkinter.Label(image=image)
            self.panel.image = image
            self.panel.pack(side="left", padx=10, pady=10)
        
        else:
            self.panel.configure(image=image)
            self.panel.image = image
    
    
 #=========Button에 대한 함수=============================================================================================== 

         
    #Button - 스냅샷을 저장하는 함수
    def takeSnapshot(self):

        #저장할 스냅샷의 파일명을 현재시간.jpg로 설정
        filename = "{}.jpg".format(datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))

        #저장할 경로 설정
        p = os.path.sep.join((self.outputPath, filename))

        #파일 저장
        try:
            cv2.imwrite(p, cv2.cvtColor(self.frame, cv2.COLOR_RGB2BGR))
            print("[telloUI] takeSnapshot saved {}".format(filename))
        except:
            print("[telloUI] takeSnapshot ERROR. 스트림을 확인해주세요: frame {}".format(self.frame))


    #Button - 포즈인식 모드를 설정하는 함수 (토글)
    def setPoseMode(self):
        
        if self.pose_mode is False:
            self.pose_mode = True
            self.btn_pose.config(text='Pose Recognition Status: On')
        else:
            self.pose_mode = False
            self.btn_pose.config(text='Pose Recognition Status: Off')
   
   
   #Button - 동영상 출력을 정지하는 함수 (토글)
    def pauseVideo(self):
        
        if self.btn_pause.config('relief')[-1] == 'sunken': #버튼이 눌러진 상태면,
            self.btn_pause.config(relief="raised") #버튼을 올리고
            self.tello.video_freeze(False) #is_freeze 변수를 false로 변경 (동영상 출력 켜기)
            print("[telloUI] pauseVideo 동영상 출력을 시작합니다")
        
        else: #버튼이 올라간 상태면,
            self.btn_pause.config(relief="sunken") #버튼을 누르고 
            self.tello.video_freeze(True) #is_freeze 변수를 true로 변경 (동영상 출력 끄기)
            print("[telloUI] pauseVideo 동영상 출력을 정지합니다")
            
            
    #Button - Tello 조종을 위한 화면을 켜는 함수
    def openCmdWindow(self):

        panel = Toplevel(self.root) #새로 만들 패널을 최상단에 배치
        panel.wm_title("Command Panel")

        # 화면에 문구 설정
        text0 = tkinter.Label(panel, text= "Tello 조종을 위한 화면입니다", font='Helvetica 10 bold')
        text0.pack(side='top')

        text1 = tkinter.Label(panel, text="""W - Move Tello Up                  Arrow Up - Move Tello Forward
                          S - Move Tello Down                   Arrow Down - Move Tello Backward
                          A - Rotate Tello Counter-Clockwise    Arrow Left - Move Tello Left
                          D - Rotate Tello Clockwise            Arrow Right - Move Tello Right""",
                          justify="left")
        text1.pack(side="top")

        #착륙 버튼
        self.btn_landing = tkinter.Button(panel, text="Land", relief="raised", command=self.tello.land)
        self.btn_landing.pack(side="bottom", fill="both", expand="yes", padx=10, pady=5)

        #이륙 버튼
        self.btn_takeoff = tkinter.Button(panel, text="Takeoff", relief="raised", command=self.tello.takeoff)
        self.btn_takeoff.pack(side="bottom", fill="both", expand="yes", padx=10, pady=5)

        #키보드 버튼들과 Tello 동작을 바인딩
        self.tmp_f = tkinter.Frame(panel, width=100, height=2)
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

        #플립에 대한 추가 화면을 여는 버튼
        self.btn_flip = tkinter.Button(panel, text="Flip", relief="raised", command=self.openFlipWindow)
        self.btn_flip.pack(side="bottom", fill="both", expand="yes", padx=10, pady=5)

        #이동거리를 변경할 바
        self.distance_bar = Scale(panel, from_=0.2, to=5, tickinterval=0.1, digits=3, label='Distance(m)', resolution=0.01)
        self.distance_bar.set(0.2)
        self.distance_bar.pack(side="left")

        #각도를 변경할 바
        self.degree_bar = Scale(panel, from_=1, to=360, tickinterval=10, label='Degree')
        self.degree_bar.set(30)
        self.degree_bar.pack(side="right")
        
        #이동거리를 적용하는 버튼
        self.btn_distance = tkinter.Button(panel, text="Distance 적용", relief="raised",command=self.updateDistancebar)
        self.btn_distance.pack(side="left", fill="both", expand="yes", padx=10, pady=5)
        
        #각도를 적용하는 버튼
        self.btn_distance = tkinter.Button(panel, text="Degree 적용", relief="raised", command=self.updateDegreebar)
        self.btn_distance.pack(side="right", fill="both", expand="yes", padx=10, pady=5)


    #이동거리 초기화를 위한 함수
    def updateDistancebar(self):
        self.distance = self.distance_bar.get()
        print("[telloUI] distance 적용: {} m".format(self.distance))


    #각도 초기화를 위한 함수
    def updateDegreebar(self):
        self.degree = self.degree_bar.get()
        print("[telloUI] degree 적용: {} deg".format(self.degree))

    
    #QuitWaitingFlag를 True로 변경하는 함수
    def _setQuitWaitingFlag(self):    
        self.quit_waiting_flag = True  


    #플립을 위한 화면을 켜는 함수
    def openFlipWindow(self):
        
        panel = Toplevel(self.root)
        panel.wm_title("Gesture Recognition")

        #좌측 플립 버튼
        self.btn_flipl = tkinter.Button(panel, text="Flip Left", relief="raised", command=lambda: self.tello.flip('l'))
        self.btn_flipl.pack(side="bottom", fill="both", expand="yes", padx=10, pady=5)

        #우측 플립 버튼
        self.btn_flipr = tkinter.Button(panel, text="Flip Right", relief="raised", command=lambda: self.tello.flip('r'))
        self.btn_flipr.pack(side="bottom", fill="both", expand="yes", padx=10, pady=5)

        #전방 플립 버튼
        self.btn_flipf = tkinter.Button(panel, text="Flip Forward", relief="raised", command=lambda: self.tello.flip('f'))
        self.btn_flipf.pack(side="bottom", fill="both", expand="yes", padx=10, pady=5)

        #후방 플립 버튼
        self.btn_flipb = tkinter.Button(panel, text="Flip Backward", relief="raised", command=lambda: self.tello.flip('b'))
        self.btn_flipb.pack(side="bottom", fill="both", expand="yes", padx=10, pady=5)
       
       
 #=========Tello의 동작에 대한 키보드 링크 함수=============================================================================================== 


    def on_keypress_w(self, event):
        print("[telloUI] on_keypress_w: up {} m".format(self.distance))
        self.tello.move('up',self.distance)
        time.sleep(0.05)

    def on_keypress_s(self, event):
        print("[telloUI] on_keypress_s: down {} m".format(self.distance))
        self.tello.move('down',self.distance)
        time.sleep(0.05)

    def on_keypress_a(self, event):
        print("[telloUI] on_keypress_a: ccw {} degree".format(self.degree))
        self.tello.rotate_ccw(self.degree)
        time.sleep(0.05)

    def on_keypress_d(self, event):
        print("[telloUI] on_keypress_d: cw {} degree".format(self.degree))
        self.tello.rotate_cw(self.degree)
        time.sleep(0.05)

    def on_keypress_up(self, event):
        print("[telloUI] on_keypress_up: forward {} m".format(self.distance))
        self.tello.move('forward',self.distance)
        time.sleep(0.05)

    def on_keypress_down(self, event):
        print("[telloUI] on_keypress_down: backward {} m".format(self.distance))
        self.tello.move('back',self.distance)
        time.sleep(0.05)

    def on_keypress_left(self, event):
        print("[telloUI] on_keypress_left: left {} m".format(self.distance))
        self.tello.move('left',self.distance)
        time.sleep(0.05)

    def on_keypress_right(self, event):
        print("[telloUI] on_keypress_right: right {} m".format(self.distance))
        self.tello.move('right',self.distance)
        time.sleep(0.05)

    def onClose(self):
        try:
            print("[telloUI] 종료 중...")
            self.stopEvent.set() #종료를 알리는 버튼을 켜기
            del self.tello #tello 객체 제거
            self.root.quit() #화면 종료
        except: pass
        exit()

