import socket #소켓 통신을 위한 라이브러리
import threading #멀티 스레딩을 위한 라이브러리
import numpy as np #H.264 영상 규격 처리 보조를 위한 수치계산 라이브러리
import h264decoder

#Tello와 상호작용하기 위한 클래스
class Tello:
    
#==========내부 사용 함수========================================================================================


    #클래스 생성 시 실행할 동작 (magic method)
    def __init__(self):

        #1) 소켓 생성
        self.socket_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # IPv4, UDP 통신 소켓 객체를 생성(command용)
        self.socket_cmd.bind(('', 8889)) #소켓 객체를 텔로와 바인딩(8889 포트)

        self.socket_state = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # IPv4, UDP 통신 소켓 객체를 생성(state용)
        self.socket_state.bind('',8890) #소켓 객체를 텔로와 바인딩(8890 포트)
        
        self.socket_camera = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # IPv4, UDP 통신 소켓 객체를 생성(camera용)
        self.socket_camera.bind(('', 11111)) #소켓 객체를 텔로와 바인딩(11111 포트)

        #2) 클래스 변수 생성
        self.tello_address = ('192.168.10.1',8889) #텔로에게 접속했을 때, 텔로의 IP주소
        self.command_timeout = 0.03 # command_timeout (float): 명령을 기다리는 시간(초)
        self.timeout_flag = False #timeout 사용을 위한 변수
        self.decoder = h264decoder.H264Decoder() #H.264 동영상 스트림 디코딩을 위한 객체
        self.response = None  #텔로의 응답을 저장할 변수
        self.frame = None  # numpy array BGR -- current camera output frame
        self.is_freeze = False  # 카메라가 puase 상태인지 확인하는 변수
        self.last_frame = None #마지막으로 저장된 frame을 기록할 변수
        self.last_height = 0 #마지막으로 저장된 높이를 기록할 변수
        
        #3) Tello에게서 응답을 받을 thread 실행
        self.receive_thread = threading.Thread(target=self._receive_thread, daemon=True)
        
        self.receive_thread.start() #스레드 시작
        print("[tello] tello_run: receive_thread 시작")

        #3) Tello를 SDK mode에 진입하도록 command 명령어를 전송
        self.socket.sendto(b'command', self.tello_address)
        print ("[tello] tello_send: command")

        #4) Tello가 동영상 스트림을 보내오도록 streamon 명령어를 전송
        self.socket.sendto(b'streamon', self.tello_address)
        print ("[tello] tello_send: streamon")

        #5) Tello에게서 동영상 스트림을 받을 thread 실행
        self.receive_video_thread = threading.Thread(target=self._receive_video_thread)
        self.receive_video_thread.daemon = True #오직 이 스레드만 실행 중인 경우 프로그램을 종료
        self.receive_video_thread.start() #스레드 시작
        print("[tello] tello_run: receive_video_thread thread 시작")


    #클래스 종료 시 실행할 동작(magic method)
    def __del__(self):
        self.socket_cmd.close() #command용 소켓 닫기
        self.socket_state.close() #state용 소켓 닫기
        self.socket_camera.close() #camera용 소켓 닫기
        print("[tello] 연결 종료")
    

    #Tello가 보낸 응답을 받아오는 함수
    def _receive_thread(self):

        err_cnt = 0 #에러 발생을 카운트하는 변수 (에러가 10번 이상 발생 시 함수 종료)
        while True:
            try:
                err_cnt = 0
                self.response = self.socket.recv(3000)

            except Exception as e:
                err_cnt += 1
                print ("[tello] tello_receive_thread socket.error 발생: {} (err_cnt: {})".format(e,err_cnt))
                
                if err_cnt == 100: break

        print("[tello] tello_run: receive_thread 종료")


    #Tello가 보낸 동영상 스트림을 받아오는 함수
    def _receive_video_thread(self):

        err_cnt = 0 #에러 발생을 카운트하는 변수 (에러가 10번 이상 발생 시 함수 종료)
        packet_data = bytes()
        
        while True:
            try:
                err_cnt = 0
                res_string = self.socket_camera.recv(2048)
                packet_data += res_string
                
                if len(res_string) != 1460: # frame의 끝이 아니면,
                    for frame in self._h264_decode(packet_data): self.frame = frame
                    packet_data = bytes()

            except Exception as e:
                err_cnt += 1
                print ("[tello] tello_receive_video_thread socket.error 발생: {} (err_cnt: {})".format(e,err_cnt))
                
                if err_cnt == 100: break

        print("[tello] tello_run: receive_video_thread 종료")
    

    #Tello가 보낸 동영상 스트림을 디코딩하는 함수
    def _h264_decode(self, packet_data):
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


    #Tello에게 command를 전송하는 함수
    def _send_command(self, command):
        print ("[tello] tello_send: {}".format(command))
        self.timeout_flag = False
        timer = threading.Timer(self.command_timeout, self.set_timeout_flag) #command_timeout(sec)마다, set_timeout_flag 함수를 시행

        #Tello에게 command를 전송
        self.socket.sendto(command.encode('utf-8'), self.tello_address)

        timer.start()
        while (self.response is None) and (self.timeout_flag is False): continue #timeout 시간만큼 대기
        timer.cancel()
        
        if self.response is None: response = "[tello] tello_send_command: command에 대한 receive가 없습니다.."
        else: response = self.response.decode('utf-8')

        self.response = None

        return response

    def read_frame(self):
        if self.is_freeze: return self.last_frame
        else: return self.frame

    def video_freeze(self, is_freeze=True):
        self.is_freeze = is_freeze
        if is_freeze: self.last_frame = self.frame
    


#==========실제 사용 함수========================================================================================


    def set_timeout_flag(self):
        # timeout을 설정하는 함수
        self.timeout_flag = True


    def set_speed(self, speed):

        speed = int(round(speed * 27.7778))

        return self._send_command('speed {}'.format(speed))


    def get_height(self): #return: Tello의 receive 'height'

        try:
            height = int(filter(str.isdigit, str(self._send_command('height?'))))
            self.last_height = height
        except:
            height = self.last_height
            pass

        return height


    def get_battery(self): #return: Tello의 receive 'battery'

        try: battery = int(self._send_command('battery?'))
        except: pass

        return battery


    def get_flight_time(self): #return: Tello의 receive 'flight_time'

        try: flight_time = int(self._send_command('time?'))
        except: pass

        return flight_time


    def get_speed(self): #return: Tello의 receive 'speed'

        try: speed = round((self._send_command('speed?') / 27.7778), 1)
        except: pass

        return speed


    def land(self): #return: Tello의 receive 'OK' or 'FALSE'
        return self._send_command('land')


    def takeoff(self): #return: Tello의 receive 'OK' or 'FALSE'
        return self._send_command('takeoff')


    def move(self, direction, distance): #return: Tello의 receive 'OK' or 'FALSE'
        # distance: 20 ~ 500 centimeters.
        # direction: 'up, down, forward', 'back', 'right', 'left'
        # Metric: .02 to 5 meters

        distance = int(round(distance * 100))

        return self._send_command("{} {}".format(direction, distance))
    

    def rotate_cw(self, degrees): #return: Tello의 receive 'OK' or 'FALSE'
        return self._send_command('cw {}'.format(degrees))


    def rotate_ccw(self, degrees): #return: Tello의 receive 'OK' or 'FALSE'
        return self._send_command('ccw {}'.format(degrees))


    def flip(self, direction): #return: Tello의 receive 'OK' or 'FALSE'
        # direction: 'f', 'b', 'r', 'l'
        return self._send_command('flip %s' % direction)