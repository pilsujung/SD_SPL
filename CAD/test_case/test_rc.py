import socket
from time import sleep



'''
<<rc command>>
rc a b c d (-100~100)

a: roll (+좌/-우)
b: ptich (+전진/-후진)
c:throttle (+상승/-하강)
d:yaw (+좌회전/-우회전)

'''

INTERVAL = 3
socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # socket for sending cmd
socket.bind(('', 8889))

def cmd_send(socket,cmd):
    print("cmd:",cmd)
    socket.sendto(cmd.encode('utf-8'), ('192.168.10.1',8889))

def cmd_wait_send(socket,cmd):
    cmd_send(socket,cmd)
    response, ip = socket.recvfrom(1024)
    response = response.decode('utf-8')
    print("response:",response)
    print()
    if "error" in response:
        cmd_send(socket,"battery?")
        response, ip = socket.recvfrom(1024)
        response = response.decode('utf-8')
        print("response:",response)
        
        if int(response) <= 20:
            print("※※※경고: 배터리의 잔량이 {}% 남았습니다※※※".format(response)) 
        

def stop(socket):
    cmd_send(socket,"rc 0 0 0 0")

def right(socket):
    cmd_send(socket,"rc 100 0 0 0")
    sleep(0.5)
    cmd_send(socket,"rc -100 0 0 0")
    sleep(0.2)
    stop(socket)

def left(socket):
    cmd_send(socket,"rc -1 0 0 0")

# def ccw():
#     cmd_send("rc 0 0 0 0")
    


# def cw():
#     cmd_send("rc 0 0 0 0")
    
# def cw():
#     cmd_send("rc 0 0 0 0")

# def cw():
#     cmd_send("rc 0 0 0 0")
    

if __name__ == "__main__":

    cmd_wait_send(socket,"command")
    cmd_wait_send(socket,"speed 100")
    cmd_wait_send(socket,"takeoff")
    
    right(socket)
    sleep(1)
    cmd_wait_send(socket,"land")



    # try:
    #     while True:
    #         socket.sendto('rc '.encode('utf-8'), ('192.168.10.1',8889))
    #         response, ip = socket.recvfrom(1024)
    #         if response == 'ok': continue
            
    #         response = response.decode('utf-8')
    #         # out = response.replace(';', ';\n')
    #         print(response)
    #         print()
    #         sleep(INTERVAL)
    # except KeyboardInterrupt: #종료 시, 설정을 다시 원래대로 복구
    #     pass


