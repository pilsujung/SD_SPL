from asyncio import sleep
import socket

INTERVAL = 100

# IPv4, UDP 방식의 소켓 생성 후 8890 포트와 연결(수신받을 포트)
socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # socket for sending cmd
socket.bind(('', 8889))
#---------------------소켓 연결-----------------------#

# 8889 포트로 SDK 모드 진입을 위한 command 명령어 송신
socket.sendto('command'.encode('utf-8'), ('192.168.10.1',8889))
response, ip = socket.recvfrom(1024)
print(response)

print(">>이거는 잘 실행 돼?")
#--------------------명령어 포트 연결--------------------------#

socket.sendto('ap LUC LUCLUCLUC'.encode('utf-8'), ('192.168.10.1',8889))
response, ip = socket.recvfrom(1024)
print(response)

print("여기까지가 스테이션 모드 테스트")

# while True:
#     c = input(">>")
#     if (c=="go"):
#         print("<TOF>")
#         socket.sendto('EXT tof?'.encode('utf-8'), ('192.168.10.1',8889))
#         response, ip = socket.recvfrom(1024)
#         if response == 'ok': continue
    
#         response = response.decode('utf-8')
#         # out = response.replace(';', ';\n')
#         print(response)
#         print()
#         sleep(INTERVAL)