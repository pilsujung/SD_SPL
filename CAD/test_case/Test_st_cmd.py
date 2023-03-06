import socket
from time import sleep
INTERVAL = 3

input("go")

# IPv4, UDP 방식의 소켓 생성 후 8890 포트와 연결(수신받을 포트)
socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # socket for sending cmd
socket.bind(('', 8889))

print("연결 되었고")

# 8889 포트로 SDK 모드 진입을 위한 command 명령어 송신
socket.sendto('command'.encode('utf-8'), ('192.168.137.146',8889))
response, ip = socket.recvfrom(1024)
print(response)

socket.sendto('mon'.encode('utf-8'), ('192.168.137.146',8889))
response, ip = socket.recvfrom(1024)
print(response)

socket.sendto('mdirection 2'.encode('utf-8'), ('192.168.137.146',8889))
response, ip = socket.recvfrom(1024)
print(response)

while True:
    print("<TOF>")
    socket.sendto('EXT tof?'.encode('utf-8'), ('192.168.137.146',8889))
    response, ip = socket.recvfrom(1024)
    if response == 'ok': continue
    
    response = response.decode('utf-8')
    # out = response.replace(';', ';\n')
    print(response)
    print()
    sleep(INTERVAL)