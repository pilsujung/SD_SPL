from unittest.mock import MagicMock, patch
import sys, os, unittest, socket, threading
import numpy as np
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.dirname(__file__)))))
from run5_CAD import Main
from CAD.Basemodel.Actor import Actor
from CAD.Basemodel.Sensor import Sensor
from CAD.Basemodel.ObjectDetector import ObjectDetector
from CAD.Decoder.H264decoder import decode
from CAD.Decoder.h264_39 import h264decoder
from CAD.ObjectDetector.YOLOv5 import YOLOv5
from CAD.Tello.Tello8889Actor import Tello8889Actor
from CAD.Tello.Tello8889Sensor import Tello8889Sensor
from CAD.Tello.Tello11111Sensor import Tello11111Sensor
from CAD.Test.TelloVirtualController import TelloVirtualController
from CAD.Plan.Planner5 import Planner
from CAD.Calculation.ValueChecker import is_tof_val, is_sdk_val
from CAD.Calculation.ValueChanger import change_mm_to_cm, change_val_to_coor, change_cmd_for_tello, change_windows_to_window, change_to_safe_cmd



#===========================================================================================================
class TestMain(unittest.TestCase):
    @patch('socket.socket', spec=socket.socket)
    @patch('socks.socksocket', spec=socket.socket)
    def test_TID4001(self, mock_socket, mock_socksocket):
        # Mock socket instance
        mock_socket_instance = MagicMock(spec=socket.socket)
        mock_socket.return_value = mock_socket_instance
        mock_socksocket.return_value = mock_socket_instance
        mock_socket_instance.recvfrom = MagicMock(return_value=(b'response', ('192.168.10.1', 8889)))
        mock_socket_instance.bind = MagicMock()
        mock_socket_instance.sendto = MagicMock()

        # Initialize Main instance
        main = Main()

        # Check attributes
        self.assertIsNotNone(main.stop_event)
        self.assertEqual(main.tello_address, ('192.168.10.1', 8889))
        self.assertEqual(main.is_takeoff, False)
        self.assertIsNotNone(main.socket8889)
        self.assertIsNotNone(main.planner)
        self.assertIsNotNone(main.tello8889sensor)
        self.assertIsNotNone(main.tello11111sensor)
        self.assertIsNotNone(main.tello8889actor)
        self.assertIsInstance(main.virtual_controller, TelloVirtualController)










#===========================================================================================================                     
class TestPlanner(unittest.TestCase):
    
    def setUp(self):
        self.main_mock = MagicMock()
        self.main_mock.test = True
        self.main_mock.stop_event = threading.Event()
        self.main_mock.stop_event.is_set = MagicMock(return_value=False)
        self.main_mock.tello_address = ('192.168.10.1', 8889)
        self.main_mock.socket8889 = MagicMock()
        self.planner = Planner(self.main_mock)
   
    def test_TID5001(self):
        self.assertIsNotNone(self.planner)
        self.assertEqual(self.planner.stop_event, self.main_mock.stop_event)
        self.assertEqual(self.planner.socket8889, self.main_mock.socket8889)
        self.assertEqual(self.planner.tello_address, self.main_mock.tello_address)
        self.assertEqual(self.planner.threshold_distance, 60)
        self.assertEqual(self.planner.base_move_distance, 60)
        self.assertEqual(self.planner.safe_constant, 20)
        self.assertEqual(self.planner._Planner__cmd_queue, [])
        self.assertIsNone(self.planner._Planner__info_8889Sensor_tof)
        self.assertIsNone(self.planner._Planner__info_8889Sensor_cmd)
        self.assertIsNone(self.planner._Planner__info_11111Sensor_frame)
        self.assertIsNone(self.planner._Planner__info_11111Sensor_image)
        self.assertIsNone(self.planner._Planner__info_11111Sensor_coor)
        self.assertEqual(self.planner._Planner__YOLOv5, "TEST")
        self.assertTrue(self.planner._Planner__thr_stay_connection.is_alive())
        self.assertTrue(self.planner._Planner__thr_planner.is_alive())
        self.assertTrue(self.planner._Planner__thr_requset_tof.is_alive())
    
    #test_pop_cmd_queue
    def test_TID1022(self):
        self.assertIsNone(self.planner.pop_cmd_queue())
        self.planner._Planner__cmd_queue = ['command1', 'command2']
        self.assertEqual(self.planner.pop_cmd_queue(), 'command1')
        self.assertEqual(self.planner._Planner__cmd_queue, ['command2'])
        
    #test_insert_cmd_queue
    def test_TID1023(self):
        self.planner._Planner__cmd_queue = []
        self.planner.insert_cmd_queue('command1')
        self.assertEqual(self.planner._Planner__cmd_queue, ['command1'])
        self.planner.insert_cmd_queue('command2')
        self.assertEqual(self.planner._Planner__cmd_queue, ['command1', 'command2']) 
    
    #test_get_info_8889Sensor_tof
    def test_TID2003(self):
        self.assertIsNone(self.planner.get_info_8889Sensor_tof())
        self.planner.set_info_8889Sensor_tof(100)
        self.assertEqual(self.planner.get_info_8889Sensor_tof(), 100)

    #test_set_info_8889Sensor_tof
    def test_TID2004(self):
        self.assertIsNone(self.planner.get_info_8889Sensor_tof())
        self.planner.set_info_8889Sensor_tof(100)
        self.assertEqual(self.planner._Planner__info_8889Sensor_tof, 100)   
    
    #test_get_info_8889Sensor_cmd
    def test_TID1024(self):
        self.assertIsNone(self.planner.get_info_8889Sensor_cmd())
        self.planner._Planner__info_8889Sensor_cmd = 'ready'
        self.assertEqual(self.planner.get_info_8889Sensor_cmd(), 'ready')

    #test_set_info_8889Sensor_cmd
    def test_TID1025(self):
        self.assertIsNone(self.planner._Planner__info_8889Sensor_cmd)
        self.planner.set_info_8889Sensor_cmd('ready')
        self.assertEqual(self.planner._Planner__info_8889Sensor_cmd, 'ready')
        
    #test_get_info_11111Sensor_frame
    def test_TID3006(self):
        self.assertIsNone(self.planner.get_info_11111Sensor_frame())
        self.planner._Planner__info_11111Sensor_frame = 'ready'
        self.assertEqual(self.planner.get_info_11111Sensor_frame(), 'ready')

    #test_set_info_11111Sensor_frame
    def test_TID3007(self):
        self.assertIsNone(self.planner._Planner__info_11111Sensor_frame)
        self.planner.set_info_11111Sensor_frame('ready')
        self.assertEqual(self.planner._Planner__info_11111Sensor_frame, 'ready')
        
    #test_get_info_11111Sensor_image
    def test_TID3008(self):
        self.assertIsNone(self.planner.get_info_11111Sensor_image())
        self.planner._Planner__info_11111Sensor_image = 'ready'
        self.assertEqual(self.planner.get_info_11111Sensor_image(), 'ready')

    #test_set_info_11111Sensor_image
    def test_TID3009(self):
        self.assertIsNone(self.planner._Planner__info_11111Sensor_image)
        self.planner.get_info_11111Sensor_image()
        self.assertEqual(self.planner._Planner__info_11111Sensor_image, None)
         
    #test_get_info_11111Sensor_coor
    def test_TID5002(self):
        self.assertIsNone(self.planner.get_info_11111Sensor_coor())
        self.planner._Planner__info_11111Sensor_coor = 'ready'
        self.assertEqual(self.planner.get_info_11111Sensor_coor(), 'ready')

    #test_set_info_11111Sensor_coor
    def test_TID5003(self):
        self.assertIsNone(self.planner._Planner__info_11111Sensor_coor)
        self.planner.get_info_11111Sensor_coor()
        self.assertEqual(self.planner._Planner__info_11111Sensor_coor, None)    
        
    #test_redraw_frame
    def test_TID5004(self):
        test_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        test_tof = 100  # Example Time-of-Flight value
        self.planner.set_info_11111Sensor_frame(test_frame)
        self.planner.set_info_8889Sensor_tof(test_tof)

        frame, tof, object_coor = self.planner._Planner__redraw_frame()

        self.assertIsNone(frame)
        self.assertIsNone(tof)
        self.assertIsNone(object_coor)

    #test__create_avd_cmd
    def test_TID5005(self):
        real_coor = (50, (30, -20), (20, 10))
        avd_cmd = self.planner._Planner__create_avd_cmd(real_coor)
        self.assertEqual(avd_cmd, "up 60")

    #test_create_real_core
    def test_TID5006(self):
        object_core = ((50, 30), (100, 80))  # Example window coordinates
        tof = 40
        screen_size = (640, 480)
        expected_result = (40, (-9.442708333333334, 7.130208333333333), (1.9270833333333333, 1000))
        result = self.planner._Planner__create_real_coor(object_core, tof, screen_size)
        self.assertEqual(result, expected_result)

#===========================================================================================================  
class TestYOLOv5(unittest.TestCase):
    @patch('CAD.ObjectDetector.YOLOv5.torch.hub.load')
    def setUp(self, mock_torch_hub_load):
        self.yolov5 = YOLOv5()
        self.mock_torch_hub_load = mock_torch_hub_load

    def test_TID4005(self):
        # Test the __init__ method
        self.mock_torch_hub_load.assert_called_once()

    #test_detect_from_frame
    def test_TID4006(self):
        # Test the detect_from_frame method
        test_frame = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8)
        tof = 25

        with patch.object(self.yolov5, '_YOLOv5__score_frame', return_value=([], [])) as mock_score_frame:
            image, fusion_window_coor = self.yolov5.detect_from_frame(test_frame, tof)

            mock_score_frame.assert_called_once_with(test_frame)

            self.assertIsNotNone(image)
            self.assertIsNotNone(fusion_window_coor)

    #test_calculate_ir_window_coor_tof_none
    def test_TID4007(self):
        # Test the __calculate_ir_window_coor method with tof=None
        tof = None
        height = 480
        width = 640
        expected_result = (None, None)
        result = self.yolov5._YOLOv5__calculate_ir_window_coor(tof, height, width)
        self.assertEqual(result, expected_result)





#===========================================================================================================  
class TestDecode(unittest.TestCase):

    def setUp(self):
        self.decoder_mock = MagicMock(spec=h264decoder)
        self.packet_data = b'\x00\x00\x00\x01'

    #test_decode
    def test_TID3010(self):
        width, height, linesize = 640, 480, 1920
        frame_data = (b'\x00' * height * linesize, width, height, linesize)
        self.decoder_mock.decode = MagicMock(return_value=[frame_data])
        res_frame_list = decode(self.decoder_mock, self.packet_data)
        self.decoder_mock.decode.assert_called_once_with(self.packet_data)
        self.assertEqual(len(res_frame_list), 1)



#===========================================================================================================  
class TestCheckValues(unittest.TestCase):
    
    #test_is_tof_val
    def test_TID1026(self):
        # Test for valid tof value
        valid_tof_val = "tof 500"
        self.assertTrue(is_tof_val(valid_tof_val))

        # Test for tof value below range
        tof_val_below_range = "tof -10"
        self.assertFalse(is_tof_val(tof_val_below_range))

        # Test for tof value above range
        tof_val_above_range = "tof 9000"
        self.assertFalse(is_tof_val(tof_val_above_range))

        # Test for invalid format
        invalid_format = "tof"
        self.assertFalse(is_tof_val(invalid_format))

        # Test for non-tof value
        non_tof_val = "battery?"
        self.assertFalse(is_tof_val(non_tof_val))

    #test_is_sdk_val
    def test_TID1027(self):
        # Test for valid SDK value
        valid_sdk_val = "command"
        self.assertTrue(is_sdk_val(valid_sdk_val))



#===========================================================================================================  
class TestChangeValues(unittest.TestCase):
    
    #test_change_mm_to_cm
    def test_TID1028(self):
        self.assertEqual(change_mm_to_cm(0), 0)
        self.assertEqual(change_mm_to_cm(5), 0)
        self.assertEqual(change_mm_to_cm(10), 1)
        self.assertEqual(change_mm_to_cm(15), 1)
        self.assertEqual(change_mm_to_cm(100), 10)
        self.assertEqual(change_mm_to_cm(1000), 100)
        self.assertEqual(change_mm_to_cm(12345), 1234)
    
    #test_change_val_to_coor
    def test_TID1029(self):
        object_val3 = (None, ((0, 0), (100, 100)), (640, 480))
        expected_output3 = None
        self.assertEqual(change_val_to_coor(object_val3), expected_output3)
        
    #test_change_cmd_for_tello
    def test_TID1030(self):
        cmd1 = "left 100"
        expected_output2 = b'rc -100 0 0 0'
        self.assertEqual(change_cmd_for_tello(cmd1), expected_output2)
        
        cmd2 = "stop"
        expected_output3 = b'rc 0 0 0 0'
        self.assertEqual(change_cmd_for_tello(cmd2), expected_output3)

        cmd3 = "fly"
        expected_output4 = b'fly'
        self.assertEqual(change_cmd_for_tello(cmd3), expected_output4)
        
        cmd4 = "right 150"
        expected_output5 = b'rc 100 0 0 0'
        self.assertEqual(change_cmd_for_tello(cmd4), expected_output5)
        
        cmd5 = "back 20"
        expected_output6 = b'rc 0 -60 0 0'
        self.assertEqual(change_cmd_for_tello(cmd5), expected_output6)

    #test_change_windows_to_window
    def test_TID1031(self):
        window_list1 = [((100, 100), (200, 200))]
        ir_left_up_coor1 = (150, 150)
        ir_right_down_coor1 = (250, 250)
        expected_output1 = ((100, 100), (200, 200))
        self.assertEqual(change_windows_to_window(window_list1, ir_left_up_coor1, ir_right_down_coor1), expected_output1)

        window_list2 = [((100, 100), (200, 200))]
        ir_left_up_coor4 = (150, 0)
        ir_right_down_coor4 = (250, 100)
        expected_output4 = ((100, 100), (200, 200))
        self.assertEqual(change_windows_to_window(window_list2, ir_left_up_coor4, ir_right_down_coor4), expected_output4)
        
        window_list3 = []
        ir_left_up_coor5 = (100, 100)
        ir_right_down_coor5 = (200, 200)
        expected_output5 = ((-131072, -131072), (131072, 131072))
        self.assertEqual(change_windows_to_window(window_list3, ir_left_up_coor5, ir_right_down_coor5), expected_output5)
    
    #test_forward_cmd
    def test_TID1032(self):
        cmd1 = "forward 30"
        tof1 = 150
        threshold1 = 50
        expected1 = "forward 70"
        self.assertEqual(change_to_safe_cmd(cmd1, tof1, threshold1), expected1)

        cmd2 = "forward 50"
        tof2 = 1000
        threshold2 = 50
        expected2 = "forward 50"
        self.assertEqual(change_to_safe_cmd(cmd2, tof2, threshold2), expected2)
        
        cmd3 = "forward 50"
        tof3 = 2000
        threshold3 = 50
        expected3 = "forward 50"
        self.assertEqual(change_to_safe_cmd(cmd3, tof3, threshold3), expected3)

        cmd4 = "turn_left 30"
        tof4 = 150
        threshold4 = 50
        expected4 = "turn_left 30"
        self.assertEqual(change_to_safe_cmd(cmd4, tof4, threshold4), expected4)
        
        

#===========================================================================================================                     
class TestScenario_control(unittest.TestCase):
    
    def setUp(self):
        self.__cmd_queue = [] #명령을 저장할 큐
        
    def on_keypress_test(self, test_key, test_direction):
        print(test_key,test_direction)
        self.move(test_direction,50)
    
    def takeoff(self): #return: Tello의 receive 'OK' or 'FALSE'
         self.send_cmd('takeoff')
    
    def land(self): #return: Tello의 receive 'OK' or 'FALSE'
        self.send_cmd('land')
        
    def move(self, direction, distance): 
        """
        direction: up, down, forward, back, right, left
        distance: 20~500 cm
        """
        self.send_cmd("{} {}".format(direction, distance))

    def send_cmd(self, msg:str):
        # self.__lock.acquire() #락 획득
        try:
            self.insert_controller_queue(msg)
            self.insert_controller_queue("stop")

        except Exception as e:
            print("ERROR {}".format(e),sys._getframe().f_code.co_name)
        # self.__lock.release() #락 해제
    
    def insert_controller_queue(self,cmd):
        self.insert_cmd_queue(cmd)
    
    def insert_cmd_queue(self, info):
        # self.__lock_cmd_queue.acquire()
        self.__cmd_queue.append(info)
        # self.__lock_cmd_queue.release()
    
    def pop_cmd_queue(self):
        # self.__lock_cmd_queue.acquire()
        data = None
        if len(self.__cmd_queue)>0:
            data = self.__cmd_queue.pop(0)
        return data

    def take_cmd_from_planner(self): 
        """
        Planner로부터 cmd를 가져온다
        """
        cmd = self.pop_cmd_queue()
        return cmd
    
    def test_TID1033(self):
        self.on_keypress_test('w', 'up')
        cmd = self.take_cmd_from_planner()
        self.assertEqual('up 50',cmd)
    
    def test_TID1034(self):
        self.on_keypress_test('a', 'ccw')
        cmd = self.take_cmd_from_planner()
        self.assertEqual('ccw 50',cmd)
    
    def test_TID1035(self):
        self.on_keypress_test('d', 'cw')
        cmd = self.take_cmd_from_planner()
        self.assertEqual('cw 50',cmd)
    
    def test_TID1036(self):
        self.on_keypress_test('s', 'down')
        cmd = self.take_cmd_from_planner()
        self.assertEqual('down 50',cmd)
    
    def test_TID1037(self):
        self.takeoff()
        cmd = self.take_cmd_from_planner()
        self.assertEqual('takeoff',cmd)
    
    def test_TID1038(self):
        self.land()
        cmd = self.take_cmd_from_planner()
        self.assertEqual('land',cmd)
        
        

#===========================================================================================================                     
class TestScenario_tof(unittest.TestCase):
    def setUp(self):
        self.__info_8889Sensor_tof = None
        self.__info_8889Sensor_cmd = None

        
    def take_data_from_sensor(self, data): 
        """
        센서로부터 data를 가져온다
        """
        return data

    def change_data_to_info(self, data: bytes):
        """
        data를 Planner가 이해할 수 있는 info로 변경한다
        """
        info:str = data.decode('utf-8')
        return info
    
    def save_to_planner(self, info: str):
        """
        info를 Planner에 저장한다
        """
        if is_tof_val(info):
            #ToF 값은 "tof 100" 형태로 들어온다
            info = change_mm_to_cm(int(info.split()[-1]))
            if info > 60:
                info = 1000
            self.set_info_8889Sensor_tof(info)
        
        else: #cmd return 값이면
            self.set_info_8889Sensor_cmd(info)
            print("[Tello8889Sensor]",info)
    
    def set_info_8889Sensor_cmd(self, info):
        # self.__lock_info_8889Sensor_cmd.acquire()
        self.__info_8889Sensor_cmd = info
        # self.__lock_info_8889Sensor_cmd.release()
    
    def set_info_8889Sensor_tof(self, info):
        # self.__lock_info_8889Sensor_tof.acquire()
        self.__info_8889Sensor_tof = info
        # self.__lock_info_8889Sensor_tof.release()
        
    def get_info_8889Sensor_tof(self):
        # self.__lock_info_8889Sensor_tof.acquire()
        info = self.__info_8889Sensor_tof
        # self.__lock_info_8889Sensor_tof.release()
        return info
    
    def test_TID2005(self):
        data = self.take_data_from_sensor(b'tof 500')
        info = self.change_data_to_info(data)
        self.save_to_planner(info)
        tof = self.get_info_8889Sensor_tof()
        self.assertEqual(50, tof)
    
    def test_TID2006(self):
        data = self.take_data_from_sensor(b'tof 1000')
        info = self.change_data_to_info(data)
        self.save_to_planner(info)
        tof = self.get_info_8889Sensor_tof()
        self.assertEqual(1000, tof)
    
    def test_TID2007(self):
        data = self.take_data_from_sensor(b'tof 2000')
        info = self.change_data_to_info(data)
        self.save_to_planner(info)
        tof = self.get_info_8889Sensor_tof()
        self.assertEqual(1000, tof)        
        
        
        
#===========================================================================================================                     
class TestScenario_camera(unittest.TestCase):
    
    def setUp(self):
        self.__packet_data = bytes()
        self.__info_11111Sensor_frame = None
        
    def take_data_from_sensor(self, data): 
        """
        센서로부터 data를 가져온다
        """
        data:bytes = data
        self.__packet_data += data
    
    def save_to_planner(self, info):
        """
        info를 Planner에 저장한다
        """
        self.set_info_11111Sensor_frame(info)
        
    def set_info_11111Sensor_frame(self, info):
        # self.__lock_info_11111Sensor_frame.acquire()
        self.__info_11111Sensor_frame = info
        # self.__lock_info_11111Sensor_frame.release()
    
    def get_info_11111Sensor_image(self):
        # self.__lock_info_11111Sensor_image.acquire()
        info = self.__info_11111Sensor_frame
        # self.__lock_info_11111Sensor_image.release()
        return info
    
    def test_TID3011(self):
        self.take_data_from_sensor(b'test')
        self.save_to_planner()
        image = self.get_info_11111Sensor_image()
        self.assertEqual(b'test',image)     
    
    def test_TID3012(self):
        self.take_data_from_sensor(None)
        self.save_to_planner()
        image = self.get_info_11111Sensor_image()
        self.assertEqual(None,image)    



#===========================================================================================================                     
class TestScenario_yolo(unittest.TestCase):
    
    def setUp(self):
       self.__info_11111Sensor_image = None
    
    def detect_from_frame(self, frame, tof): 
        return (frame, tof)
        
    def set_info_11111Sensor_image(self, info):
        # self.__lock_info_11111Sensor_image.acquire()
        self.__info_11111Sensor_image = info
        # self.__lock_info_11111Sensor_image.release()
    
    def get_info_11111Sensor_image(self):
        # self.__lock_info_11111Sensor_image.acquire()
        info = self.__info_11111Sensor_image
        # self.__lock_info_11111Sensor_image.release()
        return info
        
    def test_TID4008(self):
        image, _ = self.detect_from_frame('test', 50)
        self.set_info_11111Sensor_image(image)
        result = self.get_info_11111Sensor_image()
        self.assertEqual('test',result)
      
    def test_TID4009(self):
        image, _ = self.detect_from_frame(None, 50)
        self.set_info_11111Sensor_image(image)
        result = self.get_info_11111Sensor_image()
        self.assertEqual(None,result)
        
        
        
#===========================================================================================================                     
class TestDroneObstacleAvoidance(unittest.TestCase):
    
    def setUp(self):
        # Create a Main instance and mock its attributes
        self.main = Main()
        self.main.stop_event = MagicMock()
        self.main.virtual_controller = MagicMock(spec=TelloVirtualController)
        self.main.planner = MagicMock(spec=Planner)
        self.main.tello8889sensor = MagicMock(spec=Tello8889Sensor)
        self.main.tello11111sensor = MagicMock(spec=Tello11111Sensor)
        self.main.tello8889actor = MagicMock(spec=Tello8889Actor)

    def test_obstacle_avoidance(self):
        # Simulate the presence of an obstacle
        self.main.tello8889sensor.save_to_planner("tof 30")

        # Call the avoidance logic
        self.main.planner.__func_planner()

        # Check if the avoidance command was sent
        self.main.tello8889actor.__func_actor.assert_called_once()
        
        
        
#===========================================================================================================                     
if __name__ == "__main__":
    unittest.main()
