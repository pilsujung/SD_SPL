from unittest.mock import MagicMock, patch
import sys, os, unittest, socket, threading
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.dirname(__file__)))))
from run1_base import Main
from CAD.Basemodel.Actor import Actor
from CAD.Basemodel.Sensor import Sensor
from CAD.Tello.Tello8889Actor import Tello8889Actor
from CAD.Tello.Tello8889Sensor import Tello8889Sensor
from CAD.Test.TelloVirtualController import TelloVirtualController
from CAD.Plan.Planner1 import Planner
from CAD.Calculation.ValueChecker import is_tof_val, is_sdk_val
from CAD.Calculation.ValueChanger import change_mm_to_cm, change_val_to_coor, change_cmd_for_tello, change_windows_to_window, change_to_safe_cmd



#===========================================================================================================
class TestMain(unittest.TestCase):
    @patch('socket.socket')
    def test_TID1001(self, mock_socket):
        mock_socket_instance = MagicMock(spec=socket.socket)
        mock_socket.return_value = mock_socket_instance
        mock_socket_instance.recvfrom = MagicMock(return_value=(b'response', ('192.168.10.1', 8889)))
        mock_socket_instance.bind = MagicMock()
        mock_socket_instance.sendto = MagicMock()

        main = Main()

        self.assertIsNotNone(main.stop_event)
        self.assertEqual(main.tello_address, ('192.168.10.1', 8889))
        self.assertEqual(main.is_takeoff, False)
        self.assertIsNotNone(main.socket8889)
        self.assertIsNotNone(main.planner)
        self.assertIsNotNone(main.tello8889sensor)
        self.assertIsNotNone(main.tello8889actor)
        self.assertIsInstance(main.virtual_controller, TelloVirtualController)



#===========================================================================================================
class TestActor_(Actor):
    def __init__(self):
        self.command = None

    def take_cmd_from_planner(self, cmd):
        self.command = cmd
        return self.command

    def change_cmd_is_safe(self, cmd):
        return 'safe_' + cmd

    def change_cmd_for_drone(self, cmd):
        return 'drone_' + cmd

    def send_to_actuator(self, cmd):
        return 'sent_' + cmd

class TestActor(unittest.TestCase):
    def setUp(self):
        self.test_actor = TestActor_()

    #test_take_cmd_from_planner
    def test_TID1002(self):
        mock_planner = MagicMock()
        mock_planner.get_command.return_value = 'example_command'

        cmd = self.test_actor.take_cmd_from_planner(mock_planner.get_command.return_value)
        self.assertEqual(cmd, 'example_command')
        self.assertEqual(self.test_actor.command, 'example_command')

    #test_change_cmd_is_safe
    def test_TID1003(self):
        safe_cmd = self.test_actor.change_cmd_is_safe('example_command')
        self.assertEqual(safe_cmd, 'safe_example_command')

    #test_change_cmd_for_drone
    def test_TID1004(self):
        drone_cmd = self.test_actor.change_cmd_for_drone('example_command')
        self.assertEqual(drone_cmd, 'drone_example_command')

    #test_send_to_actuator
    def test_TID1005(self):
        sent_cmd = self.test_actor.send_to_actuator('example_command')
        self.assertEqual(sent_cmd, 'sent_example_command')
                
            
                
#===========================================================================================================         
class TestSensor_(Sensor):
    def __init__(self):
        self.data = None
        self.info = None

    def take_data_from_sensor(self, data):
        self.data = data
        return self.data

    def change_data_to_info(self, data):
        self.info = 'info_' + data
        return self.info

    def save_to_planner(self, mock_planner, info):
        mock_planner.save_info(info)
        return True

class TestSensor(unittest.TestCase):
    def setUp(self):
        self.test_sensor = TestSensor_()
        
    #test_take_data_from_sensor
    def test_TID1006(self):
        data = self.test_sensor.take_data_from_sensor('example_data')
        self.assertEqual(data, 'example_data')
        self.assertEqual(self.test_sensor.data, 'example_data')
        
    #test_change_data_to_info
    def test_TID1007(self):
        info = self.test_sensor.change_data_to_info('example_data')
        self.assertEqual(info, 'info_example_data')
        
    #test_save_to_planner
    def test_TID1008(self):
        mock_planner = MagicMock()
        info = 'info_example_data'
        result = self.test_sensor.save_to_planner(mock_planner, info)
        self.assertTrue(result)
        mock_planner.save_info.assert_called_once_with(info)

   
            
#===========================================================================================================   
class TestTello8889Actor(unittest.TestCase):
    def setUp(self):
        self.main_mock = MagicMock()
        self.main_mock.stop_event = threading.Event()
        self.main_mock.socket8889 = MagicMock()
        self.main_mock.tello_address = ('192.168.10.1', 8889)
        self.main_mock.planner = MagicMock()
        self.tello_actor = Tello8889Actor(self.main_mock)
        
    #test_take_cmd_from_planner
    def test_TID1009(self):
        self.main_mock.planner.pop_cmd_queue.return_value = 'command'
        cmd = self.tello_actor.take_cmd_from_planner()
        self.assertEqual(cmd, 'command')
        
    #test_change_cmd_is_safe
    def test_TID1010(self):
        self.main_mock.planner.get_info_8889Sensor_tof.return_value = 100
        safe_cmd = self.tello_actor.change_cmd_is_safe('forward 10')
        self.assertEqual(safe_cmd, 'forward 40')

    #test_change_cmd_for_drone
    def test_TID1011(self):
        drone_cmd = self.tello_actor.change_cmd_for_drone('forward 10')
        self.assertEqual(drone_cmd, b'rc 0 60 0 0')

    #test_send_to_actuator
    def test_TID1012(self):
        self.tello_actor.send_to_actuator("command".encode('utf-8'))
        self.main_mock.socket8889.sendto.assert_called_once_with("command".encode('utf-8'), self.main_mock.tello_address)      

      

#===========================================================================================================   
class TestTello8889Sensor(unittest.TestCase):
    def setUp(self):
        self.main_mock = MagicMock()
        self.main_mock.stop_event = threading.Event()
        self.main_mock.socket8889 = MagicMock()
        self.main_mock.planner = MagicMock()
        self.tello_sensor = Tello8889Sensor(self.main_mock)

    def tearDown(self):
        # Set the stop event to signal the daemon thread to stop
        self.main_mock.stop_event.set()
        self.tello_sensor._Tello8889Sensor__thr_sensor.join()
        
    #test_take_data_from_sensor
    def test_TID1013(self):
        self.main_mock.socket8889.recv.return_value = b'tof 100'
        data = self.tello_sensor.take_data_from_sensor()
        self.assertEqual(data, b'tof 100')

    #test_change_data_to_info
    def test_TID1014(self):
        info = self.tello_sensor.change_data_to_info(b'tof 100')
        self.assertEqual(info, 'tof 100')

    #test_save_to_planner
    def test_TID1015(self):
        self.tello_sensor.save_to_planner('tof 100')
        self.main_mock.planner.set_info_8889Sensor_tof.assert_called_once_with(10)            
 


#===========================================================================================================  
class TestTelloVirtualController(unittest.TestCase):
    
    def setUp(self):
        self.main_mock = MagicMock()
        self.main_mock.socket8889 = MagicMock()
        self.main_mock.tello_address = ('192.168.10.1', 8889)
        self.main_mock.planner = MagicMock()
        self.main_mock.stop_event = threading.Event()
        self.controller = TelloVirtualController(self.main_mock)

    def test_TID1016(self):
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__socket8889'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__tello_address'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__planner'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__stop_event'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__thread_stop_event'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__cm'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__degree'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__renewal_tof_time'))
        self.assertIsNotNone(getattr(self.controller, 'root'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__text_tof'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__text_keyboard'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__btn_landing'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__btn_takeoff'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__keyboard_connection'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__thread_update_tof'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__thread_print_video'))
    
    #test_land
    def test_TID1017(self):
        with patch.object(self.controller, 'send_cmd') as mock_send_cmd:
            self.controller.land()
            mock_send_cmd.assert_called_once_with('land')
    
    #test_on_keypress_q
    def test_TID1018(self):
        with patch.object(self.controller, 'send_cmd') as mock_send_cmd:
            self.controller.on_keypress_q(None)
            mock_send_cmd.assert_called_once_with('stop')
    
    #test_insert_controller_queue
    def test_TID1019(self):
            mock_cmd = "test_command"
            self.controller.insert_controller_queue(mock_cmd)
            self.controller._TelloVirtualController__planner.insert_cmd_queue.assert_called_with(mock_cmd)
    
    #test_send_cmd
    def test_TID1020(self):
        with patch.object(self.controller, 'insert_controller_queue') as mock_insert_controller_queue:
            self.controller.send_cmd('test_cmd')
            mock_insert_controller_queue.assert_any_call('test_cmd')
            mock_insert_controller_queue.assert_any_call('stop')



#===========================================================================================================                     
class TestPlanner(unittest.TestCase):
    
    def setUp(self):
        self.main_mock = MagicMock()
        self.main_mock.stop_event = threading.Event()
        self.main_mock.stop_event.is_set = MagicMock(return_value=False)
        self.main_mock.tello_address = ('192.168.10.1', 8889)
        self.main_mock.socket8889 = MagicMock()
        self.planner = Planner(self.main_mock)
        
    def test_TID1021(self):
        self.assertIsNotNone(self.planner)
        self.assertEqual(self.planner.stop_event, self.main_mock.stop_event)
        self.assertEqual(self.planner.socket8889, self.main_mock.socket8889)
        self.assertEqual(self.planner.tello_address, self.main_mock.tello_address)
        self.assertEqual(self.planner.threshold_distance, 60)
        self.assertEqual(self.planner.base_move_distance, 60)
        self.assertEqual(self.planner.safe_constant, 20)
        self.assertEqual(self.planner._Planner__cmd_queue, [])
        self.assertIsNone(self.planner._Planner__info_8889Sensor_cmd)
        self.assertTrue(self.planner._Planner__thr_stay_connection.is_alive())

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
if __name__ == "__main__":
    unittest.main()
