import time, os, sys, signal
from threading import Timer
from math import pi
import json
import subprocess #241128_KJ
from scipy.spatial.transform import Rotation as R
import numpy as np

# PySide6 modules #
from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import QApplication, QMainWindow, QGraphicsScene, QMessageBox, QWidget
from PySide6.QtCore import QTimer, Slot, QMutex, QThread, Signal, Qt
import PySide6.QtCore as QtCore
from main_window_ui import Ui_MainWindow

# Custum modules #
from ur_state import UR_State
from koras_gripper import KORAS_State
from aidin_FT import aidin_FT

class MainWindow(QMainWindow, QThread):
    debug = False
    period_time = 100
    timer1 = QTimer()
    ur_thread = []
    task_thread = dict()
    task_pause_list = []
    task_bool = False
    task_ongoing = False
    task_last = False
    task_rewind = False
    make_json = True    #True: task.py로 task_planning      False: json에서 직접 수정
    task_rewind_num = 0
    offset_force = [0, 0, 0, 0, 0, 0]
    point_move_mode = ''
    flag_tool_direction = False
    FT = False

    def __init__(self):
        super(MainWindow, self).__init__()
        QThread.__init__(self, parent = None)
        # link function and UI's components #
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.pushButton_33.clicked.connect(self.Btn_UR_Connect)
        self.ui.pushButton_34.clicked.connect(self.UR_Disconnect)
        self.ui.pushButton_14.pressed.connect(lambda: self.moveL_btn_callback('+X'))
        self.ui.pushButton_14.released.connect(lambda: self.moveL_btn_callback('stopL'))
        self.ui.pushButton_28.pressed.connect(lambda: self.moveL_btn_callback('+X'))
        self.ui.pushButton_28.released.connect(lambda: self.moveL_btn_callback('stopL'))
        self.ui.pushButton_27.pressed.connect(lambda: self.moveL_btn_callback('-X'))
        self.ui.pushButton_27.released.connect(lambda: self.moveL_btn_callback('stopL'))
        self.ui.pushButton_19.pressed.connect(lambda: self.moveL_btn_callback('-X'))
        self.ui.pushButton_19.released.connect(lambda: self.moveL_btn_callback('stopL'))
        self.ui.pushButton_26.pressed.connect(lambda: self.moveL_btn_callback('+Y'))
        self.ui.pushButton_26.released.connect(lambda: self.moveL_btn_callback('stopL'))
        self.ui.pushButton_25.pressed.connect(lambda: self.moveL_btn_callback('-Y'))
        self.ui.pushButton_25.released.connect(lambda: self.moveL_btn_callback('stopL'))
        self.ui.pushButton_22.pressed.connect(lambda: self.moveL_btn_callback('+Y'))
        self.ui.pushButton_22.released.connect(lambda: self.moveL_btn_callback('stopL'))
        self.ui.pushButton_24.pressed.connect(lambda: self.moveL_btn_callback('-Y'))
        self.ui.pushButton_24.released.connect(lambda: self.moveL_btn_callback('stopL'))
        self.ui.pushButton_29.pressed.connect(lambda: self.moveL_btn_callback('+Z'))
        self.ui.pushButton_29.released.connect(lambda: self.moveL_btn_callback('stopL'))
        self.ui.pushButton_30.pressed.connect(lambda: self.moveL_btn_callback('-Z'))
        self.ui.pushButton_30.released.connect(lambda: self.moveL_btn_callback('stopL'))
        self.ui.pushButton_18.pressed.connect(lambda: self.moveL_btn_callback('+Z'))
        self.ui.pushButton_18.released.connect(lambda: self.moveL_btn_callback('stopL'))
        self.ui.pushButton_17.pressed.connect(lambda: self.moveL_btn_callback('-Z'))
        self.ui.pushButton_17.released.connect(lambda: self.moveL_btn_callback('stopL'))
        self.ui.pushButton_15.pressed.connect(lambda: self.moveL_btn_callback('+Roll'))
        self.ui.pushButton_15.released.connect(lambda: self.moveL_btn_callback('stopL_A'))
        self.ui.pushButton_13.pressed.connect(lambda: self.moveL_btn_callback('-Roll'))
        self.ui.pushButton_13.released.connect(lambda: self.moveL_btn_callback('stopL_A'))
        self.ui.pushButton_21.pressed.connect(lambda: self.moveL_btn_callback('+Pitch'))
        self.ui.pushButton_21.released.connect(lambda: self.moveL_btn_callback('stopL_A'))
        self.ui.pushButton_23.pressed.connect(lambda: self.moveL_btn_callback('-Pitch'))
        self.ui.pushButton_23.released.connect(lambda: self.moveL_btn_callback('stopL_A'))
        self.ui.pushButton_16.pressed.connect(lambda: self.moveL_btn_callback('+Yaw'))
        self.ui.pushButton_16.released.connect(lambda: self.moveL_btn_callback('stopL_A'))
        self.ui.pushButton_20.pressed.connect(lambda: self.moveL_btn_callback('-Yaw'))
        self.ui.pushButton_20.released.connect(lambda: self.moveL_btn_callback('stopL_A'))
        self.ui.pushButton_2.pressed.connect(lambda: self.moveL_btn_callback('+J1'))
        self.ui.pushButton_2.released.connect(lambda: self.moveL_btn_callback('stopJ'))
        self.ui.pushButton_4.pressed.connect(lambda: self.moveL_btn_callback('+J2'))
        self.ui.pushButton_4.released.connect(lambda: self.moveL_btn_callback('stopJ'))
        self.ui.pushButton_6.pressed.connect(lambda: self.moveL_btn_callback('+J3'))
        self.ui.pushButton_6.released.connect(lambda: self.moveL_btn_callback('stopJ'))
        self.ui.pushButton_8.pressed.connect(lambda: self.moveL_btn_callback('+J4'))
        self.ui.pushButton_8.released.connect(lambda: self.moveL_btn_callback('stopJ'))
        self.ui.pushButton_10.pressed.connect(lambda: self.moveL_btn_callback('+J5'))
        self.ui.pushButton_10.released.connect(lambda: self.moveL_btn_callback('stopJ'))
        self.ui.pushButton_12.pressed.connect(lambda: self.moveL_btn_callback('+J6'))
        self.ui.pushButton_12.released.connect(lambda: self.moveL_btn_callback('stopJ'))
        self.ui.pushButton.pressed.connect(lambda: self.moveL_btn_callback('-J1'))
        self.ui.pushButton.released.connect(lambda: self.moveL_btn_callback('stopJ'))
        self.ui.pushButton_3.pressed.connect(lambda: self.moveL_btn_callback('-J2'))
        self.ui.pushButton_3.released.connect(lambda: self.moveL_btn_callback('stopJ'))
        self.ui.pushButton_5.pressed.connect(lambda: self.moveL_btn_callback('-J3'))
        self.ui.pushButton_5.released.connect(lambda: self.moveL_btn_callback('stopJ'))
        self.ui.pushButton_7.pressed.connect(lambda: self.moveL_btn_callback('-J4'))
        self.ui.pushButton_7.released.connect(lambda: self.moveL_btn_callback('stopJ'))
        self.ui.pushButton_9.pressed.connect(lambda: self.moveL_btn_callback('-J5'))
        self.ui.pushButton_9.released.connect(lambda: self.moveL_btn_callback('stopJ'))
        self.ui.pushButton_11.pressed.connect(lambda: self.moveL_btn_callback('-J6'))
        self.ui.pushButton_11.released.connect(lambda: self.moveL_btn_callback('stopJ'))
        self.ui.pushButton_38.clicked.connect(lambda: self.start_json_task(1))
        self.ui.pushButton_39.clicked.connect(lambda: self.start_json_task(2))
        self.ui.pushButton_40.clicked.connect(lambda: self.start_json_task(3))
        self.ui.pushButton_42.clicked.connect(lambda: self.start_json_task(4))
        self.ui.pushButton_43.clicked.connect(lambda: self.start_json_task(5))
        self.ui.pushButton_41.clicked.connect(lambda: self.start_json_task(6))
        self.ui.pushButton_45.clicked.connect(lambda: self.start_json_task(7))
        self.ui.pushButton_46.clicked.connect(lambda: self.start_json_task(8))
        self.ui.pushButton_44.clicked.connect(lambda: self.start_json_task(9))
        self.ui.pushButton_52.clicked.connect(lambda: self.start_json_task(10))
        self.ui.pushButton_51.clicked.connect(lambda: self.start_json_task(11))
        self.ui.pushButton_50.clicked.connect(lambda: self.start_json_task(12))
        self.ui.pushButton_47.clicked.connect(self.stop_json_task)
        self.ui.pushButton_48.clicked.connect(self.resume_json_task)
        self.ui.pushButton_49.clicked.connect(self.pause_json_task)
        self.ui.pushButton_54.clicked.connect(self.set_force_zero)
        self.ui.pushButton_31.clicked.connect(self.get_q_callback)
        self.ui.pushButton_32.clicked.connect(self.get_tcp_callback)
        self.ui.pushButton_53.clicked.connect(self.gripper_init)
        self.ui.pushButton_56.clicked.connect(self.gripper_open)
        self.ui.pushButton_57.clicked.connect(self.gripper_close)
        self.ui.pushButton_55.clicked.connect(self.gripper_stop)
        self.ui.pushButton_58.clicked.connect(self.gripper_vac_on)
        self.ui.pushButton_59.clicked.connect(self.gripper_vac_off)
        self.ui.pushButton_60.clicked.connect(self.gripper_finger_pos)
        self.ui.pushButton_58.clicked.connect(self.gripper_vac_on)
        self.ui.pushButton_59.clicked.connect(self.gripper_vac_off)
        self.ui.pushButton_60.clicked.connect(self.gripper_finger_pos)
        self.ui.pushButton_61.clicked.connect(lambda: self.start_json_task(13))
        self.ui.pushButton_62.clicked.connect(lambda: self.start_json_task(14))
        self.ui.pushButton_63.clicked.connect(lambda: self.start_json_task(15))
        self.ui.pushButton_64.clicked.connect(self.relative_move)
        self.ui.pushButton_65.clicked.connect(self.setRobotTCP)
        self.ui.pushButton_66.clicked.connect(self.point_move) 
        self.ui.pushButton_78.clicked.connect(lambda: self.start_json_task(16))
        self.ui.pushButton_67.clicked.connect(lambda: self.start_json_task(17))
        self.ui.pushButton_72.clicked.connect(lambda: self.start_json_task(18))
        self.ui.pushButton_71.clicked.connect(lambda: self.start_json_task(19))
        self.ui.pushButton_68.clicked.connect(lambda: self.start_json_task(20))
        self.ui.pushButton_70.clicked.connect(lambda: self.start_json_task(21))
        self.ui.pushButton_74.clicked.connect(lambda: self.start_json_task(22))
        self.ui.pushButton_69.clicked.connect(lambda: self.start_json_task(23))
        self.ui.pushButton_77.clicked.connect(lambda: self.start_json_task(24))
        self.ui.pushButton_73.clicked.connect(lambda: self.start_json_task(25))
        self.ui.pushButton_75.clicked.connect(lambda: self.start_json_task(26))
        self.ui.pushButton_76.clicked.connect(lambda: self.start_json_task(27))
        self.ui.pushButton_81.clicked.connect(lambda: self.start_json_task(28))
        self.ui.pushButton_79.clicked.connect(lambda: self.start_json_task(29))
        self.ui.pushButton_80.clicked.connect(lambda: self.start_json_task(30))
        self.start() # start QThread
        self.startTimer()
                
    def keyPressEvent(self, e):
        if e.key() == Qt.Key_S:
            self.stop_json_task()
        elif e.key() == Qt.Key_W:
            value = self.ui.spinBox.value() / 1000.0
            self.rel_moveL([0.0, value, 0.0, 0.0, 0.0, 0.0], vel=0.1, acc=0.5)
        elif e.key() == Qt.Key_X:
            value = self.ui.spinBox.value() / 1000.0
            self.rel_moveL([0.0, -value, 0.0, 0.0, 0.0, 0.0], vel=0.1, acc=0.5)
        elif e.key() == Qt.Key_A:
            value = self.ui.spinBox.value() / 1000.0
            self.rel_moveL([-value, 0.0, 0.0, 0.0, 0.0, 0.0], vel=0.1, acc=0.5)
        elif e.key() == Qt.Key_D:
            value = self.ui.spinBox.value() / 1000.0
            self.rel_moveL([value, 0.0, 0.0, 0.0, 0.0, 0.0], vel=0.1, acc=0.5)
        elif e.key() == Qt.Key_Q:
            value = self.ui.spinBox.value() / 1000.0
            self.rel_moveL([0.0, 0.0, -value, 0.0, 0.0, 0.0], vel=0.1, acc=0.5)
        elif e.key() == Qt.Key_E:
            value = self.ui.spinBox.value() / 1000.0
            self.rel_moveL([0.0, 0.0, value, 0.0, 0.0, 0.0], vel=0.1, acc=0.5)
        elif e.key() == Qt.Key_F:
            target_pos = self.ur1.joint_state['actualQ']
            # joint_deg = self.ur1.getForwardKinematics(target_pos)
            target_pos = self.ur1.tcp_state['actualPos']
            joint_deg = self.rad2deg_list(self.ur1.getInverseKinematics(target_pos))
            print(self.rad2deg_list(self.ur1.joint_state['actualQ']))
            print(joint_deg)
            
    def keyReleaseEvent(self, event):
        # print("abc")
        return super().keyReleaseEvent(event)
            
    def point_move(self):
        mode = self.point_move_mode
        target = eval(self.ui.lineEdit_25.text())
        
        if type(target) != list:
            print("Error: target is not list")
            return
        
        if mode == 'q':
            target_rad = self.deg2rad_list(target)
            self.abs_moveJ(target_rad)
            print("MoveJ", target)
        elif mode == 'x':
            self.abs_moveL(target)
            print("MoveL:", target)
        pass
            
    def relative_move(self):
        axis = self.ui.textEdit_2.toPlainText()
        value = self.ui.spinBox.value() / 1000.0
        
        if axis == 'x':
            self.rel_moveL([value, 0.0, 0.0, 0.0, 0.0, 0.0], vel=0.05, acc=0.1)
        elif axis == 'y':
            self.rel_moveL([0.0, value, 0.0, 0.0, 0.0, 0.0], vel=0.05, acc=0.1)
        elif axis == 'z':
            self.rel_moveL([0.0, 0.0, value, 0.0, 0.0, 0.0], vel=0.05, acc=0.1)
    def setRobotTCP(self):
        x_offset = self.ui.spinBox_2.value() / 1000.0
        y_offset = self.ui.spinBox_3.value() / 1000.0
        z_offset = self.ui.spinBox_4.value() / 1000.0
        self.ur1.setTcp([x_offset, y_offset, z_offset, 0.0, 0.0, 0.0])
        # print("[TCP Set]:", 1000*x_offset, 1000*y_offset, 1000*z_offset, " mm")
        get_offset = self.ur1.getTCPOffset()
        print("[TCP Get]:", 1000*get_offset[0], 1000*get_offset[1], 1000*get_offset[2], " mm")
            
    def BTN1_callback(self):
        t1 = self.deg2rad_list([28.333, -54.769, -119.343, -95.86, 90.287, 118.317])
        t1.append(0.2)
        t1.append(0.4)
        t1.append(0.0)
        t2 = self.deg2rad_list([-38.746, -94.711, -87.784, -87.494, 90.327, 51.261])
        t2.append(0.2)
        t2.append(0.4)
        t2.append(0.02)
        t3 = self.deg2rad_list([-38.796, -111.032, -114.687, -44.266, 90.316, 51.465])
        t3.append(0.2)
        t3.append(0.4)
        t3.append(0.0)
        t = [t1, t2, t3]
        print(type(t))
        self.ur1.rtde_c.moveJ(t)
    def messageBox(self, message, title='Message Win'):
        if type(message) != str:
            message = str(message)
        dlg = QMessageBox(self)
        dlg.setWindowTitle(title)
        dlg.setText(message)
        button = dlg.exec()
        if button == QMessageBox.Ok:
            print("OK!")
    def moveL_btn_callback(self, dir):
        self.ur_thread.append(dir)
    def get_q_callback(self):
        self.point_move_mode = 'q'
        q = self.ur1.joint_state['actualQ']
        text = str(self.round_list(self.rad2deg_list(q),3))
        self.ui.lineEdit_25.setText(text)
    def get_tcp_callback(self):
        self.point_move_mode = 'x'
        tcp = str(self.round_list(self.ur1.tcp_state['actualPos'],3))
        self.ui.lineEdit_25.setText(tcp)
    def rad2deg_list(self, l):
        temp = []
        length = len(l)
        for i in range(length):
            temp.append(l[i]*180.0/pi)
        return temp
    def deg2rad_list(self, l):
        temp = []
        length = len(l)
        for i in range(length):
            temp.append(l[i]*pi/180.0)
        return temp
    def deg2rad_path_list(self, path):
        retVal = path
        length = len(path)
        for i in range(length):
            for j in range(6):
                temp = retVal[i][j]*(pi/180.0)
                retVal[i][j] = temp
        return retVal
    def round_list(self, l, num=3):
        temp = []
        length = len(l)
        for i in range(length):
            temp.append(round(l[i], num))
        return temp
    def gripper_init(self):
        self.grp1.initialize()
    def gripper_open(self):
        self.grp1.open()
    def gripper_close(self):
        self.grp1.close()
    def gripper_stop(self):
        self.grp1.motor_stop()
    def gripper_vac_on(self):
        self.grp1.vacuum_on()
    def gripper_vac_off(self):
        self.grp1.vacuum_off()
    def gripper_finger_pos(self):
        pos = 10*self.ui.horizontalScrollBar.value()
        self.grp1.finger_pos_set(pos)
    def gripper_connect(self):
        self.grp1 = KORAS_State()
        self.grp1.start_client()
    def FT_sensor_connect(self):
        self.FT_sensor1 = aidin_FT()
        self.FT_sensor1.start_client()        
    def UR_Connect(self):
        self.ur1 = UR_State("192.168.0.12")
        self.ur1.start()
    def Btn_UR_Connect(self):
        if self.ur1.connection:
            print("Already connected.")
        else:
            self.ur1 = UR_State("192.168.0.12")
    def UR_Disconnect(self):
        if self.ur1.connection:
            self.ur1.disconnect()
        else:
            print("Already disconnected.")
        
    def set_force_zero(self):
        self.ur1.readUR()
        self.offset_force = self.ur1.tcp_state['actualForce']
        # return self.ur1.zeroFtSensor()
        if self.FT:
            self.FT_sensor1.set_bias()        
    def alarm_release(self):
        if self.debug:
            print('Alarm_Release_callback')
    def getLineEditText(self):
        value = self.ui.lineEdit_26.text()
    def setURStatus(self):
        emergency = self.ur1.stop_state['EmergencyStopped']
        protective = self.ur1.stop_state['ProtectiveStopped']
        robot_mode = self.ur1.robot_state['RobotMode']
        robot_status = self.ur1.robot_state['RobotStatus']
        safety_mode = self.ur1.robot_state['SafetyMode']
        self.ui.lineEdit_26.setText(str(emergency))
        self.ui.lineEdit_29.setText(str(protective))
        self.ui.lineEdit_27.setText(robot_mode)
        self.ui.lineEdit_28.setText(str(robot_status))
        self.ui.lineEdit_30.setText(safety_mode)
    def setActualQ(self):
        q = self.ur1.joint_state['actualQ']
        self.ui.lineEdit.setText(str(round(q[0]*180/pi,2)) + ' deg')
        self.ui.lineEdit_2.setText(str(round(q[1]*180/pi,2)) + ' deg')
        self.ui.lineEdit_3.setText(str(round(q[2]*180/pi,2)) + ' deg')
        self.ui.lineEdit_4.setText(str(round(q[3]*180/pi,2)) + ' deg')
        self.ui.lineEdit_5.setText(str(round(q[4]*180/pi,2)) + ' deg')
        self.ui.lineEdit_6.setText(str(round(q[5]*180/pi,2)) + ' deg')
    def setActualPos(self):
        pos = self.ur1.tcp_state['actualPos']
        self.ui.lineEdit_12.setText(str(round(pos[0],4)) + ' m')
        self.ui.lineEdit_11.setText(str(round(pos[1],4)) + ' m')
        self.ui.lineEdit_10.setText(str(round(pos[2],4)) + ' m')
        self.ui.lineEdit_9.setText(str(round(pos[3],3)) + ' rad')
        self.ui.lineEdit_8.setText(str(round(pos[4],3)) + ' rad')
        self.ui.lineEdit_7.setText(str(round(pos[5],3)) + ' rad')
    def setActualVel(self):
        vel = self.ur1.tcp_state['actualSpeed']
        self.ui.lineEdit_13.setText(str(round(vel[0],4)))
        self.ui.lineEdit_14.setText(str(round(vel[1],4)))
        self.ui.lineEdit_15.setText(str(round(vel[2],4)))
        self.ui.lineEdit_16.setText(str(round(vel[3],4)))
        self.ui.lineEdit_17.setText(str(round(vel[4],4)))
        self.ui.lineEdit_18.setText(str(round(vel[5],4)))
    def setActualForce(self):
        force = self.ur1.tcp_state['actualForce']
        self.ui.lineEdit_19.setText(str(round(force[0] - self.offset_force[0],1)))
        self.ui.lineEdit_20.setText(str(round(force[1] - self.offset_force[1],1)))
        self.ui.lineEdit_21.setText(str(round(force[2] - self.offset_force[2],1)))
        self.ui.lineEdit_22.setText(str(round(force[3] - self.offset_force[3],2)))
        self.ui.lineEdit_23.setText(str(round(force[4] - self.offset_force[4],2)))
        self.ui.lineEdit_24.setText(str(round(force[5] - self.offset_force[5],2)))
    def rel_moveL(self, pos, vel = 0.25, acc = 0.5, asynchronous = True):
        if self.debug:
            time.sleep(1.0)
        else:
            target_pos = self.sum_list(pos, self.ur1.tcp_state['actualPos'])
            self.ur1.moveL(target_pos, vel, acc, asynchronous)
    def abs_moveL(self, pos, vel = 0.25, acc = 0.5, asynchronous = True):
        if self.debug:
            time.sleep(1.0)
        else:
            self.ur1.moveL(pos, vel, acc, asynchronous)
    def abs_moveL_path(self, q, qd = 1.05, qdd = 1.4, asynchronous = True):
        self.ur1.rtde_c.moveL(q, asynchronous=True)
    def rel_moveJ(self, q, qd = 1.05, qdd = 1.4, asynchronous = True):
        target_q = self.sum_list(q, self.ur1.joint_state['actualQ'])
        self.ur1.moveJ(target_q, qd, qdd, asynchronous)
    def abs_moveJ(self, q, qd = 1.05, qdd = 1.4, asynchronous = True):
        self.ur1.moveJ(q, qd, qdd, asynchronous)
    def abs_moveJ_path(self, q, qd = 1.05, qdd = 1.4, asynchronous = True):
        self.ur1.rtde_c.moveJ(q, asynchronous=True)
    def tool_moveL(self, tool_vector, vel = 0.25, acc = 0.5, asynchronous = True):
        base_pos = np.array(self.ur1.tcp_state['actualPos'])
        tool_direction = (R.from_euler('xyz', base_pos[3:]).as_matrix()) @ (np.array(tool_vector))
        target_pos = base_pos + np.array([tool_direction[0], tool_direction[1], tool_direction[2], 0.0, 0.0, 0.0])
        self.abs_moveL(target_pos, vel = vel, acc = acc, asynchronous = asynchronous)
    def tool_speedL(self, tool_vector):
        base_pos = np.array(self.ur1.tcp_state['actualPos'])
        tool_direction = (R.from_euler('xyz', base_pos[3:]).as_matrix()) @ (np.array(tool_vector))
        target_pos = np.array([tool_direction[0], tool_direction[1], tool_direction[2], 0.0, 0.0, 0.0])
        return target_pos
    def sum_list(self, i, j):
        retVal = list()
        l1 = len(i)
        l2 = len(j)
        if l1 == l2:
            for index in range(l1):
                retVal.append(i[index] + j[index])
            return retVal
        else:
            return
    ## Definition of Timer functions ##
    def startTimer(self):
        self.timer1.setInterval(self.period_time) # Interval time
        self.timer1.timeout.connect(self.timer1_callback) # Set callback function
        self.timer1.start() # Start
    def stopTimer(self):
        self.timer1.stop()
    def timer1_callback(self):
        if self.ur1.connection:
            self.ur1.readUR()
            self.setURStatus()
            self.setActualQ()
            self.setActualPos()
            self.setActualVel()
            self.setActualForce()
            self.ui.checkBox_2.setChecked(True)
        else:
            self.ui.checkBox_2.setChecked(False)
            
        if self.grp1.connection:
            temp = self.ui.horizontalScrollBar.value()
            self.ui.doubleSpinBox_3.setValue(temp)
            self.ui.lineEdit_33.setText(str(round(self.grp1.finger_pos/10,1)) + " %")
            self.ui.lineEdit_34.setText(str(self.grp1.motor_cur) + " mA")
    def run(self):
        self.UR_Connect()
        self.gripper_connect()
        if self.FT:
            self.FT_sensor_connect()
        while True:
            try:
                self.do_move_btn_task()
                self.do_json_task()
            except KeyboardInterrupt:
                self.disconnect()
                self.grp1.tcp_close()
                print("\nDisconnect the communication.")
                
    def do_move_btn_task(self):
        if len(self.ur_thread) != 0:
            command = self.ur_thread[0]
            moveL_speed = self.ui.doubleSpinBox_2.value()
            moveL_acc = 2.0*moveL_speed
            moveJ_speed = (self.ui.doubleSpinBox.value())*pi/180
            moveJ_acc = 1.2*moveJ_speed
            if self.ui.checkBox.isChecked():
                if   command == '+X':     self.ur1.speedL(self.tool_speedL([moveL_speed, 0, 0]), acceleration=moveL_acc, time=0.01)
                elif command == '-X':     self.ur1.speedL(self.tool_speedL([-moveL_speed, 0, 0]), acceleration=moveL_acc, time=0.01)
                elif command == '+Y':     self.ur1.speedL(self.tool_speedL([0, moveL_speed, 0]), acceleration=moveL_acc, time=0.01)
                elif command == '-Y':     self.ur1.speedL(self.tool_speedL([0, -moveL_speed, 0]), acceleration=moveL_acc, time=0.01)
                elif command == '+Z':     self.ur1.speedL(self.tool_speedL([0, 0, moveL_speed]), acceleration=moveL_acc, time=0.01)                  
                elif command == '-Z':     self.ur1.speedL(self.tool_speedL([0, 0, -moveL_speed]), acceleration=moveL_acc, time=0.01)
            else:
                if   command == '+X':     self.ur1.speedL([moveL_speed, 0, 0, 0, 0, 0], acceleration=moveL_acc, time=0.01)
                elif command == '-X':     self.ur1.speedL([-moveL_speed, 0, 0, 0, 0, 0], acceleration=moveL_acc, time=0.01)
                elif command == '+Y':     self.ur1.speedL([0, moveL_speed, 0, 0, 0, 0], acceleration=moveL_acc, time=0.01)
                elif command == '-Y':     self.ur1.speedL([0, -moveL_speed, 0, 0, 0, 0], acceleration=moveL_acc, time=0.01)
                elif command == '+Z':     self.ur1.speedL([0, 0, moveL_speed, 0, 0, 0], acceleration=moveL_acc, time=0.01)
                elif command == '-Z':     self.ur1.speedL([0, 0, -moveL_speed, 0, 0, 0], acceleration=moveL_acc, time=0.01)

            if   command == '+Roll':  self.ur1.speedL([0, 0, 0, pi*moveL_speed, 0, 0], acceleration=10*moveL_acc, time=0.01)
            elif command == '-Roll':  self.ur1.speedL([0, 0, 0, -pi*moveL_speed, 0, 0], acceleration=10*moveL_acc, time=0.01)
            elif command == '+Pitch': self.ur1.speedL([0, 0, 0, 0, pi*moveL_speed, 0], acceleration=10*moveL_acc, time=0.01)
            elif command == '-Pitch': self.ur1.speedL([0, 0, 0, 0, -pi*moveL_speed, 0], acceleration=10*moveL_acc, time=0.01)
            elif command == '+Yaw':   self.ur1.speedL([0, 0, 0, 0, 0, pi*moveL_speed], acceleration=10*moveL_acc, time=0.01)
            elif command == '-Yaw':   self.ur1.speedL([0, 0, 0, 0, 0, -pi*moveL_speed], acceleration=10*moveL_acc, time=0.01)
            elif command == 'stopL':  self.ur1.speedStop(3.0*moveL_speed)
            elif command == 'stopL_A':  self.ur1.speedStop(15*moveL_speed)
            elif command == '+J1': self.ur1.speedJ([moveJ_speed, 0, 0, 0, 0, 0], acceleration=moveJ_acc, time=0.01)
            elif command == '+J2': self.ur1.speedJ([0, moveJ_speed, 0, 0, 0, 0], acceleration=moveJ_acc, time=0.01)
            elif command == '+J3': self.ur1.speedJ([0, 0, moveJ_speed, 0, 0, 0], acceleration=moveJ_acc, time=0.01)
            elif command == '+J4': self.ur1.speedJ([0, 0, 0, moveJ_speed, 0, 0], acceleration=moveJ_acc, time=0.01)
            elif command == '+J5': self.ur1.speedJ([0, 0, 0, 0, moveJ_speed, 0], acceleration=moveJ_acc, time=0.01)
            elif command == '+J6': self.ur1.speedJ([0, 0, 0, 0, 0, moveJ_speed], acceleration=moveJ_acc, time=0.01)
            elif command == '-J1': self.ur1.speedJ([-moveJ_speed, 0, 0, 0, 0, 0], acceleration=moveJ_acc, time=0.01)
            elif command == '-J2': self.ur1.speedJ([0, -moveJ_speed, 0, 0, 0, 0], acceleration=moveJ_acc, time=0.01)
            elif command == '-J3': self.ur1.speedJ([0, 0, -moveJ_speed, 0, 0, 0], acceleration=moveJ_acc, time=0.01)
            elif command == '-J4': self.ur1.speedJ([0, 0, 0, -moveJ_speed, 0, 0], acceleration=moveJ_acc, time=0.01)
            elif command == '-J5': self.ur1.speedJ([0, 0, 0, 0, -moveJ_speed, 0], acceleration=moveJ_acc, time=0.01)
            elif command == '-J6': self.ur1.speedJ([0, 0, 0, 0, 0, -moveJ_speed], acceleration=moveJ_acc, time=0.01)
            elif command == 'stopJ': self.ur1.speedStop(1.5*moveJ_speed)
            del self.ur_thread[0]
        else:
            time.sleep(0.01)            
    def execute_script(self,task_file_dir, task_name): #241128_KJ
        """
        지정된 디렉토리에서 파일을 실행하는 함수.
        :param directory: 스크립트가 있는 디렉토리 경로
        :param filename: 실행할 Python 파일 이름
        """
        # 파일 경로 생성
        selected_file_path = task_file_dir
        # 파일 실행
        print(f"\nExecuting task.py\n")

        try:
            subprocess.run(["python3", selected_file_path, task_name], check=True)
        except subprocess.CalledProcessError as e:
            print(f"Error while executing task.py: {e}")
        except FileNotFoundError:
            print("Python is not found. Ensure Python is installed and available in PATH.")
        finally:
            print("Task.py successfully executed.")
    #241128_KJ
    def start_json_task(self, task_num):
        if self.task_ongoing:
            self.messageBox("Task alreaday ongoing, Try again!")
            return
        self.task_rewind_num = task_num

        if self.make_json:    
            cwd = os.getcwd()       
            task_file_dir = cwd + "/task_list/task.py"
            task_name = "task_" + str(task_num)
            self.execute_script(task_file_dir, task_name)

        task_file = './task_list/task_' + str(task_num) + '.json'
        with open(task_file) as f:
            self.task_thread = json.load(f)
        print("*** Task Start: ", self.task_thread["title"], "***")
    def pause_json_task(self):
        self.task_bool = False
        # if self.task_thread["task_list"][0]["task"] == "Linear_relative" or self.task_thread["task_list"][0]["task"] == "Linear_absolute":
        #     self.ur1.stopL(1.5)
        # else:
        #     self.ur1.stopL(1.5)
        self.ur1.stopJ()
        self.ur1.stopL(1.5)
        self.ur1.speedStop(1.5)
        self.task_pause_list = self.task_thread["task_list"]
        self.task_thread["task_list"] = []
        print("Paused Task")
    def resume_json_task(self):
        self.task_bool = False
        self.task_thread["task_list"] = self.task_pause_list
        print("Resume Task")
    def stop_json_task(self):
        self.task_bool = False
        self.task_thread["task_list"] = []
        self.task_pause_list = []
        self.ur1.stopJ()
        self.ur1.stopL(1.5)
        self.ur1.speedStop(1.5)
        time.sleep(0.01)
        print("Task Stopped!")
        pass
    def do_json_task(self):
        if "task_list" in self.task_thread:
            if len(self.task_thread["task_list"]) == 1:
                self.task_last = True
            if len(self.task_thread["task_list"]) == 0:
                self.task_ongoing = False
                if self.task_last == True:
                    print("*** Task finished! ***")
                    self.task_last = False
                if self.task_rewind:
                    self.start_json_task(self.task_rewind_num)
                    self.task_rewind = False
                    print("** Task Restart! ***")
                time.sleep(0.01)
            else:
                # self.task_last = False
                self.task_ongoing = True
                task = self.task_thread["task_list"][0]["task"]
                target = self.task_thread["task_list"][0]["target"]
                ### Case 1: Linear & Relative motion ###
                if task == "Linear_relative":
                    if self.task_bool == False:
                        print("[UR MoveL_REL]: ", target)
                        if "vel_acc" in self.task_thread["task_list"][0]:
                            vel = self.task_thread["task_list"][0]["vel_acc"][0]
                            acc = self.task_thread["task_list"][0]["vel_acc"][1]
                            self.rel_moveL(target, vel=vel, acc=acc)
                        else:
                            self.rel_moveL(target)
                        self.task_bool = True
                    else:
                        time.sleep(0.01)
                    if self.ur1.isSteady() and self.task_bool:
                        self.task_bool = False
                        del self.task_thread["task_list"][0]
                ### Case 2: Linear & Absolute motion ###
                elif task == "Linear_absolute":
                    if self.task_bool == False:
                        print("[UR MoveL ABS]: ", target)
                        if "vel_acc" in self.task_thread["task_list"][0]:
                            vel = self.task_thread["task_list"][0]["vel_acc"][0]
                            acc = self.task_thread["task_list"][0]["vel_acc"][1]
                            self.abs_moveL(target, vel=vel, acc=acc)
                        else:
                            self.abs_moveL(target)
                        self.task_bool = True
                    else:
                        time.sleep(0.01)
                    if self.ur1.isSteady() and self.task_bool:
                        self.task_bool = False
                        del self.task_thread["task_list"][0]
                elif task == "Tool_Linear":
                    if self.task_bool == False:
                        print("[UR Tool MoveL]: ", target)
                        if "vel_acc" in self.task_thread["task_list"][0]:
                            vel = self.task_thread["task_list"][0]["vel_acc"][0]
                            acc = self.task_thread["task_list"][0]["vel_acc"][1]
                            self.tool_moveL(target[:3], vel=vel, acc=acc)
                        else:
                            self.tool_moveL(target[:3])
                        self.task_bool = True
                    else:
                        time.sleep(0.01)
                    if self.ur1.isSteady() and self.task_bool:
                        self.task_bool = False
                        del self.task_thread["task_list"][0]
                ### Case 3: Linear path
                elif task == "Linear_path":
                    if self.task_bool == False:
                        print("[UR MoveL PATH]: No. cs pos =", len(target))
                        self.abs_moveL_path(target)
                        self.task_bool = True
                    else:
                        time.sleep(0.01)
                    if self.ur1.isSteady() and self.task_bool:
                        self.task_bool = False
                        del self.task_thread["task_list"][0]
                ### Case 3: Joint & Relative motion ###
                elif task == "Joint_relative":
                    if self.task_bool == False:
                        print("[UR MoveJ REL]: ", target)
                        if "vel_acc" in self.task_thread["task_list"][0]:
                            velocity = self.task_thread["task_list"][0]["vel_acc"][0]
                            acceleration = self.task_thread["task_list"][0]["vel_acc"][1]
                            taeget_rad = self.deg2rad_list(target)
                            self.rel_moveJ(taeget_rad, qd=velocity, qdd=acceleration)
                        else:
                            taeget_rad = self.deg2rad_list(target)
                            self.rel_moveJ(taeget_rad)
                        self.task_bool = True
                    else:
                        time.sleep(0.01)
                    if self.ur1.isSteady() and self.task_bool:
                        self.task_bool = False
                        del self.task_thread["task_list"][0]
                ### Case 4: Joint & Absolute motion ###    
                elif task == "Joint_absolute":
                    if self.task_bool == False:
                        print("[UR MoveJ ABS]: ", target)
                        if "vel_acc" in self.task_thread["task_list"][0]:
                            velocity = self.task_thread["task_list"][0]["vel_acc"][0]
                            acceleration = self.task_thread["task_list"][0]["vel_acc"][1]
                            taeget_rad = self.deg2rad_list(target)
                            self.abs_moveJ(taeget_rad, qd=velocity, qdd=acceleration)
                        else:
                            taeget_rad = self.deg2rad_list(target)
                            self.abs_moveJ(taeget_rad)
                        self.task_bool = True
                    else:
                        time.sleep(0.01)
                    if self.ur1.isSteady() and self.task_bool:
                        self.task_bool = False
                        del self.task_thread["task_list"][0]
                ### Case 4*: Joint path motion ###    
                elif task == "Joint_path":
                    if self.task_bool == False:
                        print("[UR MoveJ PATH]: No. joint pos =", len(target))
                        taeget_rad = self.deg2rad_path_list(target)
                        self.abs_moveJ_path(taeget_rad)
                        self.task_bool = True
                    else:
                        time.sleep(0.01)
                    if self.ur1.isSteady() and self.task_bool:
                        self.task_bool = False
                        del self.task_thread["task_list"][0]
                ### Case 5: Delay function ###
                elif task == "Delay":
                    if self.task_bool == False:
                        print('[DELAY]: ', target, 'ms')
                        self.start_time = time.time()
                        self.task_bool = True
                    else:
                        time.sleep(0.01)
                    if (time.time() - self.start_time) > (target/1000.0):
                        self.task_bool = False
                        del self.task_thread["task_list"][0]
                ### Case 6: Rewind function ###
                elif task == "Rewind":
                    print('[REWIND TASK] ->', self.task_rewind_num)
                    self.task_rewind = True
                    del self.task_thread["task_list"][0]
                    time.sleep(0.2)
                ### Case 7: Gripper Initialize ###
                elif task == "Gripper_init":
                    self.grp1.initialize()
                    time.sleep(0.1)
                    del self.task_thread["task_list"][0]
                ### Case 7: Gripper Initialize 2 ###
                elif task == "Gripper_init2":
                    self.grp1.initialize2(target)
                    time.sleep(0.1)
                    del self.task_thread["task_list"][0]
                ### Case 8: Gripper Open ###
                elif task == "Gripper_open":
                    self.grp1.open()
                    time.sleep(0.1)
                    del self.task_thread["task_list"][0]
                ### Case 9: Gripper Close ###
                elif task == "Gripper_close":
                    self.grp1.close()
                    time.sleep(0.1)
                    del self.task_thread["task_list"][0]
                ### Case 10: Gripper Finger Pos ###
                elif task == "Gripper_finger_pos":
                    finger_pos = target
                    self.grp1.finger_pos_set(finger_pos)
                    time.sleep(0.1)
                    del self.task_thread["task_list"][0]
                elif task == "Motor_pos":
                    motor_pos = target
                    self.grp1.motor_pos_set(motor_pos)
                    time.sleep(0.1)
                    del self.task_thread["task_list"][0]
                ### Case 11: Vacuum On ###
                elif task == "Vacuum_on":
                    self.grp1.vacuum_on()
                    time.sleep(0.1)
                    del self.task_thread["task_list"][0]
                ### Case 12: Gripper Off ###
                elif task == "Vacuum_off":
                    self.grp1.vacuum_off()
                    time.sleep(0.1)
                    del self.task_thread["task_list"][0]
        
class UR_State_Debug():
    def __init__(self):
        self.steady = False
    def isSteady(self):
        self.steady = True
        return self.steady

        
def signal_handler(signal, frame):
    os._exit(-1)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    app = QApplication()
    window = MainWindow()
    window.show()
    app.exec()
