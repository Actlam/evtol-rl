#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sensor_msgs
import yaml
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
from std_msgs.msg import Int64
from std_srvs.srv import Empty
from sensor_msgs.msg import NavSatFix

import random
import copy
import numpy as np
import time

# global変数
init_lat = 0
init_long = 0
hov_wat = 0
pus_wat = 0
current_seq = 0 # 最新のシークエンス
prev_seq = 0 # current_seqより1つ前のシークエンス
new_episode_flag = True # 1つ前のエピソードの最終シークエンスを扱う変数
hov_wat_total = []
pus_wat_total = []
hov_wat_area = []
pus_wat_area = []
total_wat_pwm = []
r_list = []
end_sw = False
arm_state = True
episode = 1

push_wp_list = []

wp_ = [[[  0,  0, 30, 0], [1,2,3]], # start  　
      [[  0,200, 30, 0], [4]],
      [[150,100, 30, 0], [4]],
      [[300,  0, 30, 1], [4]],
      [[300,200, 30, 1], []]] # goal

# ウェイポイント
wp_ = [[[  0,  0, 30, 0], [1,2,3,4,5,6,7,8,9]], # start  　
      [[  0,200, 30, 0], [10]],
      [[  0,200, 40, 0], [10]],
      [[  0,200, 50, 0], [10]],
      [[150,100, 30, 0], [10]],
      [[150,100, 40, 0], [10]],
      [[150,100, 50, 0], [10]],
      [[300,  0, 30, 1], [10]],
      [[300,  0, 40, 1], [10]],
      [[300,  0, 50, 1], [10]],
      [[300,200, 40, 1], [999]]] # goal

wp = [[[  0,  0, 30, 0], [3,4,5,18,19,20,21,22,23,63,64,65,78,79,80,81,82,83]],
       [[  0,  0, 40, 0], [3,4,5,18,19,20,21,22,23,63,64,65,78,79,80,81,82,83]], # Start
       [[  0,  0, 50, 0], [3,4,5,18,19,20,21,22,23,63,64,65,78,79,80,81,82,83]],
       [[100,  0, 30, 0], [6,7,8,21,22,23,24,25,26,66,67,68,81,82,83,84,85,86]],
       [[100,  0, 40, 0], [6,7,8,21,22,23,24,25,26,66,67,68,81,82,83,84,85,86]],
       [[100,  0, 50, 0], [6,7,8,21,22,23,24,25,26,66,67,68,81,82,83,84,85,86]],
       [[200,  0, 30, 0], [9,10,11,24,25,26,27,28,29,69,70,71,84,85,86,87,88,89]],
       [[200,  0, 40, 0], [9,10,11,24,25,26,27,28,29,69,70,71,84,85,86,87,88,89]],
       [[200,  0, 50, 0], [9,10,11,24,25,26,27,28,29,69,70,71,84,85,86,87,88,89]],
       [[300,  0, 30, 0], [12,13,14,27,28,29,72,73,74,87,88,89]],
       [[300,  0, 40, 0], [12,13,14,27,28,29,72,73,74,87,88,89]],
       [[300,  0, 50, 0], [12,13,14,27,28,29,72,73,74,87,88,89]],
       [[400,  0, 30, 0], [15,16,17,75,76,77]],
       [[400,  0, 40, 0], [15,16,17,75,76,77]],
       [[400,  0, 50, 0], [15,16,17,75,76,77]],
       [[500,  0, 30, 0], [30,31,32,90,91,92]],
       [[500,  0, 40, 0], [30,31,32,90,91,92]],
       [[500,  0, 50, 0], [30,31,32,90,91,92]],
       [[  0,100, 30, 0], [21,22,23,33,34,35,36,37,38,81,82,83,93,94,95,96,97,98]],
       [[  0,100, 40, 0], [21,22,23,33,34,35,36,37,38,81,82,83,93,94,95,96,97,98]],
       [[  0,100, 50, 0], [21,22,23,33,34,35,36,37,38,81,82,83,93,94,95,96,97,98]],
       [[100,100, 30, 0], [24,25,26,36,37,38,84,85,86,96,97,98]],
       [[100,100, 40, 0], [24,25,26,36,37,38,84,85,86,96,97,98]],
       [[100,100, 50, 0], [24,25,26,36,37,38,84,85,86,96,97,98]],
       [[200,100, 30, 0], [27,28,29,39,40,41,87,88,89,99,100,101]],
       [[200,100, 40, 0], [27,28,29,39,40,41,87,88,89,99,100,101]],
       [[200,100, 50, 0], [27,28,29,39,40,41,87,88,89,99,100,101]],
       [[300,100, 30, 0], [39,40,41,42,43,44,99,100,101,102,103,104]],
       [[300,100, 40, 0], [39,40,41,42,43,44,99,100,101,102,103,104]],
       [[300,100, 50, 0], [39,40,41,42,43,44,99,100,101,102,103,104]],
       [[500,100, 30, 0], [45,46,47,105,106,107]],
       [[500,100, 40, 0], [45,46,47,105,106,107]],
       [[500,100, 50, 0], [45,46,47,105,106,107]],
       [[  0,200, 30, 0], [36,37,38,96,97,98]],
       [[  0,200, 40, 0], [36,37,38,96,97,98]],
       [[  0,200, 50, 0], [36,37,38,96,97,98]],
       [[100,200, 30, 0], [48,49,50,108,109,110]],
       [[100,200, 40, 0], [48,49,50,108,109,110]],
       [[100,200, 50, 0], [48,49,50,108,109,110]],
       [[300,200, 30, 0], [42,43,44,51,52,53,54,55,56,102,103,104,111,112,113,114,115,116]],
       [[300,200, 40, 0], [42,43,44,51,52,53,54,55,56,102,103,104,111,112,113,114,115,116]],
       [[300,200, 50, 0], [42,43,44,51,52,53,54,55,56,102,103,104,111,112,113,114,115,116]],
       [[400,200, 30, 0], [45,46,47,54,55,56,57,58,59,105,106,107,114,115,116,117,118,119]],
       [[400,200, 40, 0], [45,46,47,54,55,56,57,58,59,105,106,107,114,115,116,117,118,119]],
       [[400,200, 50, 0], [45,46,47,54,55,56,57,58,59,105,106,107,114,115,116,117,118,119]],
       [[500,200, 30, 0], [57,58,59,117,118,119]],
       [[500,200, 40, 0], [57,58,59,117,118,119]],
       [[500,200, 50, 0], [57,58,59,117,118,119]],
       [[200,300, 30, 0], [51,52,53,111,112,113]],
       [[200,300, 40, 0], [51,52,53,111,112,113]],
       [[200,300, 50, 0], [51,52,53,111,112,113]],
       [[300,300, 30, 0], [54,55,56,114,115,116]],
       [[300,300, 40, 0], [54,55,56,114,115,116]],
       [[300,300, 50, 0], [54,55,56,114,115,116]],
       [[400,300, 30, 0], [57,58,59,117,118,119]],
       [[400,300, 40, 0], [57,58,59,117,118,119]],
       [[400,300, 50, 0], [57,58,59,117,118,119]],
       [[500,300, 30, 0], [58]],
       [[500,300, 40, 0], [999]],
       [[500,300, 50, 0], [58]],
       [[  0,  0, 30, 1], [3,4,5,18,19,20,21,22,23,63,64,65,78,79,80,81,82,83]],
       [[  0,  0, 40, 1], [3,4,5,18,19,20,21,22,23,63,64,65,78,79,80,81,82,83]],
       [[  0,  0, 50, 1], [3,4,5,18,19,20,21,22,23,63,64,65,78,79,80,81,82,83]],
       [[100,  0, 30, 1], [6,7,8,21,22,23,24,25,26,66,67,68,81,82,83,84,85,86]],
       [[100,  0, 40, 1], [6,7,8,21,22,23,24,25,26,66,67,68,81,82,83,84,85,86]],
       [[100,  0, 50, 1], [6,7,8,21,22,23,24,25,26,66,67,68,81,82,83,84,85,86]],
       [[200,  0, 30, 1], [9,10,11,24,25,26,27,28,29,69,70,71,84,85,86,87,88,89]],
       [[200,  0, 40, 1], [9,10,11,24,25,26,27,28,29,69,70,71,84,85,86,87,88,89]],
       [[200,  0, 50, 1], [9,10,11,24,25,26,27,28,29,69,70,71,84,85,86,87,88,89]],
       [[300,  0, 30, 1], [12,13,14,27,28,29,72,73,74,87,88,89]],
       [[300,  0, 40, 1], [12,13,14,27,28,29,72,73,74,87,88,89]],
       [[300,  0, 50, 1], [12,13,14,27,28,29,72,73,74,87,88,89]],
       [[400,  0, 30, 1], [15,16,17,75,76,77]],
       [[400,  0, 40, 1], [15,16,17,75,76,77]],
       [[400,  0, 50, 1], [15,16,17,75,76,77]],
       [[500,  0, 30, 1], [30,31,32,90,91,92]],
       [[500,  0, 40, 1], [30,31,32,90,91,92]],
       [[500,  0, 50, 1], [30,31,32,90,91,92]],
       [[  0,100, 30, 1], [21,22,23,33,34,35,36,37,38,81,82,83,93,94,95,96,97,98]],
       [[  0,100, 40, 1], [21,22,23,33,34,35,36,37,38,81,82,83,93,94,95,96,97,98]],
       [[  0,100, 50, 1], [21,22,23,33,34,35,36,37,38,81,82,83,93,94,95,96,97,98]],
       [[100,100, 30, 1], [24,25,26,36,37,38,84,85,86,96,97,98]],
       [[100,100, 40, 1], [24,25,26,36,37,38,84,85,86,96,97,98]],
       [[100,100, 50, 1], [24,25,26,36,37,38,84,85,86,96,97,98]],
       [[200,100, 30, 1], [27,28,29,39,40,41,87,88,89,99,100,101]],
       [[200,100, 40, 1], [27,28,29,39,40,41,87,88,89,99,100,101]],
       [[200,100, 50, 1], [27,28,29,39,40,41,87,88,89,99,100,101]],
       [[300,100, 30, 1], [39,40,41,42,43,44,99,100,101,102,103,104]],
       [[300,100, 40, 1], [39,40,41,42,43,44,99,100,101,102,103,104]],
       [[300,100, 50, 1], [39,40,41,42,43,44,99,100,101,102,103,104]],
       [[500,100, 30, 1], [45,46,47,105,106,107]],
       [[500,100, 40, 1], [45,46,47,105,106,107]],
       [[500,100, 50, 1], [45,46,47,105,106,107]],
       [[  0,200, 30, 1], [36,37,38,96,97,98]],
       [[  0,200, 40, 1], [36,37,38,96,97,98]],
       [[  0,200, 50, 1], [36,37,38,96,97,98]],
       [[100,200, 30, 1], [48,49,50,108,109,110]],
       [[100,200, 40, 1], [48,49,50,108,109,110]],
       [[100,200, 50, 1], [48,49,50,108,109,110]],
       [[300,200, 30, 1], [42,43,44,51,52,53,54,55,56,102,103,104,111,112,113,114,115,116]],
       [[300,200, 40, 1], [42,43,44,51,52,53,54,55,56,102,103,104,111,112,113,114,115,116]],
       [[300,200, 50, 1], [42,43,44,51,52,53,54,55,56,102,103,104,111,112,113,114,115,116]],
       [[400,200, 30, 1], [45,46,47,54,55,56,57,58,59,105,106,107,114,115,116,117,118,119]],
       [[400,200, 40, 1], [45,46,47,54,55,56,57,58,59,105,106,107,114,115,116,117,118,119]],
       [[400,200, 50, 1], [45,46,47,54,55,56,57,58,59,105,106,107,114,115,116,117,118,119]],
       [[500,200, 30, 1], [57,58,59,117,118,119]],
       [[500,200, 40, 1], [57,58,59,117,118,119]],
       [[500,200, 50, 1], [57,58,59,117,118,119]],
       [[200,300, 30, 1], [51,52,53,111,112,113]],
       [[200,300, 40, 1], [51,52,53,111,112,113]],
       [[200,300, 50, 1], [51,52,53,111,112,113]],
       [[300,300, 30, 1], [54,55,56,114,115,116]],
       [[300,300, 40, 1], [54,55,56,114,115,116]],
       [[300,300, 50, 1], [54,55,56,114,115,116]],
       [[400,300, 30, 1], [57,58,59,117,118,119]],
       [[400,300, 40, 1], [57,58,59,117,118,119]],
       [[400,300, 50, 1], [57,58,59,117,118,119]],
       [[500,300, 30, 1], [59]],
       [[500,300, 50, 1], [59]],
       [[500,300, 40, 1], [999]]] # Goal

# テスト用のウェイポイントリスト
# wp_list = [[0, 0, 30, 0], [0, 200, 30, 1], [150, 100, 30, 0], [100, 0, 30, 1]]
wp_list = []



# callback関数 -> エージェントの初期座標を取得
def global_position_callback(globalposition):
  # print("\n----------globalPosition_callback----------")
  global init_lat
  global init_long
  global init_alt

  init_lat = globalposition.latitude
  init_long = globalposition.longitude

  if init_lat == 0:
    init_lat = globalposition.latitude
    init_long = globalposition.longitude
    init_alt = globalposition.altitude

# callback関数 -> servo_outputを取得し報酬を計算する
def rc_out_callback(rcout):
  global hov_wat
  global pus_wat
  
  # PWMの初期値を設定
  INIT_HOVER_RC = 900
  INIT_PUSHER_RC = 1000
  
  # プロペラのpwmを取得し, ホバリング用と推進用で学習用に丸める
  hov_wat = hov_wat + ((rcout.channels[0] + rcout.channels[1] + rcout.channels[2] + rcout.channels[3]) - INIT_HOVER_RC * 4) / 4
  pus_wat = pus_wat + (rcout.channels[4] - INIT_PUSHER_RC) * 2.75
  # print('hov_wat -> ',hov_wat)
  # print('pus_wat -> ',pus_wat)
  return hov_wat, pus_wat
  
# callback関数 -> 最新のウェイポイント情報を取得
def current_seq_callback(waypointlist):
  global prev_seq
  global current_seq
  global new_episode_flag
  global hov_wat_total
  global pus_wat_total
  global hov_wat_area
  global pus_wat_area
  global total_wat_pwm
  global push_wp_list
  global end_sw
  global r_list

  current_seq = waypointlist.current_seq
  print('waypointlist.current_seq', waypointlist.current_seq)

  # 弾きたい条件
  # 前回のエピソードの最後のシークエンスが残っているため回避
  print('new_episode_flag is ',new_episode_flag)
  print('current_seq is ', current_seq)

  if (new_episode_flag == True and current_seq == 0) or (new_episode_flag == False and current_seq != 0):

    # if current_seq == len(push_wp_list) - 1 and prev_seq == 0:
    #   print('実はここでパスしてるよ')
    #   pass
    # else:

    # 一度でもwaypointListを呼び出すとエピソードフラグがFalseになる
    new_episode_flag = False

    print('prev_seq', prev_seq)
    print('current_seq', current_seq)
    # print('prev_current_seq', prev_current_seq)
    # print('push_wp_list', push_wp_list)
    # print('push_wp_list[waypointlist]', push_wp_list[waypointlist.current_seq].command)

    # 次のウェイポイントへ進んだことを確認しpwmの累計を計算
    if prev_seq != current_seq:
      print('--------------------next_waypoint(reward)------------------')
      print('waypoint area: ',prev_seq,'-',current_seq)
      
      # 座標ウェイポイントを示す16のときの処理
      if push_wp_list[current_seq].command == 16:

        print("check:",len(hov_wat_total), current_seq)

        if len(hov_wat_total) == 0:
          prev_hov_wat = 0
        else:
          prev_hov_wat = hov_wat_total[-1]

        if len(pus_wat_total) == 0:
          prev_pus_wat = 0
        else:
          prev_pus_wat = pus_wat_total[-1]

        # 1つ前のウェイポイントから現在のウェイポイントまでに累計されたPWMをリストに追加
        hov_wat_total.append(hov_wat)
        pus_wat_total.append(pus_wat)

        # ホバリング, 推進用それぞれ全体の累計になっているリストの要素をウェイポイント区間の累計に変換
        hov_wat_area.append(hov_wat - prev_hov_wat)
        pus_wat_area.append(pus_wat - prev_pus_wat)
        

        # 最新のウェイポイント状態を更新
        prev_seq = current_seq

        print('hov_watt =', hov_wat)
        print('pus_watt =', pus_wat)
        print('hov_wat_area =', hov_wat_area)
        print('pus_wat_area =', pus_wat_area)
        print('hov_wat_total =', hov_wat_total)
        print('pus_wat_total =', pus_wat_total)
        print('total_wat_pwm', total_wat_pwm)

      elif push_wp_list[current_seq].command == 3000:
        print('遷移中')
        prev_seq = current_seq

      # action終了時にtotal_wat_pwmから報酬であるr_listを計算
      if current_seq == len(push_wp_list) - 1 and len(r_list) <= len(wp_list) - 1:
        # ホバリング用のPWM累計と, 推進用のPWM累計をウェイポイント区間に変換したものの合計
        for i in range(len(hov_wat_area)):
          total_wat_pwm.append(float(hov_wat_area[i] + pus_wat_area[i]))
        
        # 罰則値で調整する
        DENOMINATOR = 2000000
        for pwm in total_wat_pwm:
          r_list.append(float(pwm)/DENOMINATOR)

        print('できたてのr_list:', r_list)
  else:
    print('bad callback回避成功')
    pass

    # print('push_wp_list',len(push_wp_list)-1)

# callback関数 -> armの状態を取得し終了判定をする
def state_callback(state):
  global end_sw
  global prev_seq
  
  arm_state = True
  arm_state = state.armed
  
  # TODO: current_seq参照できる？
  # 1エピソードの終了判定
  # print('current_seq', current_seq)
  if prev_seq != 0 and not arm_state:
    print('end_sw is true.')
    end_sw = True
    rospy.sleep(5)
  

# subscriber -> サーボのPWMを取得するためのコールバック関数呼び出し
def rc_out():
  rospy.Subscriber('/mavros/rc/out',mavros_msgs.msg.RCOut, rc_out_callback)

# subscriber -> シークエンスを取得するためにウェイポイントリストのサブスクライバでコールバック関数を呼び出す
def waypoint_list():
  # current_seqの初期値がバグってるので回避
  rospy.Subscriber("/mavros/mission/waypoints", WaypointList, current_seq_callback, queue_size=1)

# subscriber -> armの真偽値をミッションの終了判定に用いる
def state():
  rospy.Subscriber('/mavros/state',mavros_msgs.msg.State, state_callback)

# 報酬に関する関数をまとめて呼び出す
def reward():
  print('3. 報酬の計算とSubscriber呼び出し')
  rc_out()
  waypoint_list()
  state()


# 便利関数
# 現在の機体で保存されているウェイポイントを削除
def wp_clear():
  rospy.wait_for_service("/mavros/mission/clear")
  waipoint_clear = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
  print('ウェイポイントリストのクリアが完了')
  return waipoint_clear.call().success

# アームする
def arm():
  print("\n----------armingCall----------")
  rospy.wait_for_service("/mavros/cmd/arming")
  armService = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
  resp = armService(1)
  rospy.sleep(.5)
  print('armService_bool', resp)

# 環境のリセット
def reset_world():
  rospy.wait_for_service('/gazebo/reset_world')
  reset_wor = rospy.ServiceProxy('/gazebo/reset_world', Empty)
  reset_wor()

# 環境のパブリッシャもリセットできるかも
def reset_simulation():
  rospy.wait_for_service('/gazebo/reset_simulation')
  reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
  reset_sim()

# set_currentの検証
def set_current():
  rospy.wait_for_service('/mavros/mission/set_current')
  set_cur = rospy.ServiceProxy('mavros/mission/set_current', WaypointSetCurrent)
  set_cur(wp_seq=10)
  # print("set_cur bool",set_cur())

# 変数のリセット
def reset_variable():
  global init_lat
  global init_long
  global hov_wat
  global pus_wat
  global current_seq
  global prev_seq
  global hov_wat_total
  global pus_wat_total
  global hov_wat_area
  global pus_wat_area
  global total_wat_pwm
  global r_list
  global push_wp_list
  global end_sw
  global arm_state

  # 変数の再宣言
  init_lat = 0
  init_long = 0
  hov_wat = 0
  pus_wat = 0
  current_seq = 0
  prev_seq = 0
  hov_wat_total = []
  pus_wat_total = []
  hov_wat_area = []
  pus_wat_area = []
  total_wat_pwm = []
  push_wp_list = []
  r_list = []
  end_sw = False
  arm_state = True

# 苦肉のリセット
def pause_simlation():
  rospy.wait_for_service('/gazebo/pause_physics')

  pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
  pause()

def unpause_simlation():
  rospy.wait_for_service('/gazebo/unpause_physics')
  unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
  unpause()


# ウェイポイントを操作する関数
# 引数でウェイポイント
def free_waypoint(x_lat=0, y_long=0, z_alt=10, wp_frame=6, wp_command=16, wp_param1=0):
  global init_lat
  global init_long

  wp = Waypoint()
  wp.frame = wp_frame
  wp.command = wp_command 
  wp.is_current = False
  wp.autocontinue = True
  wp.param1 = wp_param1
  wp.param2 = 0
  wp.param3 = 0
  wp.param4 = 0
  wp.x_lat = init_lat + x_lat*0.00001
  wp.y_long = init_long + y_long*0.00001
  wp.z_alt = z_alt
  return wp

# 特殊な処理をしてウェイポイントリストを生成したりするための関数
def create_push_wp_list(push_wp_list, wp_list):
  # # テスト用ウェイポイントアペンド
  # push_wp_list.append(free_waypoint(10,10,20)) # 移動
  # push_wp_list.append(free_waypoint(20,20,10)) # 移動
  # push_wp_list.append(free_waypoint(wp_frame=2,wp_command=3000,wp_param1=4)) # 遷移
  # push_wp_list.append(free_waypoint(50,50,20)) # 移動
  # push_wp_list.append(free_waypoint(wp_frame=2,wp_command=20)) # 帰還

  # 1つ前のシークエンス時のフライトモード(初期値はhoverを示す0)
  current_fm = 0

  for coord in wp_list:
    # hoverモード時の処理
    print('coord', coord)
    if coord[3] == 0:
      # 前回とモードが違えば遷移飛行とみなし, planeモードへ遷移するwpを追加
      if current_fm != coord[3]:
        push_wp_list.append(free_waypoint(wp_frame=2, wp_command=3000, wp_param1=4))
      # 遷移飛行の有無にかかわらず1coordにつき1つウェイポイントを作成する
      push_wp_list.append(free_waypoint(coord[0], coord[1], coord[2]))
    
    # planeモード時の処理
    if coord[3] == 1:
      # 前回とモードが違えば遷移飛行とみなし, hoverモードへ移行するwpを追加
      if current_fm != coord[3]:
        push_wp_list.append(free_waypoint(wp_frame=2, wp_command=3000, wp_param1=3))
      # 遷移飛行の有無にかかわらず1coordにつき1つウェイポイントを作成する
      push_wp_list.append(free_waypoint(coord[0], coord[1], coord[2]))

    current_fm = coord[3]

  # ミッション終了後にホームポジションへ帰還
  push_wp_list.append(free_waypoint(wp_frame=2, wp_command=20))
  return push_wp_list

# ウェイポイントリストを機体に送信
def push_waypoint(wp_list):
  try:
    print('----------------------------------------------------------------------call push_waypoint------------------------------------------------------------------------------')
    wpPushService = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
    wpPushService(start_index=0, waypoints=wp_list)
    print('completed sending waypoints.')
  except rospy.ServiceException as e:
    print ("Service call failed: %s" % e)

# ウェイポイント生成と送信
def action():
  global wp_list
  global push_wp_list

  print('2. wp_listから有効なウェイポイントを生成し, 機体に送信するスクリプトを実行')
  
  # 機体へ送信するためのウェイポイントリストをご用意
  # push_wp_list = []

  # エピソードごとに使用するウェイポイントは異なるので過去のウェイポイントリストを削除
  wp_clear()

  rospy.sleep(.5)

  print('actionで処理するwp_list', wp_list)

  # ウェイポイントリストを作る
  push_wp_list = create_push_wp_list(push_wp_list, wp_list)

  # ウェイポイントリストを送る
  push_waypoint(push_wp_list)

  # pushのサービスが起動するまで待機(よくわかりません)
  rospy.wait_for_service('/mavros/mission/push')
  # rospy.wait_for_service('/mavros/mission/set_current')
  rospy.wait_for_service('/mavros/set_mode')

  # waypointSetCurrService = rospy.ServiceProxy('/mavros/mission/set_current', WaypointSetCurrent)

  arm()

  rospy.sleep(1)
  setmode = rospy.ServiceProxy('mavros/set_mode', SetMode)
  print('automission call')
  print(setmode(0, 'AUTO.MISSION'))

# ↓↓↓↓↓↓ここから強化学習↓↓↓↓↓↓↓↓
def simple_convert_into_pi_from_theta(wp):
  pi=[]
  for i in range(len(wp)):
    if len(wp[i][1]) == 0:
      pi.append([])
    else:
      pi.append([1/len(wp[i][1])] * len(wp[i][1]) )
  
  return pi


# ε-greedy法を実装
def get_action(s, Q, epsilon, pi_0):
  print('epsilon is...', epsilon)

  # 行動を決める
  if np.random.rand() < epsilon:
    # εの確率でランダムに動く
    next_direction = np.random.choice(wp[s][1], p=pi_0[s])
    print('next_direction epsilon', next_direction)
  else:
    print('--------------------------------------------------Q[s]---------------------------------------------------',Q[s])
    next_direction = wp[s][1][np.nanargmax(Q[s])]
    print('next_direction else', next_direction)

  return wp[s][1].index(next_direction)

def get_s_next(s, a):
  print("s:" + str(s))
  print("a:" + str(a))
  next_direction = wp[s][1][a] #次の移動先
  
  return next_direction

pi_0 = simple_convert_into_pi_from_theta(wp)
# print('pi_0_check', pi_0)
Q = copy.deepcopy(pi_0)
eta = 0.1 #学習率
gamma = 0.9 #時間割引率
epsilon = 0.8 # ε-greedy法の初期値
is_continue = True

# Q学習の実装

def Q_learning(s,a,r,s_next,Q,eta,gamma):
  
  if s_next == 58 or s_next == 119:  #ゴール時
    Q[s][a] = Q[s][a] + eta*(r - Q[s][a])
  else:
    Q[s][a] = Q[s][a] + eta * (r + gamma *np.nanmax(Q[s_next]) - Q[s][a])
  
  return Q

def goal_graph_ret_s_a_Q(Q, epsilon, eta, gamma, pi):
  s = 0
  a = a_next = get_action(s, Q, epsilon, pi) # 初期の行動
  print('a dayo ', a)
  print('a_next dayo', a_next)
  s_a_history = [[0,np.nan]] #エージェントの移動を記録するリスト
  
  while (1):
    a = a_next #行動更新
    
    s_a_history[-1][1] = a # 現在の状態に行動を挿入
    print('while s', s)
    print('while a', a)
    
    s_next = get_s_next(s, a)
    # 次の状態を格納
    
    s_a_history.append([s_next, np.nan])
    # 次の状態を代入．行動はまだ未確定なのでnan
    
    #　報酬を与え，次の行動を求めます
    if s_next == 58 or s_next == 119:
      r = 1 # ゴールにたどり着いたら報酬を与える
      a_next = np.nan
    else:
      r = -0.001
      a_next = get_action(s_next, Q, epsilon, pi)
      # 次の行動a_nextを求めます
    
    # 価値関数を更新
    # Q = Q_learning(s,a,r,s_next,Q,eta,gamma)
    # 終了判定
    if s_next == 58 or s_next == 119:
      break
    else:
      s=s_next
    
  return [s_a_history, Q]



# 強化学習用のメイン文
def main():
  global episode
  global eta
  global gamma
  global epsilon
  global is_continue
  global Q
  global s_list
  global r_list
  global end_sw
  global wp_list
  global new_episode_flag

  # # global変数
  # init_lat = 0
  # init_long = 0
  # hov_wat = 0
  # pus_wat = 0
  # current_seq = 0
  # prev_current_seq = 0
  # hov_wat_total = []
  # pus_wat_total = []
  # hov_wat_area = []
  # pus_wat_area = []
  # total_wat_pwm = []
  # r_list = []
  # end_sw = False

  # ノードを定義　pythonファイルにつき1つあればよい
  rospy.init_node('qlearning')

  # subscriber -> 呼び出し
  rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, global_position_callback)
  rospy.wait_for_service('/mavros/mission/set_current')
  # set_cur = rospy.ServiceProxy('mavros/mission/set_current', WaypointSetCurrent)
  reward()

  while is_continue:
    print ("---------------------------------------------episode : "+ str(episode)+"-----------------------------------------------------")

    # ε-greedyの値を少しずつ小さくする
    epsilon = epsilon / 2
    
    # Q-学習でグラフを解き，移動した履歴と更新したQ値を求める
    [s_a_history, Q] = goal_graph_ret_s_a_Q(Q, epsilon, eta, gamma, pi_0)
    
    print("グラフを解くのにかかったステップ数は" + str(len(s_a_history) - 1) + "です")

    # 100エピソード繰り返す
    episode = episode + 1
    print('s_a_history', s_a_history)
    wp_list = [] #ウェイポイント(座標とフライトモード)のリスト
    s_list = [] # 状態のリスト
    a_list = [] # アクションのリスト
    # next_s_list = [] # 次の状態のリスト

    print('s_a_historyからwp_listをappendされる前のwp_list', wp_list)

    for i in range(len(s_a_history)):
      wp_list.append(wp[s_a_history[i][0]][0])
      s_list.append(s_a_history[i][0])
      a_list.append(s_a_history[i][1])
      if episode > 10:
        break
    
    print('1. 実行するwp_listをwpから生成する')
    print('wpから生成したwp_listはコチラ→ ', wp_list)

    # rospy.sleep(5)

    # wp_listからmavrosで有効なウェイポイントに変換して送信する
    action()

    # 機体の使用したPWMから報酬値を返す関数
    # reward()

    # 終了判定
    print('ここがTrueになるとバッチ更新処理に入るよ/end_sw is ->', end_sw)
    while not end_sw:
      pass

    # Q-Learning用変数のデバッグ
    print('s_list ', s_list)
    print('a_list ', a_list)
    print('r_list ', r_list)
    # print('wp     ', wp)
    # print('Q      ', Q)
    print('eta    ', eta)
    print('gamma  ', gamma)

    """Qテーブルのバッチ更新"""
    print('4. Qテーブルのバッチ更新するよ')
    # goal_graph_ret_s_a_Q()のQ値の更新を消す
    for i in range(len(s_a_history) - 1):
      Q = Q_learning(s_list[i],a_list[i],r_list[i],wp[s_list[i]][1][a_list[i]],Q,eta,gamma)

    print('5. リセットする予定だよ')

    # reset()
    wp_clear()
    reset_variable()
    print('reset_variable done.')
    print('r_listってほんとに再宣言できてるのかチェック', r_list)
    print('end_swってほんとにFalseになってるのかチェック', end_sw)
    print('3秒間待機')

    # エピソードの終了に合わせてエピソードフラグをTrueにする
    new_episode_flag = True

    time.sleep(.3)



    # current_seqのリセット
    # set_current()
    # set_cur(wp_seq=0)

    # print('reset_world()')
    # # reset_world()
    # time.sleep(1)

    # print('pause')
    # pause_simlation()
    # time.sleep(2)ValueError: 'a' cannot be empty unless no samples are taken
    # time.sleep(5)

    # print('unpause')
    # unpause_simlation()
    # print('2秒で世界が動き出す')
    # time.sleep(2)





  # rospy.spin()


if __name__ == '__main__':
  main()
  # rospy.wait_for_service('/mavros/mission/set_current')
  # waypointSetCurrService = rospy.ServiceProxy('/mavros/mission/set_current', WaypointSetCurrent)