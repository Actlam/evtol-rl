#!/usr/bin/env python

import rospy
import sensor_msgs
import yaml
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
from std_msgs.msg import Int64
from std_srvs.srv import Empty
from sensor_msgs.msg import NavSatFix

# global変数
init_lat = 0
init_long = 0
hov_wat = 0
pus_wat = 0
current_seq = 0
prev_current_seq = 0
hov_wat_total = []
pus_wat_total = []
hov_wat_area = []
pus_wat_area = []
total_wat_pwm = []
s_list = []
push_wp_list = []

# ウェイポイント
wp = [[[  0,  0, 30, 0], [1,2,3]], # start  　
      [[  0,200, 30, 0], [2,4]],
      [[150,100, 30, 0], [1,3,4]],
      [[300,  0, 30, 1], [2, 4]],
      [[300,200, 30, 1], []]] # goal

# テスト用のウェイポイントリスト
wp_list = [[0, 0, 30, 0], [0, 200, 30, 1], [150, 100, 30, 0], [100, 0, 30, 1]]



# callback関数 -> エージェントの初期座標を取得
def global_position_callback(data):
  # print("\n----------globalPosition_callback----------")
  global init_lat
  global init_long
  global init_alt

  init_lat = data.latitude
  init_long = data.longitude

  if init_lat == 0:
    init_lat = data.latitude
    init_long = data.longitude
    init_alt = data.altitude

# callback関数 -> servo_outputを取得し報酬を計算する
def rc_out_callback(data):
  global hov_wat
  global pus_wat
  
  # PWMの初期値を設定
  INIT_HOVER_RC = 900
  INIT_PUSHER_RC = 1000
  
  # プロペラのpwmを取得し, ホバリング用と推進用で学習用に丸める
  hov_wat = hov_wat + ((data.channels[0] + data.channels[1] + data.channels[2] + data.channels[3]) - INIT_HOVER_RC * 4) / 4
  pus_wat = pus_wat + (data.channels[4] - INIT_PUSHER_RC) * 2.75
  # print('hov_wat -> ',hov_wat)
  # print('pus_wat -> ',pus_wat)
  return hov_wat, pus_wat
  
# callback関数 -> 最新のウェイポイント情報を取得
def current_seq_callback(data):
  global current_seq
  global hov_wat_total
  global pus_wat_total
  global hov_wat_area
  global pus_wat_area
  global total_wat_pwm
  global push_wp_list
  global prev_current_seq

  print('current_seq', current_seq)
  print('data.current_seq', data.current_seq)
  print('prev_current_seq', prev_current_seq)
  print('push_wp_list[data]', push_wp_list[data.current_seq].command)


  # 次のウェイポイントへ進んだことを確認しpwmの累計を計算
  if current_seq != data.current_seq:
    print('--------------------next_waypoint(reward)------------------')
    print('waypoint area: ',current_seq,'-',data.current_seq)
    
    # 座標ウェイポイントを示す16のときの処理
    if push_wp_list[data.current_seq].command == 16:
      print('hogehoge')

      print("check:",len(hov_wat_total), data.current_seq)

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



      # ホバリング用のPWM累計と, 推進用のPWM累計をウェイポイント区間に変換したものの合計
      total_wat_pwm.append(hov_wat_area + pus_wat_area)
      

      # 最新のウェイポイント状態を更新
      current_seq = data.current_seq


      print('hov_watt =', hov_wat)
      print('pus_watt =', pus_wat)
      print('hov_wat_area =', hov_wat_area)
      print('pus_wat_area =', pus_wat_area)
      print('hov_wat_total =', hov_wat_total)
      print('pus_wat_total =', pus_wat_total)
      print('total_wat_pwm', total_wat_pwm)

    elif push_wp_list[data.current_seq].command == 3000:
      print('ただいま遷移中')
      current_seq = data.current_seq




# subscriber -> サーボのPWMを取得するためのコールバック関数呼び出し
def rc_out():
  rospy.Subscriber('/mavros/rc/out',mavros_msgs.msg.RCOut, rc_out_callback)

# subscriber -> シークエンスを取得するためにウェイポイントリストのサブスクライバでコールバック関数を呼び出す
def waypoint_list():
  # current_seqの初期値がバグってるので回避
  rospy.sleep(2)
  rospy.Subscriber("/mavros/mission/waypoints", WaypointList, current_seq_callback)
  rospy.sleep(8)

# 報酬に関する関数をまとめて呼び出す
def reward():
  rc_out()
  waypoint_list()

# 便利関数
def wp_clear():
  rospy.wait_for_service("/mavros/mission/clear")
  waipoint_clear = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
  print('wp clear done')
  return waipoint_clear.call().success

def arm():
  print("\n----------armingCall----------")
  rospy.wait_for_service("/mavros/cmd/arming")
  armService = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
  resp = armService(1)
  rospy.sleep(.5)

def calc_reward(total_wat_pwm):
  global s_list
  DENOMINATOR = 2000000 # 罰則値の調整を行う
  
  for i in total_wat_pwm:
    s_list.append((round(i[0] + i[1])/DENOMINATOR,2))    
    


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

  rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, global_position_callback)
  
  # 機体へ送信するためのウェイポイントリストをご用意
  push_wp_list = []

  # エピソードごとに使用するウェイポイントは異なるので過去のウェイポイントリストを削除
  wp_clear()

  rospy.sleep(.5)

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
  print(setmode(0, 'AUTO.MISSION'))
  rospy.spin()

# 強化学習用
def main():
  global episode
  global eta
  global gamma
  global epsilon
  global is_continue
  global Q
  global s_list
  global end_sw

  # ノードを定義　pythonファイルにつき1つあればよい
  rospy.init_node('qlearning')

  reward()
  action()
  calc_reward(total_wat_pwm)
  print('s_list :', s_list)

  rospy.spin()

if __name__ == '__main__':
  main()
  # rospy.wait_for_service('/mavros/mission/set_current')
  # waypointSetCurrService = rospy.ServiceProxy('/mavros/mission/set_current', WaypointSetCurrent)
