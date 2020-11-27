#!/usr/bin/env python
# coding: utf-8

import rospy
import mavros
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

# テスト用の生のウェイポイント
wp_raw = [[[  0,  0, 30, 0], [1,2,3]], # start  　
      [[  0,200, 30, 0], [2,4]],
      [[150,100, 30, 0], [1,3,4]],
      [[300,  0, 30, 1], [2, 4]],
      [[300,200, 30, 1], []]] # goal

# テスト用のウェイポイントリスト
wp_list = [[0, 0, 30, 0], [0, 200, 30, 0], [150, 100, 30, 0], [0, 200, 30, 0], [150, 100, 30, 0], [0, 200, 30, 0], [150, 100, 30, 0], [0, 200, 30, 0], [300, 200, 30, 1]]


# -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# qlearning

def simple_convert_into_pi_from_theta(wp):
  pi=[]
  for i in range(len(wp)):
    if len(wp[i][1]) == 0:
      pi.append([])
    else:
      pi.append([1/len(wp[i][1])] * len(wp[i][1]) )
  
  return pi

pi_0 = simple_convert_into_pi_from_theta(wp)


# ε-greedy法を実装
def get_action(s, Q, epsilon, pi_0):

  # 行動を決める
  if np.random.rand() < epsilon:
    # εの確率でランダムに動く
    next_direction = np.random.choice(wp[s][1], p=pi_0[s])
  else:
    next_direction = wp[s][1][np.nanargmax(Q[s])]

  return wp[s][1].index(next_direction)

def get_s_next(s, a):
  print("s:" + str(s))
  print("a:" + str(a))
  next_direction = wp[s][1][a] #次の移動先
  
  return next_direction

Q = copy.deepcopy(pi_0)
eta = 0.1 #学習率
gamma = 0.9 #時間割引率
epsilon = 0.5 # ε-greedy法の初期値
is_continue = True
episode = 1

# Q学習の実装

def Q_learning(s,a,r,s_next,Q,eta,gamma):
  
  if s_next == 13:  #ゴール時
    Q[s][a] = Q[s][a] + eta*(r - Q[s][a])
  else:
    Q[s][a] = Q[s][a] + eta * (r + gamma *np.nanmax(Q[s_next]) - Q[s][a])
  
  return Q

def goal_graph_ret_s_a_Q(Q, epsilon, eta, gamma, pi):
  s = 0
  a = a_next = get_action(s, Q, epsilon, pi) # 初期の行動
  s_a_history = [[0,np.nan]] #エージェントの移動を記録するリスト
  
  while (1):
    a = a_next #行動更新
    
    s_a_history[-1][1] = a # 現在の状態に行動を挿入
    
    s_next = get_s_next(s, a)
    # 次の状態を格納
    
    s_a_history.append([s_next, np.nan])
    # 次の状態を代入．行動はまだ未確定なのでnan
    
    #　報酬を与え，次の行動を求めます
    if s_next == 4:
      r = 1 # ゴールにたどり着いたら報酬を与える
      a_next = np.nan
    else:
      r = -0.001
      a_next = get_action(s_next, Q, epsilon, pi)
      # 次の行動a_nextを求めます
    
    # 価値関数を更新
    # Q = Q_learning(s,a,r,s_next,Q,eta,gamma)
    # 終了判定
    if s_next == 4:
      break
    else:
      s=s_next
    
  return [s_a_history, Q]

# main関数
def main():
  global episode
  global eta
  global gamma
  global epsilon
  global is_continue
  global Q
  global s_list
  global end_sw

  rospy.init_node('qlearning')

  while is_continue:
    print("エピソード："+ str(episode))
    
    # ε-greedyの値を少しずつ小さくする
    epsilon = epsilon / 2
    
    # Q-学習でグラフを解き，移動した履歴と更新したQ値を求める
    [s_a_history, Q] = goal_graph_ret_s_a_Q(Q, epsilon, eta, gamma, pi_0)
    
    print("グラフを解くのにかかったステップ数は" + str(len(s_a_history) - 1) + "です")
    
    # 100エピソード繰り返す
    episode = episode + 1
    print(s_a_history)
    wp_list = [] #ウェイポイント(座標とフライトモード)のリスト
    s_list = [] # 状態のリスト
    a_list = [] # アクションのリスト
    next_s_list = [] # 次の状態のリスト

    for i in range(len(s_a_history)):
      wp_list.append(wp[s_a_history[i][0]][0])
      s_list.append(s_a_history[i][0])
      a_list.append(s_a_history[i][1])
      if episode > 3:
        break
    
    print(wp_list)
    # デバッグ用
    # print('-----------main debug-----------------')
    # print('s_list_',s_list)
    # print('a_list', a_list)
    # print('next_list', next_s_list)
    # print('wp_list', wp_list)``
    
    wp_clear()
    reward()

    # print('reward!!!!!!!!!!!!', total_wat_r)
    # print(len(wp_list))
    # actionを実行
    # action(wpoint_list)
    # wl = action(wp_list)
    # print('waypoint list', wl)
    # print('after action')
    # print('end_sw', end_sw)

    while not end_sw:
      pass
    
    calc_reward(total_wat_pwm, wl)
    # print('calc reward!!!')
    # print('reward : ', total_wat_r)

    # 環境のリセット
    reset_env()
    # print('reset env!!!')

    """報酬を受け取る部分"""
    r_list = total_wat_r
    # print('wpoint_list', wpoint_list)
    # print('len(wpoint_list)', len(wpoint_list))
    # print('total_wat_r', total_wat_r)
    # print('len(total_wat_r)', len(total_wat_r))

    """Qテーブルのバッチ更新"""
    # goal_graph_ret_s_a_Q()のQ値の更新を消す
    for i in range(len(s_a_history) - 1):
      Q = Q_learning(s_list[i],a_list[i],r_list[i],wp[s_list[i]][1][a_list[i]],Q,eta,gamma)

  rospy.spin()

if __name__ == "__main__":
    main()