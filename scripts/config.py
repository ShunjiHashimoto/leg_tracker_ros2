#!/usr/bin/env python3
# -*- coding: utf-8 -*-

class LegTrackerParam:
    # 重みの定義
    id_weight = 0.0
    distance_weight = 0.0 
    speed_weight = 0.0
    static_penalty = 0.0 # 静的オブジェクトに対するペナルティ
    single_leg_penalty = 0.0 # 片足のみしか検出しない場合はペナルティを追加
    obstacle_speed_threshold = 0.25
    vel_history_size = 50
    debug = True
