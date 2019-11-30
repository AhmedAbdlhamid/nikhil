#!/usr/bin/env python3
# -*- coding: utf-8 -*-

###############################################################################
#

import const

class Config:
    def __init__(self): # model_name):
        self.version = (0, 4) # version 0.4
        self.valid_rate = 0.3
        self.fname_ext = '.jpg'
        self.num_epochs = 20
        self.batch_size = 16
        self.num_outputs = 1  # steering_angle, throttle
        #self.raw_scale = 1.0 # Multiply raw input by this scale
        #self.jitter_tolerance = 0.009 # joystick jitter
       
        #net_model_type = (0)Nikhil
        #                 (1)Jkwon-Shobhit,
        #                 (2)Nvidia, 
        #                 (3)SqueezeNet (Transfer_Learning),
        #                 (4)MirNet_C (LSTM-fc6),
        #                 (5)MirNet_L (LSTM-fc8),
        #                 (6)Transfer_Learning_ResNet
        self.net_model_type = const.NET_TYPE_NIKHIL #NET_TYPE_MIR

        if self.net_model_type == const.NET_TYPE_NIKHIL:
            self.image_size = (200, 66, 3)
            self.capture_area = (0,402,800,800)

        elif self.net_model_type == const.NET_TYPE_SQUEEZE or self.net_model_type == const.NET_TYPE_RESNET: 
            # Transfer_Learning_ResNet, SqueezeNet
            self.image_size = (400, 200, 3)       
            self.capture_area = (0,370,800,620)
        else: 
            #Jkwon-Shobhit, Nvidia, MirNet_C, MirNet_L
            self.image_size = (160, 70, 3)
            self.capture_area = (0,380,800,800)
        #self.capture_size = (self.capture_area[3]-self.capture_area[1], 
        #                     self.capture_area[2]-self.capture_area[0], 3)
