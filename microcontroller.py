#!/usr/bin/env python3

import time
import evdev
import threading
import os
import pygame
from tflite_runtime.interpreter import Interpreter
from tflite_runtime.interpreter import load_delegate
import cv2
import numpy as np
import picamera
from picamera import mmal, mmalobj as mo
import atexit
from PIL import Image
from picamera.array import PiRGBArray
import io

vector = None
vector_lock = threading.Lock()
cam = None
cam_lock = threading.Lock()
orig_img = None
orig_img_lock = threading.Lock()
img = None
img_lock = threading.Lock()
boxs = []
boxs_lock = threading.Lock()
right_pressed = False
right_pressed_lock = threading.Lock()
left_pressed = False
left_pressed_lock = threading.Lock()



class ComputerVision:
    
    def __init__(self):
        self.i = Interpreter(model_path='efficientdet-lite-fortnite_edgetpu.tflite', experimental_delegates=[load_delegate('libedgetpu.so.1')])
        self.i.allocate_tensors()
        self.output = self.i.get_output_details()
        self.input = self.i.get_input_details()[0]
        
        self.thread = threading.Thread(target=self._run)
        
        
    def get_boxes(self, img_tensor):
        global boxs
        img_tensor = img_tensor = np.expand_dims(img_tensor, axis=0)
        self.i.set_tensor(self.input['index'], img_tensor)
        self.i.invoke()
        # classes = self.i.get_tensor(self.output[1]['index'])[0]
        scores = self.i.get_tensor(self.output[0]['index'])[0]
        locations = self.i.get_tensor(self.output[1]['index'])[0]
        boxes = []
        filt_boxes = []
        for(index, score) in enumerate(scores):
            if score>0.4:
                boxes.append(locations[index])
                if locations[index][2] < 0.8:
                    filt_boxes.append(locations[index])
        with boxs_lock:
            boxs = boxes
        return filt_boxes
        
    
    def get_closest_center(self,boxes):
        centers = np.array(list([(box[1]+box[3])/2, ((1-box[0])+(1-box[2]))/2] for box in boxes))
        image_center = np.array([0.5, 0.5])
        distances = np.linalg.norm(centers - image_center, axis=1)
        closest_box_idx = np.argmin(distances)
        closest_center = centers[closest_box_idx]
        return closest_center, closest_box_idx
    
    def get_vector(self, center):
        v = center - np.array([0.5,0.5])
        v = 320*v
        return v
    
    def start(self):
        self.thread.start()
    
    def _run(self):
        global img
        global vector
        while True:
            start = time.time()
            with img_lock:
                frame = img
            if frame is None:
                time.sleep(1)
                continue
            boxes = self.get_boxes(frame)
            if len(boxes) == 0:
                with vector_lock:
                    vector = None
            else:
                closest_center, closest_box_idx = self.get_closest_center(boxes)
                closest_vector = self.get_vector(closest_center)
                # print(closest_vector)
                with vector_lock:
                    vector = closest_vector
            print(str(int(1/(time.time() - start))) + " FPS")


class MouseInputBlocker:
    
    def __init__(self):
        pygame.init()
        self.screen_size = pygame.display.Info().current_w, pygame.display.Info().current_h
        self.screen = pygame.display.set_mode(self.screen_size)
        pygame.event.set_blocked(pygame.MOUSEBUTTONDOWN)
        pygame.event.set_blocked(pygame.MOUSEBUTTONUP)
        pygame.mouse.set_visible(False)
        pygame.event.set_blocked(pygame.MOUSEMOTION)
        self._maximize_window()
        self.thread = threading.Thread(target=self._run)
        cv = ComputerVision()
        cv.start()

    def start(self):
        self.thread.start()

    def stop(self):
        pygame.quit()

    def _run(self):
        global img
        global boxes
        clock = pygame.time.Clock()
        
        while True:
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    self.stop()
                    return
                
            with img_lock:
                current_image = img
            with boxs_lock:
                current_boxes = boxs.copy()
                
            if current_image is not None:
                current_image = cv2.rotate(current_image, cv2.ROTATE_90_CLOCKWISE)
                current_image = cv2.flip(current_image,1)
                pygame_surface = pygame.surfarray.make_surface(current_image)
                for box in current_boxes:
                    top,left,bottom,right = box
                    top = int(320*top)
                    left = int(320*left)
                    width = int((320*right) - left)
                    height = int(top - int(320*bottom))
                    pygame.draw.rect(pygame_surface, ((0,255,0) if bottom < 0.9 else (220,220,220)), (left,int(320*bottom),width,height), 2)
            
                self.screen.blit(pygame_surface, ((1920-320)//2,(1080-320)//2))  
                pygame.display.flip()
            clock.tick(30)

    def _maximize_window(self):
        taskbar_size = self.screen_size[1] - pygame.display.get_surface().get_height()
        pygame.display.set_mode((self.screen_size[0], self.screen_size[1] - taskbar_size), pygame.NOFRAME)
        pygame.display.set_mode((self.screen_size[0], self.screen_size[1]), pygame.NOFRAME)


class MouseEmulator:
    
    def __init__(self):
        self.thread = threading.Thread(target=self._run)
    
    def write_report(self, report):
        with open('/dev/hidg0', 'wb+') as fd:
            fd.write(report)
            
    def get_device(self, name):
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            if name in device.name:
                mouse_device = device.path
        return evdev.InputDevice(mouse_device)
    
    
    def adjust_pixels(self, pixels, horiz, left, right):
        return int(max(min(pixels,127),-128))
    

    def move_mouse(self, pixels, horiz, left, right):
        pixels = self.adjust_pixels(pixels, horiz, left, right)
        click = b'\x02' if right else (b'\x01' if left else b'\x00')
        if horiz:
            self.write_report(click + pixels.to_bytes(1, 'little', signed=True) + b'\x00')
        else:
            self.write_report(click + b'\x00' + pixels.to_bytes(1, 'little', signed=True))

    def print_mouse_events(self):
        mouse = self.get_device('Mouse')
        for event in mouse.read_loop():
            print(event)

    def _run(self):
        
        global right_pressed
        global left_pressed

        mib = MouseInputBlocker()
        mib.start()
        
        mouse = self.get_device('Mouse')
        left = False
        right = False
        
        for event in mouse.read_loop():
            with right_pressed_lock:
                right_pressed = right
            with left_pressed_lock:
                left_pressed = left
            if event.type == 1:
                if event.code == 272:
                    left = True if event.value == 1 else False
                    self.move_mouse(0, True, left, False)
                else:
                    right = True if event.value == 1 else False
                    self.move_mouse(0, True, False, right)
            elif event.type == 2:
                if event.code == 0:
                    self.move_mouse(event.value, True, left, right)
                else:
                    self.move_mouse(event.value, False, left, right)
                    
    def start(self):
        self.thread.start()
        

class MouseInjector(MouseEmulator):
    
    def __init__(self):
        self.thread = threading.Thread(target=self._run)
    
    def _run(self):
        global left_pressed
        global right_pressed
        global vector
        SENSITIVITY = 0.05
        while True:
            with left_pressed_lock:
                left_pressed_curr = left_pressed
            with right_pressed_lock:
                right_pressed_curr = right_pressed
            with vector_lock:
                vector_curr = vector
            if (left_pressed_curr or right_pressed_curr) and vector_curr is not None:
                self.move_mouse(vector_curr[0]*SENSITIVITY, True, left_pressed_curr,right_pressed_curr)
                self.move_mouse(-1*vector_curr[1]*SENSITIVITY, False, left_pressed_curr, right_pressed_curr)
            time.sleep(0.01)


def exit_handler():
    global cam
    with cam_lock:
        cam.close()
        
def run_camera():
    
    def image_callback(port,buf):
        global img
        img_data = np.frombuffer(buf.data,dtype=np.uint8)
        img_data = img_data.reshape((352,608,3))[16:336,144:464]
        with img_lock:
            img = img_data
    
    camera = mo.MMALCamera()
    preview = mo.MMALRenderer()
    camera.outputs[0].framesize = (608, 342)
    camera.outputs[0].framerate = 60
    camera.outputs[0].format = mmal.MMAL_ENCODING_RGB24
    camera.outputs[0].commit()
    camera.outputs[0].enable(image_callback)
    while True:                 
        time.sleep(10)
                    
def main():
    
    cam_thread = threading.Thread(target=run_camera)
    cam_thread.start()
    
    m = MouseEmulator()
    m.start()
    
    mi = MouseInjector()
    mi.start()
    
    
    
if __name__ == "__main__":
    main()
