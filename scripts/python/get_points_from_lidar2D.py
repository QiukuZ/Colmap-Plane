# coding: utf-8
import math
import cv2
import copy
from io_tools import *
"""
cv2鼠标回调函数
"""
def drawPoint(event,x,y,flags,param):
    global im_last
    global points
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append([x, y])
        im_last = copy.deepcopy(im)
        if len(points)>1 and len(points)%2==0:
            dx = points[-1][0]-points[-2][0]
            dy = points[-1][1]-points[-2][1]
            l = math.sqrt(dx*dx+dy*dy)*resolution_in_m*image_resize_scale
            print("Length in World = ",l,' point1 =',points[-2],' point2 =',points[-1])
"""
绘制函数
"""
def drawPoints(im,p):
    im_new = copy.deepcopy(im)
    for i in range(len(p)):
        cv2.circle(im_new, (p[i][0], p[i][1]), 1, (0, 0, 255), -1)
        if i%2 == 1:
            cv2.line(im_new, (p[i-1][0], p[i-1][1]), (p[i][0], p[i][1]), (0, 255, 0),1)
    return im_new



if __name__ == "__main__":


    path = '/home/alex/Desktop/Lidar_Visual_Plane/0603_long_try/data/raw_data/2020-06-01-19-49-55_cut.bag_xray_xy_all.png'
    output_path = '/home/alex/Desktop/Lidar_Visual_Plane/0603_long_try/data'
    output_path = output_path+'/lidar_plane'
    mkdir(output_path)
    points = []
    lines = []


    resolution_in_m = 0.01
    image_resize_scale = 1.0 / 0.5

    im_raw = cv2.imread(path)
    im = cv2.resize(im_raw,(int(im_raw.shape[1]/image_resize_scale),int(im_raw.shape[0]/image_resize_scale)))


    cv2.namedWindow('image1')
    cv2.setMouseCallback('image1', drawPoint)
    while (1):
        cv2.imshow('image1', drawPoints(im,points))
        key = cv2.waitKey(10)
        if key == 27:
            'Esc'
            print(points)
            break
        elif key == 115:
            's: Save and Quit'
            print(points)
            with open(output_path+'/png_points.txt','w') as f:
                for i in range(int(len(points)/2)):
                    points_str =str(int(points[2*i][0]*image_resize_scale)) + ' ' + \
                                str(int(points[2*i][1]*image_resize_scale)) + ' ' + \
                                str(int(points[2*i+1][0]*image_resize_scale)) + ' ' + \
                                str(int(points[2*i+1][1]*image_resize_scale))
                    f.write(points_str + '\n')
            break
        elif key == 114:
            'r: Clear all'
            print('Clear All!')
            points = []
        elif key == 32:
            '空格：撤回一步操作'
            if len(points) > 0 :
                print("Pop One Point ! ")
                points.pop(-1)
            else :
                print("No points!")

    cv2.destroyAllWindows()