#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/10/22
import cv2
import math
import yaml
import rospy
import random
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Quaternion

range_rgb = {
    'red': (0, 50, 255),
    'blue': (255, 50, 0),
    'green': (50, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

def loginfo(msg):
    rospy.loginfo('\033[1;32m%s\033[0m' % msg)

def cv2_image2ros(image, frame_id=''):
    image = image[:,:,::-1]
    ros_image = Image()
    header = Header(stamp=rospy.Time.now())
    header.frame_id = frame_id
    ros_image.height = image.shape[:2][0]
    ros_image.width = image.shape[:2][1]
    ros_image.encoding = 'rgb8'
    ros_image.data = image.tostring()
    ros_image.header = header
    ros_image.step = ros_image.width * 3

    return ros_image

def get_yaml_data(yaml_file):
    yaml_file = open(yaml_file, 'r', encoding='utf-8')
    file_data = yaml_file.read()
    yaml_file.close()

    data = yaml.load(file_data, Loader=yaml.FullLoader)

    return data

def save_yaml_data(data, yaml_file):
    f = open(yaml_file, 'w', encoding='utf-8')
    yaml.dump(data, f)

    f.close()

def distance(point_1, point_2):
    """
    计算两个点间的距离(calculate the distance between two points)
    :param point_1: 点1(point 1)
    :param point_2: 点2(point 2)
    :return: 两点间的距离(distance between two points)
    """
    return math.sqrt((point_1[0] - point_2[0]) ** 2 + (point_1[1] - point_2[1]) ** 2)

def box_center(box):
    """
    计算四边形box的中心(calculate the center of quadrangle box)
    :param box: box （x1, y1, x2, y2)形式(box （x1, y1, x2, y2)type)
    :return:  中心坐标（x, y)(center coordinate（x, y))
    """
    return (box[0] + box[2]) / 2, (box[1] + box[3]) / 2

def bgr8_to_jpeg(value, quality=75):
    """
    将cv bgr8格式数据转换为jpg格式(convert data in the format of cv bgr8 into jpg)
    :param value: 原始数据(original data)
    :param quality:  jpg质量 最大值100(jpg quality. Maximum value is 100)
    :return:
    """
    return bytes(cv2.imencode('.jpg', value)[1])

def point_remapped(point, now, new, data_type=float):
    """
    将一个点的坐标从一个图片尺寸映射的新的图片上(map the coordinate of one point from a picture to a new picture of different size)
    :param point: 点的坐标(coordinate of point)
    :param now: 现在图片的尺寸(size of current picture)
    :param new: 新的图片尺寸(new picture size)
    :return: 新的点坐标(new point coordinate)
    """
    x, y = point
    now_w, now_h = now
    new_w, new_h = new
    new_x = x * new_w / now_w
    new_y = y * new_h / now_h
    return data_type(new_x), data_type(new_y)

def get_area_max_contour(contours, threshold=100):
    """
    获取轮廓中面积最重大的一个, 过滤掉面积过小的情况(get the contour whose area is the largest. Filter out those whose area is too small)
    :param contours: 轮廓列表(contour list)
    :param threshold: 面积阈值, 小于这个面积的轮廓会被过滤(area threshold. Contour whose area is less than this value will be filtered out)
    :return: 如果最大的轮廓面积大于阈值则返回最大的轮廓, 否则返回None(if the maximum contour area is greater than this threshold, return the
    largest contour, otherwise return None)
    """
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # 历遍所有轮廓(traverse through all contours)
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积(calculate the area of the contour)
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > threshold:  # 只有在面积大于50时，最大面积的轮廓才是有效的，以过滤干扰(only consider the maximum contour as valid if its area is greater than 50, allowing to filter the interference)
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓(return the maximum contour)

def vector_2d_angle(v1, v2):
    """
    计算两向量间的夹角 -pi ~ pi(calculate the angle between two vectors -pi ~ pi)
    :param v1: 第一个向量(the first vector)
    :param v2: 第二个向量(the second vector)
    :return: 角度(angle)
    """
    d_v1_v2 = np.linalg.norm(v1) * np.linalg.norm(v2)
    cos = v1.dot(v2) / (d_v1_v2)
    sin = np.cross(v1, v2) / (d_v1_v2)
    angle = np.degrees(np.arctan2(sin, cos))
    return angle

def warp_affine(image, points, scale=1.0):
    """
     简单的对齐，计算两个点的连线的角度，以图片中心为原点旋转图片，使线水平(Simple alignment. Calculate the angle of the line connecting the two points.
     Rotate the picture around the image center to make the line horizontal)
    可以用在人脸对齐上(can be used to align the face)

    :param image: 要选择的人脸图片(select face picture)
    :param points: 两个点的坐标 ((x1, y1), (x2, y2))(coordinate of two points ((x1, y1), (x2, y2)))
    :param scale: 缩放比例(scaling)
    :return: 旋转后的图片(rotated picture)
    """
    w, h = image.shape[:2]
    dy = points[1][1] - points[0][1]
    dx = points[1][0] - points[0][0]
    # 计算旋转角度并旋转图片(calculate the rotation angle and rotate picture)
    angle = cv2.fastAtan2(dy, dx)
    rot = cv2.getRotationMatrix2D((int(w / 2), int(h / 2)), angle, scale=scale)
    return cv2.warpAffine(image, rot, dsize=(h, w))

def perspective_transform(img, src, dst, debug=False):
    """
    执行透视变换：将倾斜视角拍摄到的道路图像转换成鸟瞰图，即将摄像机的视角转换到和道路平行。(Perform perspective transform: convert a skewed road image to a bird's-eye view, which means transform the camera perspective to be parallel to the road)
    :param img: 输入图像(the input image)
    :param src: 源图像中待测矩形的四点坐标(four points' coordinates of the rectangle to be measured in the original image)
    :param dst: 目标图像中矩形的四点坐标(four points' coordinates of the rectangle in the target image)
    """
    img_size = (img.shape[1], img.shape[0])
    # 手动提取用于执行透视变换的顶点(Manually extract the vertices for performing perspective transform)
    '''
    # left_down
    # left_up
    # right_up
    # right_down
    src = np.float32(
        [[89, 370],
         [128, 99],
         [436, 99],
         [472, 371]])
    
    dst = np.float32(
        [[65, 430],
         [65, 55],
         [575,55],
         [575,430]])
    '''
    m = cv2.getPerspectiveTransform(src, dst)  # 计算透视变换矩阵(calculate the perspective transform matrix)
    if debug:
        m_inv = cv2.getPerspectiveTransform(dst, src)
    else:
        m_inv = None
    # 进行透视变换 参数：输入图像、输出图像、目标图像大小、cv2.INTER_LINEAR插值方法(Perform perspective transformation with parameters: input image, output image, target image size, and cv2.INTER_LINEAR interpolation method)
    warped = cv2.warpPerspective(img, m, img_size, flags=cv2.INTER_LINEAR)
    #unwarped = cv2.warpPerspective(warped, m_inv, (warped.shape[1], warped.shape[0]), flags=cv2.INTER_LINEAR)  # 调试(debug)

    return warped, m, m_inv

class Colors:
    # Ultralytics color palette https://ultralytics.com/
    def __init__(self):
        # hex = matplotlib.colors.TABLEAU_COLORS.values()
        hex = ('FF3838', 'FF9D97', 'FF701F', 'FFB21D', 'CFD231', '48F90A', '92CC17', '3DDB86', '1A9334', '00D4BB',
               '2C99A8', '00C2FF', '344593', '6473FF', '0018EC', '8438FF', '520085', 'CB38FF', 'FF95C8', 'FF37C7')
        self.palette = [self.hex2rgb('#' + c) for c in hex]
        self.n = len(self.palette)

    def __call__(self, i, bgr=False):
        c = self.palette[int(i) % self.n]
        return (c[2], c[1], c[0]) if bgr else c

    @staticmethod
    def hex2rgb(h):  # rgb order (PIL)
        return tuple(int(h[1 + i:1 + i + 2], 16) for i in (0, 2, 4))

def plot_one_box(x, img, color=None, label=None, line_thickness=None):
    """
    description: Plots one bounding box on image img,
                 this function comes from YoLov5 project.
    param:
        x:      a box likes [x1,y1,x2,y2]
        img:    a opencv image object
        color:  color to draw rectangle, such as (0,255,0)
        label:  str
        line_thickness: int
    return:
        no return

    """
    tl = (
            line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1
    )  # line/font thickness`
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    if label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(
            img,
            label,
            (c1[0], c1[1] - 2),
            0,
            tl / 3,
            [225, 255, 255],
            thickness=tf,
            lineType=cv2.LINE_AA,
        )

def val_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def qua2rpy(qua):
    if type(qua) == Quaternion:
        x, y, z, w = qua.x, qua.y, qua.z, qua.w
    else:
        x, y, z, w = qua[0], qua[1], qua[2], qua[3]
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
  
    return roll, pitch, yaw

def rpy2qua(roll, pitch, yaw):
    cy = math.cos(yaw*0.5)
    sy = math.sin(yaw*0.5)
    cp = math.cos(pitch*0.5)
    sp = math.sin(pitch*0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    q = Pose()
    q.orientation.w = cy * cp * cr + sy * sp * sr
    q.orientation.x = cy * cp * sr - sy * sp * cr
    q.orientation.y = sy * cp * sr + cy * sp * cr
    q.orientation.z = sy * cp * cr - cy * sp * sr
    return q.orientation

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6  

def rot2rpy(R):
    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    
    if not singular:
        r = math.atan2(R[2, 1], R[2, 2])
        p = math.atan2(-R[2, 0], sy)
        y = math.atan2(R[1, 0], R[0, 0])
    else:
        r = math.atan2(-R[1, 2], R[1, 1])
        p = math.atan2(-R[2, 0], sy)
        y = 0
    
    return r, p, y

def rpy2rot(r, p, y):
    cr = math.cos(r)
    sr = math.sin(r)
    cp = math.cos(p)
    sp = math.sin(p)
    cy = math.cos(y)
    sy = math.sin(y)

    R = np.array([[cp*cy, -cr*sy + cy*sp*sr, cr*cy*sp + sr*sy],
        [cp*sy, cr*cy + sp*sr*sy, cr*sp*sy - cy*sr],
        [-sp, cp*sr, cp*cr]])

    return R

def rot2qua(M):
    Qxx, Qyx, Qzx, Qxy, Qyy, Qzy, Qxz, Qyz, Qzz = M.flat
    K = np.array([
        [Qxx - Qyy - Qzz, 0,               0,               0              ],
        [Qyx + Qxy,       Qyy - Qxx - Qzz, 0,               0              ],
        [Qzx + Qxz,       Qzy + Qyz,       Qzz - Qxx - Qyy, 0              ],
        [Qyz - Qzy,       Qzx - Qxz,       Qxy - Qyx,       Qxx + Qyy + Qzz]]
        ) / 3.0
    vals, vecs = np.linalg.eigh(K)
    qua = vecs[[3, 0, 1, 2], np.argmax(vals)]
    if qua[0] < 0:
        qua *= -1

    q = Pose()
    q.orientation.w = qua[0]
    q.orientation.x = qua[1]
    q.orientation.y = qua[2]
    q.orientation.z = qua[3]

    return q.orientation

def qua2rot(qua):
    if type(qua) == Quaternion:
        x, y, z, w = qua.x, qua.y, qua.z, qua.w
    else:
        x, y, z, w = qua[0], qua[1], qua[2], qua[3]
    rot_matrix = np.array(
        [[1.0 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (w * y + x * z)],
        [2 * (x * y + w * z), 1.0 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1.0 - 2 * (x * x + y * y)]])

    return rot_matrix

def putText(img, text):
    line = cv2.LINE_AA
    font = cv2.FONT_HERSHEY_PLAIN
    cv2.putText(img, text, (11, 20), font, 1.0, (32, 32, 32), 4, line)
    cv2.putText(img, text, (10, 20), font, 1.0, (240, 240, 240), 1, line)
    
    return img


