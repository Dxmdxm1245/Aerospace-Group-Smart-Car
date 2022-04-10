import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from skimage.draw import line
import imutils
import time


def do_canny(frame):
    # 边缘检测
    gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)#从RGB颜色空间转换到灰度空间
    # 使用11*11的高斯模糊 具体参数自己调
    blur = cv.GaussianBlur(gray, (11, 11), 0)
    # 边缘检测最小值50 最大值150
    canny = cv.Canny(blur, 50,150)#第一个参数是需要处理的原图像，该图像必须为单通道的灰度图；第二个参数是阈值1；第三个参数是阈值2。
    return canny
 
 
def do_segment(frame):
    # 区域遮罩
    polygons = np.array([
        [(100, 800), (1800, 800), (1800, 1200), (100, 1200)]
    ])#创建数组构建多边形选定感兴趣区
 
    mask = np.zeros_like(frame)#构建一个与frame同维度的数组，并初始化所有变量为零。
    cv.fillPoly(mask, polygons, 255)#绘制多边形并对其填充
    segment = cv.bitwise_and(frame, mask)#RGB图像选取掩膜（mask）选定的区域
    return segment

def do_color(img):#对车道线进行轮廓识别
    img = np.where(img < 100, 0, 255).astype(np.uint8)#满足条件(img < 100)，输出0，不满足输出255。
    conts, _ = cv.findContours(img, mode=cv.RETR_EXTERNAL, method=cv.CHAIN_APPROX_SIMPLE)#查找检测物体的轮廓。
    img = cv.cvtColor(img, cv.COLOR_GRAY2BGR)#从灰度空间转换到BGR颜色空间
    img = img.copy()#复制图像
    cv.drawContours(img, conts, -1, (0, 255, 0), 1)#轮廓绘制，第一个参数是指明在哪幅图像上绘制轮廓，第二个参数是轮廓本身，第三个参数指定绘制轮廓list中的哪条轮廓，如果是-1，则绘制其中的所有轮廓
    return img

def do_center(image):#处理中心点
    gray = image
    blurred = cv.GaussianBlur(gray, (5, 5), 0)#高斯滤波减噪
    thresh = cv.threshold(blurred, 60, 255, cv.THRESH_BINARY)[1]#阈值处理
    cnts = cv.findContours(thresh.copy(), cv.RETR_EXTERNAL,#查找检测物体的轮廓。
	    cv.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)#返回cnts中的countors(轮廓)
    for c in cnts:
        M = cv.moments(c)#图像矩， M['m00'] 表示轮廓面积
        if M["m00"]==0:
            continue
        cX = int(M["m10"]/ M["m00"])
        cY = int(M["m01"]/ M["m00"])
        cv.drawContours(image, [c], -1, (0, 255, 0), 2)
        cv.circle(image, (cX, cY), 7, (255, 255, 255), -1)#根据给定的圆心和半径等画圆
        cv.putText(image, "center", (cX - 20, cY - 20),cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)#文字绘制
        posex=cX
        posey=cY
        print("中点坐标为(x=",posex,",y=",posey,")")#输出中心点坐标
    return image 


def image_resize(image):#将图片改为（，）大小
    image=cv.resize(image,(640,480))#更改图片大小为640*480
    return image


 
cap = cv.VideoCapture("C:\\Users\\Desktop\\3.mp4")#把素材视频放到对应地址
start_time = time.time()#计算代码运行时间
counter = 0 
fps = cap.get(cv.CAP_PROP_FPS) #视频平均帧率


while cap.isOpened():
    ret, frame = cap.read()#视频读取
    key = cv.waitKey(1) & 0xff
    if key == ord(" "):#空格暂停
        cv.waitKey(0)
    if key == ord("q"):#q键结束
        break
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
    if not ret:
        print('视频读完！')#若视频结束显示视频读完！
        break
    counter += 1#计算帧数
    if (time.time() - start_time) != 0:#实时显示帧数
        cv.putText(frame, "FPS {0}".format(float('%.1f' % (counter / (time.time() - start_time)))), (500, 50),
                    cv.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255),
                    3)#文字绘制
        cv.imshow("initial",image_resize(frame))#展示图像
        canny = do_canny(frame)   
        cv.imshow("canny", image_resize(canny))         
        segment_canny = do_segment(canny)
        segment = do_segment(frame)
        cv.imshow("segment+canny", image_resize(segment_canny))
        cv.imshow("segment",image_resize(segment))
        color=do_color(segment_canny)
        cv.imshow("color",image_resize(color))
        center=do_center(segment_canny)
        cv.imshow("center", image_resize(center))
        counter = 0
        start_time = time.time()
cap.release()#需要放置在while循环体外。发布软件资源，释放硬件资源
cv.destroyAllWindows()#用来删除窗口，（）里不指定任何参数，则删除所有窗口，删除特定的窗口，往（）输入特定的窗口值。






