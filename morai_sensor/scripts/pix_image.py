#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
from PIL import Image
import io

def main():
    rospy.init_node('image_publisher', anonymous=True)
    publisher = rospy.Publisher('/image_jpeg/compressed', CompressedImage, queue_size=10)

    # 이미지 파일을 열고, 바이트로 변환
    img_path = "./image/image9.jpg"
    img = Image.open(img_path)
    img_byte_arr = io.BytesIO()
    img.save(img_byte_arr, format='JPEG')
    img_byte_arr = img_byte_arr.getvalue()

    # CompressedImage 메시지 생성
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = img_byte_arr

    rate = rospy.Rate(1) # 1 Hz

    while not rospy.is_shutdown():
        publisher.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
