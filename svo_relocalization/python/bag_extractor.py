
import rosbag
import numpy as np
import cv2

extract_directory = '/tmp/'
name_patter = 'image{0:4d}.png'
bag = rosbag.Bag('/home/fox/dense_input_data.bag')
pose = []
for topic, msg, t in bag.read_messages(topics=['/nanoslam/dense_input']):
  im = np.fromstring(msg.image.data, np.uint8)
  im = im.reshape((msg.image.height, msg.image.width))
  cv2.imwrite(extract_directory+name_patter.format(msg.frame_id),  im)
  pose.append([
    msg.frame_id,
    msg.pose.position.x,
    msg.pose.position.y,
    msg.pose.position.z,
    msg.pose.orientation.x,
    msg.pose.orientation.y,
    msg.pose.orientation.z,
    msg.pose.orientation.w])
  
# save poses in a file [id x y z rx ry rz]
np.savetxt('/tmp/pose.txt', pose, ['%d', '%f', '%f', '%f', '%f', '%f', '%f', '%f'])
bag.close()
