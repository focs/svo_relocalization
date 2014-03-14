
import sys
import os
import rosbag
import numpy as np
import cv2

def ensure_dir(f):
  d = os.path.dirname(f)
  if not os.path.exists(d):
    os.makedirs(d)

def main():
  if len(sys.argv) < 3:
    print 'Usage: %s <file.bag> <save_folder>' % sys.argv[0]
    sys.exit()

  bag_file = sys.argv[1]
  bag_file_name = bag_file.split('/')[-1]
  print bag_file_name
  extract_directory = sys.argv[2]+bag_file_name[:-4]+'/'
  name_patter = 'image{0:4d}.png'
  bag = rosbag.Bag(bag_file)

  print bag_file
  print extract_directory


  ensure_dir(extract_directory)

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
  np.savetxt(extract_directory+'pose.txt', pose, ['%d', '%f', '%f', '%f', '%f', '%f', '%f', '%f'])
  bag.close()

if __name__ == '__main__':
  main()
