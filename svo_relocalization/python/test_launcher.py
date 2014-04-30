#!/usr/bin/python

import os
import sys
import yaml
import subprocess

if __name__ == '__main__':

  param_file = sys.argv[1]
  print param_file
  params = yaml.load(open(param_file, 'r'))

  cali_file = params['cali_file']
  cali = yaml.load(open(cali_file, 'r'))

  command = ['rosrun svo_relocalization test_relocalizer']
  command.append(params['train_path'])
  command.append(params['test_path'])
  command.append(params['results_path'])
  command.append(params['place_finder'])
  command.append(params['relpos_finder'])

  command.append(str(cali['cam_width']))
  command.append(str(cali['cam_height']))
  command.append(str(cali['cam_fx']))
  command.append(str(cali['cam_fy']))
  command.append(str(cali['cam_cx']))
  command.append(str(cali['cam_cy']))
  command.append(str(cali['cam_d0']))

  print 'running:', ' '.join(command)
  #subprocess.call(command)
  os.system(' '.join(command))
