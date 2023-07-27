import os, sys
from platform import system


if system() == 'Linux':
  current_path = str(os.path.abspath(os.getcwd())) + '/scripts'
  pico_dir = "/media/" + os.popen('whoami').read().replace('\n', '') + "/RPI-RP2"
  if os.path.exists(pico_dir):
    os.system("cp " + current_path +  "/../.pio/build/pico/firmware.uf2 " + pico_dir)
  else:
    sys.exit()

elif system() == 'Windows':
  current_path = str(os.path.abspath(os.getcwd())) + '\\scripts'

  device_ids = os.popen('wmic logicaldisk get deviceid').read().replace(' ', '').strip().split('\n')
  volume_names = os.popen('wmic logicaldisk get volumename').read().replace(' ', '').strip().split('\n')
  for volume_name, device_id in zip(volume_names, device_ids):
    if volume_name == 'RPI-RP2':
      pico_dir = "\\" + device_id
      break
    else:
      sys.exit()
    
  os.system("copy " + current_path +  "\\..\\.pio\\build\\pico\\firmware.uf2 " + pico_dir)
    
else:
  sys.exit()