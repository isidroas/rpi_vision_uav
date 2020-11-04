import os
import shutil
import yaml

# Asegurarse de que estamos en el directorio 'build'
curr_path= os.getcwd()
if os.path.basename(curr_path)!='build':
    print(f'Se debe de estar en el directorio build')
    quit(1)

# read parameters
params=None
with open('../vision_params.yml') as file:
    # remove first line since it contains invalid syntax
    file.readline()
    params = yaml.load(file.read(), Loader=yaml.FullLoader)

# crear las carpetas necesarias si no existen
exists_dirs=os.path.exists('../results/latest')
if not exists_dirs:
    os.makedirs('../results/latest')
else:
    shutil.rmtree('../results/latest')

write_images=params["write_images"]
if write_images:
    os.makedirs('../results/latest/images')
else:
    os.makedirs('../results/latest')

log_file=params["log_file"]
if log_file:
    os.system('touch ../results/latest/log.csv')

print(f" el estado de write_images es de {write_images}")

quit(0)
