#!pip install roboflow

from roboflow import Roboflow
rf = Roboflow(api_key="6ihZt6lj0HEjI3AqIjXO")
project = rf.workspace("tanimsk").project("wro-detection-47xs2-rrvwj")
version = project.version(1)
dataset = version.download("yolov8")
                
