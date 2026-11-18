# tools 안의 스크립트가 위 폴더의 config, camera 등을 import할 수 있게 함
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
