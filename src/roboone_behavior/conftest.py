# colcon を通さず ws ルートから pytest を直接叩いたときに
# roboone_behavior パッケージを見つけられるようにする（roboone_motion と同じ）。
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
