import rospy
import rospkg
from sdpp_navigation.value_iter_weighting import ValueIterationWeightingMulti as VIWM



if __name__ == "__main__":
    rospy.init_node("VIWM_node")

    VIWM_config = {"agent_list": ["human_0", "human_1"]

    }

    rospack = rospkg.RosPack()

    path = rospack.get_path('sdpp_navigation')

    test = VIWM()
