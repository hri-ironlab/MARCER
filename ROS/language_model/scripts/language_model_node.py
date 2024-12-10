#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from language_model.language_model import LanguageModel
from language_model.srv import QueryLanguageModel, QueryLanguageModelRequest, QueryLanguageModelResponse

class LanguageModelNode:
    def __init__(self):
        rospy.init_node("language_model_node")

        self.language_model = LanguageModel(openai_key="ADD_KEY_HERE",
                        model="gpt-4o", translation_model="sentence-transformers/all-roberta-large-v1", n=5, temperature=1)
        
        self.service = rospy.Service('language_model/query_language_model', QueryLanguageModel, self.language_model.handle_language_model_query)
        
if __name__ == "__main__":

    language_model_node = LanguageModelNode()
    rospy.sleep(3)

    rate = rospy.Rate(20)  # 10hz
    while not rospy.is_shutdown():
        rate.sleep()