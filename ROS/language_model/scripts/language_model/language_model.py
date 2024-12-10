#!/usr/bin/env python3
import numpy as np
import torch
import rospy
import time
import openai

from ipdb import set_trace
from language_model.srv import QueryLanguageModelResponse

try:
    import language_model.config as config
except ImportError:
    import config


class LanguageModel:
    def __init__(self, **kwargs):
        if torch.cuda.is_available():
            torch.cuda.set_device(0)
            print("Cuda is enabled")

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.OPENAI_KEY = kwargs["openai_key"]
        self.hyperparams = config.Hyperparameters()

        self.language_model = kwargs["model"]
        self.translation_model = kwargs["translation_model"]

        self.language_model_client = openai.OpenAI(api_key=kwargs["openai_key"])
        
        #self.run_test_prompt()

    def run_test_prompt(self):
        response = self.query_language_model(config.test_prompt)
        print(response)

    def handle_language_model_query(self, req):
        formatted_prompt = self.format_prompt(req.prompt)
        # Generate response using the language model

        response = self.query_language_model(formatted_prompt)

        # Return the response back to the client
        return QueryLanguageModelResponse(response)

    def query_language_model(self, formatted_prompt):
        samples, log_probability, _ = self.generate_response(
            formatted_prompt, self.hyperparams.sampling_params
        )
        samples = samples[np.argmax(log_probability)]
        return samples

    def format_prompt(self, prompt):
        formatted_prompt = [
            {"role": prompt.primary_role, "content": prompt.primary_role_content},
            {
                "role": prompt.secondary_role,
                "content": [
                    {"type": content_item.type, "text": content_item.text}
                    for content_item in prompt.secondary_role_content.content_items
                ],
            },
        ]
        return formatted_prompt

    def generate_response(self, prompt, sampling_params):
        completion = self.language_model_client.chat.completions.create(
            model=self.language_model, messages=prompt, **sampling_params
        )

        # generated_samples = [completion.choices[i].message.content.strip().lower() for i in range(sampling_params['n'])]
        generated_samples = [
            completion.choices[i].message.content for i in range(sampling_params["n"])
        ]

        # calculate mean log prob across tokens
        mean_log_probs = [
            np.mean([lp.logprob for lp in completion.choices[i].logprobs.content])
            for i in range(sampling_params["n"])
        ]
        return generated_samples, mean_log_probs, completion.usage.total_tokens
