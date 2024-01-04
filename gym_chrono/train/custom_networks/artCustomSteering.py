import torch as th
from torch import nn
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor


class CustomCombinedExtractor(BaseFeaturesExtractor):
    def __init__(self, observation_space, features_dim=10):
        super(CustomCombinedExtractor, self).__init__(
            observation_space, features_dim)
        extractors = {}
        for key, space in observation_space.spaces.items():
            if key == "lidar":
                features_dim = 10
                extractors[key] = nn.Sequential(
                    nn.Linear(90, 256),
                    nn.ReLU(),
                    nn.Linear(256, 128),
                    nn.ReLU(),
                    nn.Linear(128, features_dim),
                    nn.ReLU()
                )
            else:
                extractors[key] = nn.Sequential(
                    # Assuming the additional features are a flat vector
                    nn.Linear(4, 20),
                    nn.ReLU()
                )
        self.extractors = nn.ModuleDict(extractors)
        self._features_dim = 20

    def forward(self, observations):
        encoded_tensor_list = []

        # self.extractors contain nn.Modules that do all the processing.
        for key, extractor in self.extractors.items():
            encoded_tensor_list.append(extractor(observations[key]))
        # Return a (B, self._features_dim) PyTorch tensor, where B is batch dimension.
        return th.cat(encoded_tensor_list, dim=1)
