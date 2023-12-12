from tensorflow.python.summary.summary_iterator import summary_iterator
import os
import matplotlib.pyplot as plt
import re


def extract_data_from_event_file(event_file_path, tag):
    data = []
    for e in summary_iterator(event_file_path):
        for v in e.summary.value:
            if v.tag == tag:
                data.append((e.step, v.simple_value))
    return data


# Replace with your TensorBoard log directory and the tag you are interested in
log_dir = '../art_logs_3/'

tags = ['rollout/total_episode_num', 'rollout/total_crashes',
        'rollout/total_fallen', 'rollout/total_success', 'rollout/total_timeout']
# tag = 'rollout/ep_rew_mean'
# tag = 'train/entropy_loss'
# tag = 'train/approx_kl'
# tag = 'train/explained_variance'
# tag = 'train/policy_gradient_loss'
# tag = 'train/value_loss'


# Custom sorting function
def numeric_sort_key(s):
    return [int(text) if text.isdigit() else text.lower() for text in re.split('([0-9]+)', s)]


all_data = []
# Walk through the base directory
for root, dirs, files in os.walk(log_dir, topdown=True):
    # Sort directories numerically
    dirs.sort(key=numeric_sort_key)
    for dir in sorted(dirs, key=numeric_sort_key):
        print(dir)
        dir_path = os.path.join(root, dir)
        for file in sorted(os.listdir(dir_path), key=numeric_sort_key):
            if file.startswith("events.out.tfevents"):
                event_file_path = os.path.join(dir_path, file)
                for tag in tags:
                    all_data.extend(
                        extract_data_from_event_file(event_file_path, tag))

                    # Sorting the data by step
                    all_data.sort(key=lambda x: x[0])
                    steps, values = zip(*all_data)
                    print(values[-1])

                    # index = list(range(len(steps)))

# plt.figure(figsize=(10, 5))
# plt.plot(index, values, label=tag)
# plt.xlabel('Step')
# plt.ylabel('Value')
# plt.title(f'Training Progress for {tag}')
# plt.legend()
# plt.show()
