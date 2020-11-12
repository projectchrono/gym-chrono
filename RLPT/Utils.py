import matplotlib.pyplot as plt
import torch
import numpy as np

def plot(frame_idx, rewards):
    plt.figure(figsize=(20,5))
    plt.subplot(131)
    plt.title('frame %s. reward: %s' % (frame_idx, rewards[-1]))
    plt.plot(rewards)
    plt.show()

def CalcMeanRew(rew, not_dones):
    rewarr = np.asarray([line.cpu().numpy() for line in rew])
    not_donearr = np.asarray([line.cpu().numpy() for line in not_dones])
    # Get an array which is = 1 when en episode is done
    donearr = np.ones(not_donearr.shape) - not_donearr
    rewsum = 0
    undone_ep = 0
    # Iterating over parallel episodes
    for i in range(donearr.shape[1]):
        done_ind = np.nonzero(donearr[:,i,:])
        # If an environment times out before being done even once, it is counted as one episode
        if done_ind[0].size == 0:
            last_done = -1
            undone_ep += 1
        # If not, we consider the rewards until the
        else:
            last_done = np.amax(done_ind[0])
        rewsum += np.sum( rewarr[0:last_done,i,:] )
    # Divide the total reward by the number of episodes
    mean_reward = rewsum / (np.count_nonzero(donearr) + undone_ep )
    return mean_reward

def cat_tuple_ob(states, tuple_len):
    l1 = []
    for i in range(tuple_len):
        l2 = []
        for j in range(len(states)):
            l2.append(states[j][i][:])
        l1.append(l2)
    state_l = [torch.cat(s) for s in l1]
    return state_l