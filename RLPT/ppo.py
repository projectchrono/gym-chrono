import numpy as np
import torch
import torch.optim as optim
import datetime
import Utils



class PPO:
    def __init__(self,
        model,
        envs,
        device,
        modelpath,
        lr = 3e-4,
        tuple_ob = False):

        self.envs = envs
        self.optimizer = optim.Adam(model.parameters(), lr=lr)
        self.model = model
        self.modelpath = modelpath
        self.device = device
        self.tuple_ob = tuple_ob

    def compute_gae(self, next_value, rewards, masks, values, gamma=0.99, tau=0.95):
        values = values + [next_value]
        gae = 0
        returns = []
        for step in reversed(range(len(rewards))):
            # evaluate the n-ith Temporal Difference. The mask value is 0 whenever the step is terminal (done=True), so Vt+1 = 0
            delta = rewards[step] + gamma * values[step + 1] * masks[step] - values[step]
            gae = delta + gamma * tau * masks[step] * gae
            returns.insert(0, gae + values[step])
        return returns


    def ppo_iter(self, mini_batch_size, states, actions, log_probs, returns, advantage):
        batch_size = returns.size(0)
        ids = np.random.permutation(batch_size)
        ids = np.split(ids, batch_size // mini_batch_size)
        for i in range(len(ids)):
            if self.tuple_ob:
                yield [s[ids[i], :] for s in states], actions[ids[i], :], log_probs[ids[i], :], returns[ids[i], :], advantage[ids[i], :]
            else:
                yield states[ids[i], :], actions[ids[i], :], log_probs[ids[i], :], returns[ids[i], :], advantage[ids[i], :]



    def ppo_update(self, ppo_epochs, mini_batch_size, states, actions, log_probs, returns, advantages, clip_param=0.2):
        for _ in range(ppo_epochs):
            for state, action, old_log_probs, return_, advantage in self.ppo_iter(mini_batch_size, states, actions, log_probs, returns, advantages):
                dist, value = self.model(state)
                entropy = dist.entropy().mean()
                new_log_probs = dist.log_prob(action)

                ratio = (new_log_probs - old_log_probs).exp()
                surr1 = ratio * advantage
                surr2 = torch.clamp(ratio, 1.0 - clip_param, 1.0 + clip_param) * advantage

                actor_loss  = - torch.min(surr1, surr2).mean()
                critic_loss = (return_ - value).pow(2).mean()

                loss = 0.5 * critic_loss + actor_loss - 0.001 * entropy

                self.optimizer.zero_grad()
                loss.backward()
                self.optimizer.step()

    def ppo_train(self, num_steps, mini_batch_size, ppo_epochs = 8,
                  max_frames = np.inf, max_pol_updates = 200,save_interval = 10, increasing_length = 0,
                  test_interval = 40, savepath='./'):

        frame_idx = 0
        pol_updates = 0
        test_rewards = []
        early_stop = False
        state = self.envs.reset()
        mean_rewards = []

        while frame_idx < max_frames and pol_updates < max_pol_updates and not early_stop:
            log_probs = []
            values    = []
            states    = []
            actions   = []
            rewards   = []
            masks     = []
            entropy = 0

            for step_number in range(num_steps):
                if self.tuple_ob:
                    arr_l = []
                    for i in range(state.shape[1]):
                        arr = np.stack( state[:,i][:] )
                        arr_l.append(arr)

                    state = [torch.FloatTensor(arr).to(self.device) for arr in arr_l]
                else:
                    state = torch.FloatTensor(state).to(self.device)
                dist, value = self.model(state)

                action = dist.sample()
                next_state, reward, done, _ = self.envs.step(action.cpu().numpy())

                log_prob = dist.log_prob(action)
                entropy += dist.entropy().mean()

                log_probs.append(log_prob)
                values.append(value)
                rewards.append(torch.FloatTensor(reward).unsqueeze(1).to(self.device))
                masks.append(torch.FloatTensor(1 - done).unsqueeze(1).to(self.device))

                states.append(state)
                actions.append(action)

                state = next_state
                frame_idx += 1

            if frame_idx % test_interval == 0:
                mean_reward = Utils.CalcMeanRew(rewards, masks)
                mean_rewards.append(mean_reward)

            if self.tuple_ob:
                arr_l = []
                for i in range(next_state.shape[1]):
                    arr = np.stack(next_state[:, i][:])
                    arr_l.append(arr)

                next_state = [torch.FloatTensor(arr).to(self.device) for arr in arr_l]
            else:
                next_state = torch.FloatTensor(next_state).to(self.device)
            _, next_value = self.model(next_state)
            returns = self.compute_gae(next_value, rewards, masks, values)

            returns   = torch.cat(returns).detach()
            log_probs = torch.cat(log_probs).detach()
            values    = torch.cat(values).detach()
            if self.tuple_ob:
                states    = Utils.cat_tuple_ob(states, state.shape[1])
            else:
                states    = torch.cat(states)
            actions   = torch.cat(actions)
            advantage = returns - values

            self.ppo_update(ppo_epochs, mini_batch_size, states, actions, log_probs, returns, advantage)
            pol_updates += 1
            if increasing_length:
                num_steps += pol_updates * increasing_length
            # if np.asarray(mean_rewards)[-1] >= 15000:
            #     print('Threshold Met!!!')
            #     torch.save(self.model.state_dict(), self.modelpath)
            #     print("Policy Saved after " + str(pol_updates) + "updates \n")
            #     print(datetime.datetime.now().time())
            #     np.save(savepath+'rew', np.asarray(mean_rewards))
            #     exit(0)
            if (pol_updates)%save_interval == 0:
                torch.save(self.model.state_dict(), self.modelpath)
                print("Policy Saved after " + str(pol_updates) + "updates \n")
                print(datetime.datetime.now().time())
            np.save(savepath+'rew', np.asarray(mean_rewards))
            print('Update {0} done. Mean reward is {1}'.format(pol_updates, np.asarray(mean_rewards)[-1]))
