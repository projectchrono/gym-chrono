import gymnasium as gym
from gym_chrono.envs.wheeled.off_road_art_steering import off_road_art
from stable_baselines3.common.env_checker import check_env

render = True
additional_render = True
if __name__ == '__main__':

    # Add the agent POV as a render mode
    if additional_render:
        env = off_road_art(additional_render_mode='agent_pov')
    else:
        env = off_road_art()
    # check_env(env)
    # Set the mode to render for nice viz
    env.set_nice_vehicle_mesh()
    obs, _ = env.reset()
    if render:
        env.render('follow')

    print(env.observation_space)
    print(env.action_space)
    print(env.action_space.sample())

    # Hardcoded best agent: always go left!
    n_steps = 1000000
    for step in range(n_steps):
        print(f"Step {step + 1}")
        if (step < 100):
            obs, reward, terminated, truncated, info = env.step([0.0, 0.7])
        elif (step > 100):
            obs, reward, terminated, truncated, info = env.step(
                [0., 0.7, 0.])
        print(obs)
        print("Terminated=", terminated, "Truncated=", truncated)
        done = terminated or truncated
        if render:
            env.render('follow')
        if done:
            print("reward=", reward)
            break
