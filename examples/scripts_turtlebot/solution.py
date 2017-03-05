
#!/usr/bin/env python

'''
Based on:
https://github.com/vmayoral/basic_reinforcement_learning
https://gist.github.com/wingedsheep/4199594b02138dd427c22a540d6d6b8d
'''
import sys
import gym
import gym_gazebo
from dqn import Agent
import memory
import random
import numpy

if __name__ == '__main__':

    #REMEMBER!: turtlebot_nn_setup.bash must be executed.
    env = gym.make('GazeboFlatTurtlebotLidarNn-v0')
    outdir = '/tmp/gazebo_gym_experiments/'
    num_episodes = 1000

    env = gym.make('GazeboFlatTurtlebotLidarNn-v0')

    agent = Agent(state_size=env.observation_space.shape,
                  number_of_actions=env.action_space.n,
                  save_name=env_name)

    for e in xrange(num_episodes):
        observation = env.reset()
        done = False
        agent.new_episode()
        total_cost = 0.0
        total_reward = 0.0
        frame = 0
        while not done:
            frame += 1
            #env.render()
            action, values = agent.act(observation)
            #action = env.action_space.sample()
            observation, reward, done, info = env.step(action)
            total_cost += agent.observe(reward)
            total_reward += reward
        print "total reward", total_reward
        print "mean cost", total_cost/frame

env.monitor.close()
env.close()
