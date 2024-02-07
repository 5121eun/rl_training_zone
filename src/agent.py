#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from training_zone.srv import Step

from model import PPO

import time

class Agent(Node): 
  def __init__(self):
    super().__init__('agent')
    
    self.model = PPO()

    self.step_client = self.create_client(Step, 'step')
    while not self.step_client.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('service not available, waiting again...')
    
  def process(self, n_rollout: int = 20):
  
    
    for epoch in range(500):
      done = False
      init = True
      
      state = list()
      next_state = list()
  
      time.sleep(1.0)
      while not done:
        buffer = []
        score = 0
        
        for i in range(n_rollout):
          state = next_state
      
          req = Step.Request()
          action, action_prob = (1, 1.0) if init else self.model.get_action(state)
          req.action = action
          req.init = init

          res = self.step_client.call_async(req)
          
          rclpy.spin_until_future_complete(self, res)
          
          next_state = res.result().state
          reward = res.result().reward
          done = res.result().done

          if not init:
            self.model.add_data((state, action, reward, next_state, done, action_prob))
            score += reward
          else:
            init = False
            
          if done:
            break
          
          time.sleep(0.05)
        
        loss = self.model.update()
        self.get_logger().info(f'loss: {loss:.2f}')

def main(args=None):
    rclpy.init(args=args)

    agn = Agent()
    
    agn.process()
    
    env.destroy_node()
    
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
