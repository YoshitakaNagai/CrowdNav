from crowd_sim.envs.utils.agent import Agent
from crowd_sim.envs.utils.state import JointState
from crowd_nav.policy.ddqn import Network

class Robot(Agent):
    def __init__(self, config, section):
        super().__init__(config, section)

    def act(self, ob_m, ob_g, ob_v):
        # if self.policy is None:
        #     raise AttributeError('Policy attribute has to be set!')
        # state = JointState(self.get_full_state(), ob)
        # action = self.policy.predict(state)
        action = 
        return action
