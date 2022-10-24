from math import pi as  PI
import math

import pymunk
from pymunk import Vec2d
from util import *

MAX_REWARD = 10
MIN_DISTANT_FROM_BOX = 40
MIN_DISTANT_FROM_BALL = 60
MIN_REWARD = -100
PICK_UP_PARAMETER = 10
SHOT_PARAMETER = 15
VELOCITY_CHANGE = 0.95

COLOR_ROBOT = (204, 0, 153, 0)
COLOR_BASKET_CONNECTOR = (255, 57, 58, 0)
COLOR_BASKET_BOARD = (225, 225, 225, 0)
COLOR_BASKET_LIMITS = (0, 0, 0, 0)
COLOR_HANDS = (102, 0, 102, 0)
COLOR_ARMS = (204, 204, 0, 0)
COLOR_BALL = (255, 110, 0, 0)
COLOR_WHEELS = (255, 255, 0, 0)

COLL_TYPES = {'box': 1, 'robot': 2, 'ball': 4}
STAGES = {'normal': 0, 'go_to': 1,'pick_up': 2, 'throw': 3}


def _get_cos_sin_angle(angle, length):
    return length * Vec2d(math.cos(angle), math.sin(angle))


class Robot:
    def __init__(self, space, size):
        self.box_d = 0
        Box(space, p1=size, d=self.box_d)
        self.space = space
        self.size = self.w, self.h = size

        # radius
        self.wheel_radius = 10
        self.ball_radius = 20
        # angles
        self.angleArms, self.angleHands = 0, PI/2
        self.min_angle_arms , self.max_angle_arms = -PI/2, 0
        self.min_angle_hands, self.max_angle_hands = -PI/3, PI/2
        self.angle_change = PI/30
        # power
        self.pow_throw = 0
        self.max_pow = 1500
        self.pow_change = 100
        # length
        self.robot_size_x, self.robot_size_y = 60, 80
        self.lengthHands, self.lengthArms = 40, 60

        self.move_robot_change = 10

        self.stage = 0 # 0 - normal, 1 - go to shooting position, 2 - robot pick up the ball, 3 - robot throw the ball

        # all the positions of the objects in the screen define by the screen size such that we can change the screen
        # size and all the position also change with out involve and set each position separate.
        self.robot_initial_pos = Vec2d(480, self.h - self.box_d - self.robot_size_y/2 - self.wheel_radius)
        self.robot_vs = [(-self.robot_size_x/2, self.robot_size_y/2), (self.robot_size_x/2, self.robot_size_y/2),
                         (self.robot_size_x/2, -self.robot_size_y/2), (-self.robot_size_x/2, -self.robot_size_y/2)]
        self.v0, self.v1, self.v2, self.v3 = self.robot_vs

        self.ball_initial_pos = self.w - 100, self.h - self.box_d - self.ball_radius

        self.bodies = []

        # add robot body
        self.robot_body = Poly(space, self.robot_initial_pos, self.robot_vs, group=COLL_TYPES['robot'],
                               color=COLOR_ROBOT)
        self.bodies.append(self.robot_body.body)

        self.__add_ball()

        self.__add_wheels()

        self.__add_arms_and_hands()

        # positions of shooting, basket
        self.shooting_position = self.robot_initial_pos
        self.basket_position = Vec2d(790, 215)
        # position of the limit of the basket
        self.left_limit_basket_pos,  self.right_limit_basket_pos = (750, 180) , (830, 180)
        # vectors of the limit's angles of the basket
        self.vec_left_limit_basket, self.vec_right_limit_basket = (15, 70), (-15, 70)
        self.basket_board_pos, self.vec_board_basket = (845, 125), (0, 115)
        self.basket_connector_pos, self.vec_basket_connector = (845, 180), (-15, 0)
        self.basket_segments = [] # list of objects of segment limits of the basket

        self.__add_basket()

        self.num_shoot = 0 # shoot numbers the robot success
        self.path = [] # path of the ball during a shot
        self.closest_point = (math.inf, math.inf) # the closest point to the basket in last shot?
        self.max_point = (math.inf, math.inf)

        # distant of the ball from the robot, use in the first stage the robot moving towards the ball
        self.distant = self.ball.body.position[0] - self.robot_body.body.position[0]

        # to render in basketball
        self.counter_picks_up = 0
        self.counter_throws = 0
        self.counter_shoots = 0

        self.shooting_values = dict()


    def __add_wheels(self):
        # left wheel
        self.wheel1 = Circle(self.space, self.robot_initial_pos + self.v0, self.wheel_radius, group=COLL_TYPES['robot'],
                             color=COLOR_WHEELS)
        self.bodies.append(self.wheel1.body)
        # right wheel
        self.wheel2 = Circle(self.space, self.robot_initial_pos + self.v1, self.wheel_radius, group=COLL_TYPES['robot'],
                             color=COLOR_WHEELS)
        self.bodies.append(self.wheel2.body)

    def __add_ball(self):
        self.ball = Circle(self.space, self.ball_initial_pos, self.ball_radius, group=COLL_TYPES['ball'],
                           color=COLOR_BALL)
        self.ball.body.body_type = pymunk.Body.DYNAMIC

    def __add_arms_and_hands(self):
        # left arm
        v_arms = _get_cos_sin_angle(self.angleArms, self.lengthArms)
        self.left_arm = Segment(self.space, self.robot_initial_pos, v_arms, color=COLOR_ARMS,
                                group=COLL_TYPES['ball'])
        self.bodies.append(self.left_arm.body)

        # left hand
        v_hands = _get_cos_sin_angle(0, self.lengthHands)
        self.left_hand = Segment(self.space, self.robot_initial_pos + v_arms, v_hands, group=COLL_TYPES['ball'],
                                 color=COLOR_HANDS)
        self.left_hand.body.angle = self.angleHands
        self.bodies.append(self.left_hand.body)

        # right arm
        self.right_arm = Segment(self.space, self.robot_initial_pos, v_arms, color=COLOR_ARMS,
                                 group=COLL_TYPES['ball'])
        self.bodies.append(self.right_arm.body)

        # right hand
        self.right_hand = Segment(self.space, self.robot_initial_pos + v_arms, v_hands,
                                  color=COLOR_HANDS, group=COLL_TYPES['ball'])
        self.right_hand.body.angle = self.angleHands
        self.bodies.append(self.right_hand.body)

    def __add_basket(self):
        self.basket_segments.append(Segment(self.space, self.left_limit_basket_pos, self.vec_left_limit_basket, 0,
                                            color=COLOR_BASKET_LIMITS))
        self.basket_segments.append(Segment(self.space, self.right_limit_basket_pos, self.vec_right_limit_basket, 0,
                                            color=COLOR_BASKET_LIMITS))
        self.basket_segments.append(Segment(self.space, self.basket_board_pos, self.vec_board_basket, 0,
                                            color=COLOR_BASKET_BOARD))
        self.basket_segments.append(Segment(self.space, self.basket_connector_pos, self.vec_basket_connector, 0,
                                            color=COLOR_BASKET_CONNECTOR))
        for s in self.basket_segments:
            s.body.body_type = pymunk.Body.STATIC


    def set_shooting_values(self, state):
        """
        save the state that make the robot shoot rhe ball into the basket
        :param state:  arms angle, hands angle, power of the throw
        :return:
        """
        self.shooting_values[self.shooting_position] = state

    def update(self):
        """
        update the robot and ball
        :return:
        """
        self.ball.body.velocity = self.ball.body.velocity * VELOCITY_CHANGE  # make the ball stop moving

        # update distant between the robot and the ball
        self.distant = self.ball.body.position[0] - self.robot_body.body.position[0]

        # detaches the ball from the others bodies of the robot when the ball is thrown
        if self.stage not in [STAGES['pick_up'], STAGES['go_to']] and self.bodies[-1] == self.ball.body:
            self.bodies = self.bodies[:-1]

        if self.stage == STAGES['throw']:
            # draw the shot path of the ball
            d = Dot(self.space, self.ball.body.position)
            if abs(d.body.position - self.basket_position) < abs(self.closest_point - self.basket_position):
                self.closest_point = d.body.position
            if d.body.position[1] < self.max_point[1]:self.max_point = self.ball.body.position
            self.path.append(d.body)
            self.path.append(d.shape)

            x , y = self.ball.body.velocity
            # case that the ball static and move to stage normal such that the robot go to pick up the ball
            if (int(x), int(y)) <= (1 ,1) and (self.ball.body.position[1] > self. h  - self.ball_radius - 1 or
                (self.ball.body.position[1] > self.robot_body.body.position[1] -self.robot_size_y/2-self.ball_radius - 2
                 and self.robot_body.body.position[0] - self.robot_size_x/2 <= self.ball.body.position[0] <=
                                               self.robot_body.body.position[0] + self.robot_size_x/2)) :
                self.space.remove(*self.path)  # remove the points of the shot path
                # initialize the list of the points to be empty to the next time the robot will throw the ball
                self.path = []
                self.ball.body.velocity = (0, 0)
                self.reset()

    def set_shooting_position(self, pos):
        self.shooting_position = pos

    def is_shooting(self):
        return abs(self.basket_position - self.closest_point) < SHOT_PARAMETER

    def throw(self, state):
        """
        function that used by ga algorithm, pass the part of the robot pick up the ball and throw the ball with
        the parameter in state the function receive.
        :param state: angleArms, angleHands, power
        :return:
        """
        x, angleArms, angleHands, power = state

        self.__move_pick_up()
        self.counter_picks_up -= 1

        pos = self.robot_body.body.position =   Vec2d(x, self.robot_body.body.position[1])
        self.wheel1.body.position , self.wheel2.body.position = pos + self.v0, pos + self.v1
        self.left_arm.body.position = self.right_arm.body.position = pos
        self.left_hand.body.angle = self.right_hand.body.angle = self.angleHands = angleHands
        self.left_arm.body.angle = self.right_arm.body.angle = self.angleArms = angleArms
        vec_arms = _get_cos_sin_angle(self.angleArms , self.lengthArms)
        self.left_hand.body.position = self.right_hand.body.position = pos + vec_arms
        self.ball.body.body_type = pymunk.Body.KINEMATIC # update the ball be kinematic
        self.__update_center_ball()
        self.pow_throw = power

        self.__move_throw()



    def _reset_hands_and_arms(self):
        self.left_arm.body.position = self.right_arm.body.position = self.robot_body.body.position
        self.angleHands = self.max_angle_hands
        self.angleArms = self.max_angle_arms
        vec_arms = _get_cos_sin_angle(self.angleArms , self.lengthArms)
        self.left_hand.body.position = self.right_hand.body.position =  self.robot_body.body.position + vec_arms
        self.left_hand.body.angle = self.right_hand.body.angle = self.angleHands
        self.left_arm.body.angle = self.right_arm.body.angle = self.angleArms

    def reset(self):
        # return all the object to the initial position and the initial angles
        # self.robot_body.body.position = self.robot_initial_pos
        # self.wheel1.body.position , self.wheel2.body.position = self.robot_initial_pos + self.v0,\
        #                                                         self.robot_initial_pos + self.v1
        self._reset_hands_and_arms()
        if self.ball.body.position < self.robot_body.body.position or self.ball.body.position[0] > self.w - 100:
            self.ball.body.position = self.ball_initial_pos
        else: self.ball.body.position = round(self.ball.body.position[0]/10) * 10, self.ball.body.position[1]
        self.ball.body.velocity = 0, 0
        self.pow_throw = 0
        self.stage = STAGES['normal']



    def __get_state(self):
        if self.stage == STAGES['normal'] and self.distant > MIN_DISTANT_FROM_BALL: return self.distant, self.stage
        if self.stage == STAGES['normal']: return self.distant, self.angleArms, self.angleHands, self.stage
        if self.stage == STAGES['go_to']: return  self.robot_body.body.position, self.stage
        if self.stage == STAGES['pick_up']: return self.angleHands, self.angleArms, self.pow_throw, self.stage
        return self.stage, # throw stage

    def getCurrentState(self):
        return self.__get_state()

    def _get_edge_hands_pos(self, pos, angle1, angle2):
        return pos + _get_cos_sin_angle(angle1, self.lengthArms)\
               + _get_cos_sin_angle(angle2, self.lengthHands)

    def __get_hands_and_arms_actions(self, arms_angle, hands_angle):
        actions = []
        if arms_angle < self.max_angle_arms: actions += ['arms_down']
        if arms_angle > self.min_angle_arms: actions += ['arms_up']
        if hands_angle < self.max_angle_hands: actions += ['hands_down']
        if hands_angle > self.min_angle_hands: actions += ['hands_up']
        return actions

    def getPossibleActions(self, state):
        stage = state[-1]
        if stage == STAGES['throw']:
            return ['none']

        if stage == STAGES['go_to']:
            robot_pos = state[0]
            if robot_pos[0] > self.shooting_position[0]: return ['left']
            if robot_pos[0] < self.shooting_position[0]: return ['right']
            self.stage = STAGES['pick_up'] # in case robot position is equal to shooting position.
            return ['none']

        if stage == STAGES['pick_up']:
            arms_angle, hands_angle, power, stage = state
            actions = self.__get_hands_and_arms_actions(arms_angle, hands_angle)
            if power < self.max_pow: return  actions + ['pow_up', 'throw']
            return actions + ['throw'] # in case power = max_pow = 1500

        # case stage is normal - 0
        robot_pos = self.robot_body.body.position
        ball_pos = self.ball.body.position
        angle_arms, angle_hands = self.angleArms, self.angleHands
        dist = state[0]
        # if distant less 60 the state include the angles of the hands an arms
        if dist <= MIN_DISTANT_FROM_BALL: angle_arms, angle_hands = state[1], state[2]
        actions = self.__get_hands_and_arms_actions(angle_arms, angle_hands)
        # calculate the edge hands position
        edge_hands_pos = self._get_edge_hands_pos(robot_pos, angle_arms, angle_hands)
        # if the distant of the edge of the hands and the ball is less then 10 the robot can pick up the ball and
        # the robot can't move right, to not collide with the ball
        if abs(edge_hands_pos - ball_pos) < PICK_UP_PARAMETER: return actions + ['pick_up', 'left']
        # if abs(edge_hands_pos - ball_pos) < 10: return actions + ['left']
        # the robot can collide with the border or the ball
        if edge_hands_pos[0] < self.size[0] and self.distant > MIN_DISTANT_FROM_BALL: actions += ['right']
        if robot_pos[0] - MIN_DISTANT_FROM_BOX > 0: actions += ['left']
        return actions

    def doAction(self, action):
        if action == 'none':
            return self.getCurrentState(), 0
        if action == 'left':
            return self.__move_left_or_right(True)
        elif action == 'right':
            return self.__move_left_or_right(False)
        elif action == 'arms_up':
            return self.__move_hands_or_arms(True, True)
        elif action == 'arms_down':
            return self.__move_hands_or_arms(True, False)
        elif action == 'hands_up':
            return self.__move_hands_or_arms(False, True)
        elif action == 'hands_down':
            return self.__move_hands_or_arms(False, False)
        elif action == 'pick_up':
            return self.__move_pick_up()
        elif action == 'pow_up':
            return self.__move_pow_up()
        elif action == 'throw':
            return self.__move_throw()

    def __move_throw(self):
        self.max_point = (math.inf, math.inf)
        self.closest_point = (math.inf, math.inf)
        self.counter_throws += 1 # update the numbers of the ball throws
        self.stage = STAGES['throw']  # update the stage to be 'throw'
        # disconnects the ball from the other bodies
        if self.bodies[-1] == self.ball.body: self.bodies = self.bodies[:-1]
        self.ball.body.body_type = pymunk.Body.DYNAMIC # update the ball be dynamic after it was kinematic
        # update the velocity daring shot time
        self.ball.body.velocity = _get_cos_sin_angle(self.angleHands, 1) * self.pow_throw

        if self.shooting_position not in self.shooting_values:
            return self.getCurrentState(), 0 # return the next state and the reward
        return self.getCurrentState(), \
               -abs(self.shooting_values[self.shooting_position][0] - self.angleArms) \
               - abs(self.shooting_values[self.shooting_position][1] - self.angleHands) \
               - -abs(self.pow_throw - self.shooting_values[self.shooting_position][2])

    def __move_pow_up(self):
        self.pow_throw = min(self.pow_throw + self.pow_change, self.max_pow) # increases power by 100
        if self.shooting_position not in self.shooting_values:
            return self.getCurrentState(), 0 # return next state and reward
        return self.getCurrentState(), self.shooting_values[self.shooting_position][2] - self.pow_throw
    def __move_pick_up(self):
        self.counter_picks_up += 1 # update the numbers of ball pick ups
        # set the stage to be 'goto' or 'pick_up'
        if self.robot_body.body.position[0] != self.shooting_position[0]: self.stage = STAGES['go_to']
        else: self.stage = STAGES['pick_up']
        # set the ball be kinematic body such that it is not affected by defined physical factors in the world
        self.ball.body.body_type = pymunk.Body.KINEMATIC
        self.ball.body.velocity = 0, 0 # set the velocity
        self.bodies.append(self.ball.body) # connects the ball to the other bodies
        self._reset_hands_and_arms()
        self.__update_center_ball()
        return self.getCurrentState(), 0


    def __move_left_or_right(self, left):
        # calculate hands edge position
        edge = self.left_hand.body.position + _get_cos_sin_angle(self.angleHands, self.lengthHands)
        robot_pos = self.robot_body.body.position
        # set robot after left or right action
        if left:
            for body in self.bodies:
                body.position = body.position[0] - self.move_robot_change, body.position[1]
        else:
            for body in self.bodies:
                body.position = body.position[0] + self.move_robot_change, body.position[1]

        # return next state and reward adjust to the stage and different considerations
        # in case the robot goes to the shot position and reached the position
        if self.stage == STAGES['go_to'] :
            if self.robot_body.body.position == self.shooting_position: self.stage = STAGES['pick_up']
            return  self.getCurrentState(), 0
        # in case the robot throw or pick up the ball
        if self.stage == STAGES['throw'] or self.stage == STAGES['pick_up']: return self.getCurrentState(), MIN_REWARD
        # in case the distant between the robot and the ball bigger then 60 or the robot move left
        if self.distant > MIN_DISTANT_FROM_BALL or left:
            return self.getCurrentState(), self.robot_body.body.position[0] - robot_pos[0]
        # in case the distant between the previous hands edge position less then 10 and left,
        # we prefer do another actions
        if left and abs(self.ball.body.position - edge) < PICK_UP_PARAMETER: return self.getCurrentState(), MIN_REWARD
        # calculate the new hands edge position
        new_edge = self.left_hand.body.position + _get_cos_sin_angle(self.angleHands, self.lengthHands)
        # returns the difference between the distances in case stage is normal
        return self.getCurrentState(),  abs(self.ball.body.position - edge) - abs(self.ball.body.position - new_edge)

    def __update_center_ball(self):
        if self.stage in [STAGES['pick_up'], STAGES['go_to']]:
            self.ball.body.position = self.left_hand.body.position +\
                                      _get_cos_sin_angle(self.angleHands, self.lengthHands)

    def __update_angle(self, is_arm, up):
        if is_arm:
            if up: self.angleArms = max(self.min_angle_arms, self.angleArms - self.angle_change) # raises the angle of the arms
            else: self.angleArms = min(self.max_angle_arms, self.angleArms + self.angle_change) # lower the angle of the arms
            # set the angles
            self.left_arm.body.angle = self.right_arm.body.angle = self.angleArms
        else:
            if up: self.angleHands = max(self.min_angle_hands, self.angleHands - self.angle_change) #raises the angle of the hands
            else: self.angleHands = min(self.max_angle_hands, self.angleHands + self.angle_change) #lower the angle of the hands
            # set angles
            self.left_hand.body.angle = self.right_hand.body.angle = self.angleHands
        self.__update_center_ball()

    def __move_hands_or_arms(self, is_arm, up):
        edge = self.left_hand.body.position + _get_cos_sin_angle(self.angleHands, self.lengthHands)
        angleArms, angleHands = self.angleArms, self.angleHands
        self.__update_angle(is_arm, up) # update the angles
        if is_arm: # if is arms we set also the hans position
            self.left_hand.body.position = self.right_hand.body.position = self.left_arm.body.position + \
                                           _get_cos_sin_angle(self.angleArms, self.lengthArms)

        # return next state and reward in different cases
        if self.stage == STAGES['pick_up']: # return the different between the good angle to robot angle
            if self.shooting_position not in self.shooting_values: return self.getCurrentState(), 0
            if is_arm:
                return self.getCurrentState(), \
                       abs(angleArms - self.shooting_values[self.shooting_position][0]) -\
                       abs(self.angleArms - self.shooting_values[self.shooting_position][0])
            return self.getCurrentState(), \
                   (abs(angleHands - self.shooting_values[self.shooting_position][1]) -
                   abs(self.angleHands - self.shooting_values[self.shooting_position][1])) * 100
        if self.stage == STAGES['throw'] or self.stage == STAGES['pick_up']: return self.getCurrentState(), -MAX_REWARD
        # if the distant between the hands edge position from the ball less than 10 we prefer to choose pick up action
        if abs(self.ball.body.position - edge) < PICK_UP_PARAMETER or up: return self.getCurrentState(), -MAX_REWARD
        # also we prefer down action so the ball more close to the edge of the hands
        if self.distant <= MIN_DISTANT_FROM_BALL and not up: return self.getCurrentState(), MAX_REWARD
        # else - returns the difference between the distances
        new_edge = self.left_hand.body.position + _get_cos_sin_angle(self.angleHands, self.lengthHands)
        return self.getCurrentState(), abs(self.ball.body.position - edge) - abs(self.ball.body.position - new_edge)


