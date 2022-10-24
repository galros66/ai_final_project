"""This example spawns (bouncing) balls randomly on a L-shape constructed of
two segment shapes. Not interactive.
"""
NAVY = (0, 0, 102)

__version__ = "$Id:$"
__docformat__ = "reStructuredText"

# Python imports

# Library imports
import pygame
from math import pi as PI

# pymunk imports
from pymunk.pygame_util import *
import pymunk
from pygame.locals import *
from util import *

from PIL import Image

from robot import Robot
from learning import qlearningAgents

from genetic import GA

import random


size = w, h = 1000, 430
SIZE = W, H = w + 180, h + 132
fps = 30
steps = 10

BLACK = (0, 0, 0)
GRAY = (220, 220, 220)
WHITE = (255, 255, 255)

background_image = pygame.image.load(r'background2.jpeg')


str_robot_pos = 'Robot position: {}'
str_arms_angle = 'Arms angle: {}'
str_hands_angle = 'Hands angle: {}'

str_ball_pos = 'Ball position: {}'
str_ball_velocity = 'Ball velocity: {}'
str_power = 'Power: {}'

str_steps = 'Steps: {}'
str_pick_ups = 'Ball pick ups: {}'
str_throws = 'Ball throws: {}'
str_shooting = 'Ball shooting: {}'

str_step_delay = 'Step Delay:', '%.5f'
str_epsilon = 'Epsilon:', '%.3f'
str_discount = 'Discount:', '%.3f'
str_learning_rate = 'Learning Rate:', '%.3f'


class BasketBall:

    def __init__(self):
        self.space = pymunk.Space()
        self.space.gravity = 0, 900
        pygame.init()
        self.font = pygame.font.SysFont('Corbel',16)
        self.font_bold = pygame.font.SysFont('Corbel', 16, bold =True )
        self.clock = pygame.time.Clock()
        self.fps = 60
        self.screen = pygame.display.set_mode(SIZE)
        self.draw_options = DrawOptions(self.screen)
        self.running = True
        self.robot = Robot(self.space, size)
        self.ep = 0.005
        self.ga = 2
        self.al = 2
        actionFn = lambda state: \
            self.robot.getPossibleActions(state)
        self.learner = qlearningAgents.QLearningAgent(actionFn=actionFn)
        self.tm = 1/ self.fps/ steps
        self.epsilon = self.sigmoid(self.ep)
        self.alpha = self.sigmoid(self.al)
        self.gamma = self.sigmoid(self.ga)
        self.learner.setEpsilon(self.epsilon)
        self.learner.setLearningRate(self.alpha)
        self.learner.setDiscount(self.gamma)

        self.dict_buttons_text ={str_step_delay: [self.tm],
                                 str_epsilon : [self.ep],
                                 str_discount: [self.ga],
                         str_learning_rate: [self.al]}

        self.dict_set_func = {str_step_delay: self.func, str_epsilon: self.learner.setEpsilon,
                              str_discount: self.learner.setDiscount,
                              str_learning_rate: self.learner.setLearningRate}

        self.__add_buttons()
        self.images = []
        self.gif = 0
        self.counter = 0
        self.stepCount = 0


        self.mouse = pygame.mouse.get_pos()

        self.angle_arms_throw = self.robot.min_angle_arms + random.random()* \
                                (self.robot.max_angle_arms - self.robot.min_angle_arms)
        self.angle_hands_throw = self.robot.min_angle_hands + random.random() * \
                                 (self.robot.max_angle_hands - self.robot.min_angle_hands)
        self.pow = random.randint(5, 10)
        self.action_throw_list = []
        self.in_pick_up_stage = False
        self.throw_dict = {'hands_up':  -round((self.robot.min_angle_hands - self.angle_hands_throw) * 30 / PI),
                           'arms_up': -round((self.robot.min_angle_arms - self.angle_arms_throw) * 30 / PI),
                           'pow_up': self.pow}

        self.throw_values = dict()

        self.move_index = 0

        self.ga = GA(self)

        self.mode = 0

        self.flag_ga = True
        self.flag_run_ga = False

        self.ff = False

    def func(self, val):
        self.fps = max(val, 60)
        return None
    def sigmoid(self, x):
        return 1.0 / (1.0 + 2.0 ** (-x))

    def __add_buttons(self):
        x, y = w + 90, 285
        vs = [(-80, 15), (80, 15), (80, -15), (-80, -15)]
        vs_buttons = [(-10, 10), (10, 10), (10, -10), (-10, -10)]

        for s in self.dict_buttons_text:
            Poly(self.space, (x, y), vs, ( 255, 255, 255, 0))
            p = Poly(self.space, (x - 55, y), vs_buttons, (220, 220, 220, 0))
            p.body.body_type = pymunk.Body.STATIC
            p = Poly(self.space, (x + 55, y), vs_buttons, (220, 220, 220, 0))
            p.body.body_type = pymunk.Body.STATIC

            self.dict_buttons_text[s] += [(x - 20, y -8), (x - 35, y - 32), (x - 57, y - 8),
                                          (x + 52, y - 8), (x - 55, y), (x + 55, y)]
            y += 50

        vsd = [(-80, 40), (80, 40), (80, -40), (-80, -40)]
        Poly(self.space, (x, y + 20), vsd, (230, 247, 255, 0))

    def _add_set_func(self, s):
        pass

    def reset_learning_iteration(self):
        self.in_pick_up_stage = False
        self.move_index = 0
        self.action_throw_list = []
        self.throw_action_learning()

    def set_throw_actions(self):
        if self.robot.shooting_position[0] in self.throw_values:
            self.throw_dict = self.throw_values[self.robot.shooting_position[0]]

        for a in self.throw_dict.keys():
            self.action_throw_list += [a] * self.throw_dict[a]
        self.action_throw_list += ['throw']

    def get_throw_action(self):
        if self.robot.stage == 0:
            self.reset_learning_iteration()
            return 'none'
        if self.move_index >= len(self.action_throw_list): return 'none'
        self.move_index += 1
        return self.action_throw_list[self.move_index - 1]

    def throw_action_learning(self):
        if self.robot.is_shooting():
            self.robot.counter_shoots +=1
            self.throw_values[self.robot.shooting_position[0]] = self.throw_dict
            self.throw_dict = {'hands_up': -round((self.robot.min_angle_hands - self.angle_hands_throw) * 30 / PI),
                               'arms_up': -round((self.robot.min_angle_arms - self.angle_arms_throw) * 30 / PI),
                               'pow_up': self.pow}
            self.robot.set_shooting_position(pymunk.Vec2d(random.randrange(120, 540, 20),
                                                          self.robot.shooting_position[1]))
            return
        # print(self.robot.max_point)
        if (self.robot.max_point[1] < self.robot.basket_position[1]/2 and
            self.robot.max_point[0] > self.robot.basket_position[0]) or \
                self.robot.max_point[1] < 40:
            self.throw_dict['pow_up'] = max(0, self.throw_dict['pow_up'] - 1)
        else:
            if random.random() < 0.6: self.throw_dict['pow_up'] = min(15, self.throw_dict['pow_up'] + 1)
        if (self.robot.max_point[1] < self.robot.basket_position[1]  - 100 and
            self.robot.max_point[0] < self.robot.basket_position[0])  or self.robot.max_point[1] < 40:
            if random.random() < 0.6: self.throw_dict['hands_up'] = max(0, self.throw_dict['hands_up'] - 1)
        elif self.robot.max_point[1] > self.robot.basket_position[1] - 100:
            self.throw_dict['hands_up'] = min(25, self.throw_dict['hands_up'] + 2)



    def step(self):
        self.stepCount += 1
        state = self.robot.getCurrentState()
        actions = self.robot.getPossibleActions(state)
        if len(actions) == 0.0:
            self.robot.reset()
            self.robot.ball.body.position = self.robot.ball_initial_pos
            state = self.robot.getCurrentState()
            actions = self.robot.getPossibleActions(state)
            print('Reset!')

        if not self.in_pick_up_stage : action = self.learner.getAction(state)
        else: action = self.get_throw_action()
        # action = self.learner.getAction(state)

        if action is None:
            raise Exception('None action returned: Code Not Complete')

        nextState, reward = self.robot.doAction(action)

        self.learner.observeTransition(state, action, nextState, reward)

        if state[-1] == 1 and nextState[-1] == 2:
            self.set_throw_actions()
            self.in_pick_up_stage = True


    def run_robot_pick_up_ball(self):
        while (self.robot.stage < 2 or not self.flag_ga) and self.running and not self.flag_run_ga:
            self.robot.update() # update
            self.step()
            for event in pygame.event.get():
                self.do_event(event)

            self.mouse = pygame.mouse.get_pos()
            self.draw()
            self.clock.tick(self.fps)

            for i in range(steps):
                self.space.step(1/fps/steps)
            self.clock.tick(self.fps)


    def run_robot_throw_ball(self):
        self.robot.stage = 0
        self.ga.position_x = int(self.robot.shooting_position[0])
        while self.running and self.flag_ga and not self.flag_run_ga:
            self.robot.update()
            for event in pygame.event.get():
                self.do_event(event)
            p = self.ga.step()
            if p:
                self.robot.set_shooting_values(p)
                self.robot.set_shooting_position(pymunk.Vec2d(random.randrange(120, 540, 20), self.robot.shooting_position[1]))
                return p
            self.draw()

            for i in range(steps):
                self.space.step(1 / fps / steps)

    def run(self):
        # self.robot.set_shooting_values(self.ga.run())
        while self.running:
            self.run_robot_pick_up_ball()
            if self.flag_ga and not self.flag_run_ga:
                self.run_robot_throw_ball()
            if self.flag_run_ga:
                self.ga.run()
                self.robot.reset()
                self.flag_run_ga = False

        pygame.quit()

    def do_event(self, event):
        if event.type == QUIT:
            self.running = False
            self.ga.running = False

        if event.type == pygame.MOUSEBUTTONDOWN:
            for s in self.dict_buttons_text.keys():
                self.set_parm(s)

        if event.type == KEYDOWN:
            if event.key in (K_q, K_ESCAPE):
                self.running = False
                self.ga.running = False

            if event.key == K_t:
                self.flag_ga = not self.flag_ga

            if event.key == K_g:
                self.flag_run_ga = not  self.flag_run_ga





    def set_parm(self, s):
        # if s == str_discount: print('innnn', self.dict_buttons_text[s][6])
        inc = 0.5
        if self.dict_buttons_text[s][5][0] - 10 < \
                self.mouse[0] < self.dict_buttons_text[s][5][0] + 10 and \
                self.dict_buttons_text[s][5][1] - 10 < \
                self.mouse[1] < self.dict_buttons_text[s][5][1] + 10:
             if s == str_step_delay[0]: self.dict_buttons_text[s][0] *= 2
             else: self.dict_buttons_text[s][0]-= inc
             self.dict_set_func[s](self.sigmoid(self.dict_buttons_text[s][0]))
        if self.dict_buttons_text[s][6][0] - 10 < \
                self.mouse[0] < self.dict_buttons_text[s][6][0] + 10 and \
                self.dict_buttons_text[s][6][1] - 10 < \
                self.mouse[1] < self.dict_buttons_text[s][6][1] + 10:
            if s == str_step_delay[0]: self.dict_buttons_text[s][0] *= 0.5
            else: self.dict_buttons_text[s][0] += inc
        if s == str_step_delay[0]: self.dict_set_func[s](self.dict_buttons_text[s][0])
        else: self.dict_set_func[s](self.sigmoid(self.dict_buttons_text[s][0]))


    def draw(self):
        self.screen.fill(GRAY)
        self.screen.blit(background_image, (0, 0))

        self._draw_data()

        self.space.debug_draw(self.draw_options)

        self._draw_buttons()

        pygame.display.update()

        text = f'fpg: {self.clock.get_fps():.1f}'
        pygame.display.set_caption(text)
        self.make_gif()

    def _draw_data(self):
        self.screen.blit(self.font.render(str_steps.format(self.stepCount) ,
                                          True , BLACK), (w + 10, 10))
        self.screen.blit(self.font.render('__________________' , True , BLACK), (w + 10, 20))
        self.screen.blit(self.font.render(str_robot_pos.format(self.robot.robot_body.body.position.int_tuple) ,
                                          True , BLACK), (w + 10, 40))
        self.screen.blit(self.font.render(str_arms_angle.format(round(self.robot.angleArms * 180, 2)) ,
                                          True , BLACK), (w + 10, 60))
        self.screen.blit(self.font.render(str_hands_angle.format(round(self.robot.angleHands * 180, 2)) ,
                                          True , BLACK), (w + 10, 80))
        self.screen.blit(self.font.render('__________________' , True , BLACK), (w + 10, 90))
        self.screen.blit(self.font.render(str_ball_pos.format(self.robot.ball.body.position.int_tuple) ,
                                          True , BLACK), (w + 10, 110))
        self.screen.blit(self.font.render(str_ball_velocity.format(self.robot.ball.body.velocity.int_tuple) ,
                                          True , BLACK), (w + 10, 130))
        self.screen.blit(self.font.render(str_power.format(self.robot.pow_throw) ,
                                          True , BLACK), (w + 10, 150))
        self.screen.blit(self.font.render('__________________' , True , BLACK), (w + 10, 160))
        self.screen.blit(self.font.render(str_pick_ups.format(self.robot.counter_picks_up) ,
                                          True , BLACK), (w + 10, 180))
        self.screen.blit(self.font.render(str_throws.format(self.robot.counter_throws) ,
                                          True , BLACK), (w + 10, 200))
        self.screen.blit(self.font.render(str_shooting.format(self.robot.counter_shoots) ,
                                          True , BLACK), (w + 10, 220))
        self.screen.blit(self.font.render('__________________' , True , BLACK), (w + 10, 230))




    def _draw_buttons(self):
        for s in self.dict_buttons_text:
            if s == str_step_delay[0]:
                self.screen.blit(self.font.render(s[1]%(1/self.fps/steps),
                                              True, BLACK), self.dict_buttons_text[s][1])
            else:
                self.screen.blit(self.font.render(s[1]%self.sigmoid(self.dict_buttons_text[s][0]),
                                              True, BLACK), self.dict_buttons_text[s][1])
            self.screen.blit(self.font.render(s[0], True, BLACK), self.dict_buttons_text[s][2])
            self.screen.blit(self.font.render('-', True, BLACK), self.dict_buttons_text[s][3])
            self.screen.blit(self.font.render('+', True, BLACK), self.dict_buttons_text[s][4])

        if self.flag_ga or self.flag_run_ga:
            self.screen.blit(self.font_bold.render('Genetic Algorithm', True, NAVY), (w + 20, H - 95))
        else: self.screen.blit(self.font_bold.render('Throw Heuristic', True, NAVY), (w + 20, H - 95))
        self.screen.blit(self.font.render("-Press 't' to switch", True, NAVY), (w + 20, H - 80))
        self.screen.blit(self.font.render(" the throw algorithm.", True, NAVY), (w + 20, H - 65))
        self.screen.blit(self.font.render("-Press 'g' to run only", True, NAVY), (w + 20, H - 50))
        self.screen.blit(self.font.render(" Genetic Algorithm", True, NAVY), (w + 20, H - 35))


    def make_gif(self):
        self.counter += 1
        if self.gif > 0:
            strFormat = 'RGBA'
            raw_str = pygame.image.tostring(self.screen, strFormat, False)
            image = Image.frombytes(
                strFormat, self.screen.get_size(), raw_str)
            self.images.append(image)
            self.gif -= 1
            if self.gif == 0:
                self.images[0].save('basketball{}.gif'.format(self.counter),
                                    save_all=True, append_images=self.images[1:],
                                    optimize=True, duration=1000//self.fps, loop=0)
                self.images = []




class App:
    def __init__(self):
        self.space = pymunk.Space()
        self.space.gravity = 0, 900
        b0 = self.space.static_body
        # self.space.add(b0)
        pygame.init()
        self.clock = pygame.time.Clock()
        self.fps = 60
        self.screen = pygame.display.set_mode(size)
        self.draw_options = DrawOptions(self.screen)
        self.running = True
        self.gif = 0
        self.images = []
        # Box(self.space)
        self.robot = Robot(self.space, size)
        # c.body.apply_impulse_at_local_point((10, 0))

    def run(self):

        while self.running:
            self.robot.update()
            for event in pygame.event.get():
                self.do_event(event)

            self.draw()
            # self.clock.tick(fps)

            for i in range(steps):
                self.space.step(1/fps/steps)
            # self.clock.tick(self.fps)

        pygame.quit()

    def do_event(self, event):
        if event.type == QUIT:
            self.running = False

        if event.type == KEYDOWN:
            if event.key in (K_q, K_ESCAPE):
                self.running = False

            elif event.key == K_p:
                pygame.image.save(self.screen, 'basketball.png')

            elif event.key == K_g:
                self.gif = 360
                # print('innn')
            elif event.key == K_LEFT:
                self.robot.doAction('left')
            elif event.key == K_RIGHT:
                self.robot.doAction('right')
            elif event.key == K_x:
                self.robot.doAction('arms_up')
            elif event.key == K_UP:
                self.robot.doAction('hands_up')
            elif event.key == K_z:
                self.robot.doAction('arms_down')
            elif event.key == K_DOWN:
                self.robot.doAction('hands_down')
            elif event.key == K_u:
                self.robot.doAction('pick_up')
                # self.robot.stage = ['pick_up']
            elif event.key == K_v:
                self.robot.doAction('pow_up')
            elif event.key == K_b:
                print(int((PI/2  - self.robot.angleHands) * 30 / PI) ,
                      int(self.robot.angleArms * 30 / PI), self.robot.pow_throw)
                self.robot.doAction('throw')
            elif event.key == K_a:
                self.robot.throw((600, -PI/2, -PI/3, 800))



    def draw(self):
        self.screen.fill(GRAY)
        self.space.debug_draw(self.draw_options)
        pygame.display.update()

        text = f'fpg: {self.clock.get_fps():.1f}'
        pygame.display.set_caption(text)
        self.make_gif()

    def make_gif(self):
        if self.gif > 0:
            strFormat = 'RGBA'
            raw_str = pygame.image.tostring(self.screen, strFormat, False)
            image = Image.frombytes(
                strFormat, self.screen.get_size(), raw_str)
            self.images.append(image)
            self.gif -= 1
            if self.gif == 0:
                self.images[0].save('basketball.gif',
                                    save_all=True, append_images=self.images[1:],
                                    optimize=True, duration=1000//fps, loop=0)
                self.images = []


if __name__ == "__main__":
    game = BasketBall()
    #game = App() # getting input from the users
    game.run()

