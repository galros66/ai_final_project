R_MUT = 0.9
R_CROSS = 0.9
MAX_SHOOTING_POS = 540
POS_CHANGE = 20
MIN_SHOOTING_POS = 120
pop_len = 10

__version__ = "$Id:$"
__docformat__ = "reStructuredText"

# Python imports

# Library imports
import pygame

from pymunk.pygame_util import *

from pygame.locals import *

import random

fps = 30
steps = 10

background_image = pygame.image.load(r'background2.jpeg')

class GA:
    def __init__(self, bb):
        self.bb = bb
        self.space = bb.space
        self.clock = bb.clock
        self.running = True
        self.robot = bb.robot

        self.steps = 0


        self.pop = [] # population

        self.aA, self.bA = 0, self.get_angle_number(self.robot.max_angle_arms, True)
        self.aH, self.bH = 15, self.get_angle_number(self.robot.max_angle_hands, False)
        self.aP, self.bP = 5, 15
        self.range_list = [(self.aA, self.bA), (self.aH, self.bH), (self.aP, self.bP)]
        self.fitness_value = []
        self.xi = 0
        for _ in range(pop_len):
            self.pop.append([self.calculate_angle(random.randint(self.aA, self.bA),True),
                        self.calculate_angle(random.randint(self.aH, self.bH), False),
                        self.robot.pow_change * random.randint(self.aP, self.bP)])

        # self.position_x = 600
        self.position_x = self.robot.shooting_position[0]

        self.res = dict()

    def change_position(self, p):
        if self.position_x == MIN_SHOOTING_POS:
            self.running = False
        self.pop = [p]
        self.position_x -= POS_CHANGE
        for _ in range(pop_len - 1):
            self.pop.append([self.calculate_angle(random.randint(self.aA, self.bA),True),
                        self.calculate_angle(random.randint(self.aH, self.bH), False),
                        self.robot.pow_change * random.randint(self.aP, self.bP)])



    def calculate_angle(self, n, arm):
        if arm: return  max(self.robot.max_angle_arms - n * self.robot.angle_change, self.robot.min_angle_arms)
        return  max(self.robot.max_angle_hands - n * self.robot.angle_change, self.robot.min_angle_hands)

    def get_angle_number(self, angle, arm):
        if arm: return -round((self.robot.min_angle_arms - angle) /self.robot.angle_change)
        return -round((self.robot.min_angle_hands - angle) / self.robot.angle_change)

    def selection(self):
        ix = random.randint(0, len(self.pop) -1)
        for i in range(0, len(self.pop), random.randint(1, len(self.pop) - 2)):
            if self.fitness_value[i] > self.fitness_value[ix]:
                ix = i
        return self.pop[ix]

    def crossover(self, p1, p2, r_cross):
        c1, c2 = p1.copy(), p2.copy()
        if random.random() < r_cross:
            pt = random.randint(1,len(p1) - 2)
            c1 = c1[:pt] + c2[pt:]
            c2 = c2[:pt] + c1[pt:]
        return [c1, c2]

    def mutation(self, p, r_mut):
        for i in range(len(p)):
            # check for a mutation
            if random.random() < r_mut:
                # flip the bit
                if i == 2 : p[i] = random.randint(self.range_list[i][0], self.range_list[i][1]) * 100
                else: p[i] = self.calculate_angle(random.randint(self.range_list[i][0], self.range_list[i][1]), i == 0)
        return p

    def step(self):
        """
        using by the runs function in this class and the Basketball class
        :return: the state that make the robot shots the ball into the basket in the shooting position of the robot
        """
        # If the ball is thrown we will wait for it to stop to make the next shot
        if self.robot.stage == 0:
            self.steps += 1
            # self.bb.stepCount = self.steps
            if self.xi != 0:
                # set the fitness value after the ball spot
                self.fitness_value.append(-abs(self.robot.basket_position[0] - self.robot.closest_point[0]) +
                                          (self.robot.basket_position[1] - self.robot.closest_point[1]))
                # Checks if the ball enters the basket, if so we have completed the part of the genetic algorithm
                # and return the values that give a shot into the basket.
                if self.robot.is_shooting():
                    p = self.pop[(self.xi - 1) % len(self.pop)].copy()
                    self.robot.counter_shoots += 1
                    if self.position_x not in self.res :self.res[self.position_x] = p
                    self.xi = 0
                    return p
                # An extreme case where the ball went into the basket the last time but not in the second time,
                # happens in a situation where the ball almost went in the first time.
                if self.xi - 1 == 0 and self.position_x in self.res: #
                    self.res.pop(self.position_x)
            # After we have all the fitness values implement the rest of the operations of the genetic algorithm
            if self.xi != 0 and self.xi % len(self.pop) == 0:
                selected = [self.selection() for _ in range(len(self.pop))] # selected randomly states from population
                children = [] # the states we will get after the crossover and mutation
                for k in range(0, len(self.pop), 2):
                    # in chance 0.9 make the crossover between 2 state that chosen randomly in the selection function
                    for c in self.crossover(selected[k], selected[k + 1], R_CROSS):
                        # adding the new state after the mutation that appends in chances of 0.9
                        children.append(self.mutation(c, R_MUT))
                self.fitness_value = [] # reset the fitness values for the next iterations of the new states
                self.pop = children # update the population be be the new states
            if self.position_x in self.res: self.robot.throw([self.position_x] + self.res[self.position_x])
            else: self.robot.throw([self.position_x] + self.pop[self.xi % len(self.pop)]) # throw the ball
            self.xi += 1

    def run(self):
        """
        running the genetic algorithm to each possible position the robot can shoot the ball into the basket in screen
        using the draw an do_event functions of BASKETBALL class
        :return: the resolute of the states of each position the robot shots the ball into the basket
        """
        self.robot.stage = 0
        self.position_x = MAX_SHOOTING_POS
        while self.running and self.bb.flag_run_ga:
            self.robot.update()
            for event in pygame.event.get():
                self.bb.do_event(event)

            p = self.step()
            if p :
                self.change_position(self.pop[(self.xi - 1) % len(self.pop)])
            self.bb.draw()

            for i in range(steps):
                self.space.step(1/fps/steps)

        self.running = True
        self._show_shots()

        # self.robot.counter_throws -= self.steps
        # self.bb.stepCount -= self.steps

        return self.res.copy()


    def _show_shots(self):
        """
        called from the run function, returned all the shots in the running
        :return:
        """
        x = MAX_SHOOTING_POS
        while self.running and self.bb.flag_run_ga:
            self.robot.update()
            if self.robot.stage == 0:
                self.robot.throw([x] + self.res[x])
                if x == MIN_SHOOTING_POS: break
                x -= POS_CHANGE
            for event in pygame.event.get():
                self.bb.do_event(event)

            self.bb.mouse = pygame.mouse.get_pos()
            self.bb.draw()
            self.clock.tick(self.bb.fps)

            for i in range(steps):
                self.space.step(1/fps/steps)
            self.clock.tick(self.bb.fps)



if __name__ == '__main__':
    from basketball import BasketBall
    game = GA(BasketBall())
    game.run()