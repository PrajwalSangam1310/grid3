from controllor import Controller
from typing import List, Tuple
import pygame
import math
from collections import deque

class ControllerTester:
    GAME_WIDTH = 1000
    GAME_HEIGHT = 500
    FRAME_RATE = 15
    GAME_BACKGROUND_COLOR = (0,0,0)
    MARKER_UNVISITED_COLOR = (255,0,0)
    MARKER_VISITED_COLOR = (0,255,0)
    MARKER_RADIUS = 10
    MARKER_BORDER_WIDTH = 4
    TARGET_THRESHOLD = 20
    
    def __init__(self) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode((ControllerTester.GAME_WIDTH,ControllerTester.GAME_HEIGHT))
        self.clock = pygame.time.Clock()

    def start_test(self,start_point : Tuple[int,int], target_points : List[Tuple[int,int]]) -> None:
        self.running = True
        self.bot = Bot(self.screen, start_point)
        self.target_points = deque(target_points)
        self.visited_points = deque()
        self.current_target_index = 0

        while self.running:
            pygame.draw.rect(self.screen,ControllerTester.GAME_BACKGROUND_COLOR,(0,0,ControllerTester.GAME_WIDTH,ControllerTester.GAME_HEIGHT))
            self.draw_target_points()
            if len(self.target_points) > 0:
                distance_error = self.bot.update_bot_velocity(self.target_points[0])
                self.update_current_target_point(distance_error)
                self.bot.move()
            self.bot.draw_bot()
            self.update_screen()
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.quit()
                    break

    def draw_target_points(self):
        for point in self.target_points:
            pygame.draw.circle(self.screen, ControllerTester.MARKER_UNVISITED_COLOR, point, ControllerTester.MARKER_RADIUS,width=ControllerTester.MARKER_BORDER_WIDTH)
        for point in self.visited_points:
            pygame.draw.circle(self.screen, ControllerTester.MARKER_VISITED_COLOR, point, ControllerTester.MARKER_RADIUS,width=ControllerTester.MARKER_BORDER_WIDTH)
    
    def update_current_target_point(self,distance_error):
        if distance_error < ControllerTester.TARGET_THRESHOLD:
            point = self.target_points.popleft()
            self.visited_points.append(point)

    def update_screen(self) -> None:
        pygame.display.update()
        self.clock.tick(ControllerTester.FRAME_RATE)

    def quit(self) -> None:
        pygame.quit()
        self.running = False

class Bot:
    BOT_RADIUS = 40
    BOT_COLOR = (255,255,255)
    BOT_DIRECTION_POINTER_COLOR = (255,0,0)
    MINIMUM_VELOCITY = 3.0

    def __init__(self, screen : pygame.Surface, start_point : Tuple[int,int]) -> None:
        self.screen = screen
        self.botx = start_point[0]
        self.boty = start_point[1]
        self.dpx = start_point[0] + Bot.BOT_RADIUS/2
        self.dpy = start_point[1]
        self.linear_velocity = 0.0
        self.radial_velocity = 0.0#-0.001
        self.linear_acceleration = 0.0#-0.1
        self.radial_acceleration = 0.0
        self.linear_controller = Controller('linear')
        self.radial_controller = Controller('radial')

    
    def draw_bot(self):
        pygame.draw.circle(self.screen, Bot.BOT_COLOR, (round(self.botx), round(self.boty)), Bot.BOT_RADIUS)
        pygame.draw.circle(
            self.screen, Bot.BOT_DIRECTION_POINTER_COLOR, (round(self.dpx), round(self.dpy)), Bot.BOT_RADIUS/2)
    
    def set_acceleration(self, linear, radial):
        self.linear_acceleration = linear
        self.radial_acceleration = radial
    
    def set_velocity(self, linear, radial):
        self.linear_velocity = linear
        self.radial_velocity = radial
    
    def get_current_location(self):
        return (self.botx,self.boty)
    
    def get_direction_pointer_location(self):
        return (self.dpx,self.dpy)
    
    def move(self):
        self._radial_step()
        self._linear_step()
    
    def _linear_step(self):
        self.linear_velocity += self.linear_acceleration
        D = ((self.botx - self.dpx)**2 + (self.boty - self.dpy)**2)**0.5
        y = (self.dpy-self.boty)*self.linear_velocity/D
        x = (self.dpx-self.botx)*self.linear_velocity/D
        print("Linear Step: ", x, y)
        self.botx += x
        self.boty += y
        self.dpx += x
        self.dpy += y
    
    def _radial_step(self):
        self.radial_velocity += self.radial_acceleration
        sin_theta = math.sin(self.radial_velocity)
        cos_theta = math.cos(self.radial_velocity)
        x1 = self.dpx - self.botx
        y1 = self.dpy - self.boty
        x = cos_theta * x1 - sin_theta * y1 - x1
        y = sin_theta * x1 + cos_theta * y1 - y1
        print("Radial Step: ", self.radial_velocity)
        self.dpx += x
        self.dpy += y
    
    def update_bot_velocity(self, target):
        distance_error = ((target[0] - self.botx)**2 + (target[1] - self.boty)**2)**0.5
        print("Distance Error: ", distance_error)
        linear_velocity = self.linear_controller.step(distance_error)
        linear_velocity = max(linear_velocity,Bot.MINIMUM_VELOCITY)
        
        angle_error = math.atan2((target[1]-self.boty),(target[0]-self.botx)) - math.atan2((self.dpy-self.boty),(self.dpx-self.botx))
        if angle_error > math.pi:
            angle_error -= math.pi * 2
        elif angle_error < -1*math.pi:
            angle_error += math.pi * 2
        print("Angle Error: ", angle_error)
        radial_velocity = self.radial_controller.step(angle_error)

        self.set_velocity(linear_velocity, radial_velocity)
        return distance_error

tester = ControllerTester()

# tester.start_test((40,40),[(100,200),(200,200),(200,300),(400,400)])
# tester.start_test((40,150),[(90,100),(140,200),(190,100),(240,200)])
# tester.start_test((200,200),[(40,200)])
# tester.start_test((500,250),[(400,200),(300,300),(200,200),(100,300)])
# tester.start_test((500,40),[(500,100),(500,200),(500,400),(400,400),(200,400),(100,400)])
tester.start_test((300,200),[(350,250),(400,300),(350,350),(300,400),(250,350),(200,300),(250,250),(300,200)])