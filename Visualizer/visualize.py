import pygame, math, sys
from pygame.locals import *

# size of the car
CAR_X = 22
CAR_Y = 29

# Screen resolution
RESX = 300
RESY = 750

# Grid specifications
NUMBER_LANES = 2
WIDTH_LANE = 35
GRID_HEIGHT = 45
GRID = 15

GAME_Y = GRID * GRID_HEIGHT
BASE_POS = ((RESX+WIDTH_LANE) / 2.0 - CAR_X / 2.0, -GRID_HEIGHT/2 + (RESY+GAME_Y)/2 - CAR_Y / 2.0)

# open the data file
data_file = open(sys.argv[1], 'r')

#Start 'game'
pygame.init()
screen = pygame.display.set_mode((RESX, RESY))

OWN_CAR = pygame.image.load('car1.png')
DUMMY_CAR = pygame.image.load('car2.png')
clock = pygame.time.Clock()

WHITE = (255, 255, 255)
BLUE =  (  0,   0, 255)

isPaused = False
car_positions = []

coordS = []
coordE = []
for i in range(0, GRID+1):
    coordS.append([RESX/2-WIDTH_LANE, i*GRID_HEIGHT+(RESY-GAME_Y)/2])
    coordE.append([RESX/2+WIDTH_LANE, i*GRID_HEIGHT+(RESY-GAME_Y)/2])
    
def position(lane, gridpos):
        ADD_POS = (lane*(-WIDTH_LANE),(gridpos-1)*(-GRID_HEIGHT))
        return tuple(map(sum,zip(BASE_POS,ADD_POS)))
        
lane = 1
grid = 7


# main
while True:
    clock.tick(1)
    for event in pygame.event.get():
        if not hasattr(event, 'key'): continue
        down = event.type == KEYDOWN # key down or up ?
        if(not down):
            speed = 0
        else:
            if event.key == K_SPACE:
                isPaused = not isPaused
            elif event.key == K_ESCAPE:
                sys.exit(0)
    screen.fill(WHITE)
    
    #Display your car
    screen.blit(OWN_CAR, position(1,8))   

    #Display other cars            
    line_car_timestamp = data_file.readline()
    if not line_car_timestamp:
        isPaused = True
       
    if not isPaused:
        car_positions = []
        while True:
            line = data_file.readline()
            if "#" in line:
                break
            car_pos = line.split(' ')
            car_positions.append((int(car_pos[0]), int(car_pos[1])))

        
    for (car_posx, car_posy) in car_positions:
        screen.blit(DUMMY_CAR, position(car_posx,car_posy))


    # Display the gird
    pygame.draw.line(screen, BLUE, [RESX/2-WIDTH_LANE,(RESY-GAME_Y)/2], [RESX/2-WIDTH_LANE,(RESY+GAME_Y)/2],1)
    pygame.draw.line(screen, BLUE, [RESX/2,(RESY-GAME_Y)/2], [RESX/2,(RESY+GAME_Y)/2],1)
    pygame.draw.line(screen, BLUE, [RESX/2+WIDTH_LANE,(RESY-GAME_Y)/2], [RESX/2+WIDTH_LANE,(RESY+GAME_Y)/2],1)
    for i in range(0, GRID + 1):
        pygame.draw.line(screen, BLUE, coordS[i], coordE[i], 1)


    pygame.display.flip()
    
