import pygame, sys, pymunk

def create_apple(space):
    body = pymunk.Body(1, 100, body_type = pymunk.Body.DYNAMIC)
    body.position = (400, 0)
    shape = pymunk.Circle(body, 80)
    space.add(body, shape)
    return shape

def draw_apples(apples):
    for apple in apples:
        pos_x = int(apple.body.position.x)
        pos_y = int(apple.body.position.y)
        pygame.draw.circle(screen, (0,0,0),(pos_x,pos_y),80)

def static_ball(space):
    body = pymunk.Body(body_type = pymunk.Body.STATIC)
    body.position = (500, 500)
    shape = pymunk.Circle(body, 50)
    space.add(body, shape)
    return shape

def draw_static_ball(balls):
    for ball in balls:
        pos_x = int(ball.body.position.x)
        pos_y = int(ball.body.position.y)
        pygame.draw.circle(screen, (0,0,0),(pos_x,pos_y),80)

pygame.init()
screen = pygame.display.set_mode((800, 600))
clock = pygame.time.Clock()

space = pymunk.Space()
space.gravity = (0,300)
apples = []
apples.append(create_apple(space))
balls = []
balls.append(static_ball(space))


while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.MOUSEBUTTONDOWN:
            apples.append(create_apple(space))
    screen.fill((217,217,217))
    draw_apples(apples)
    draw_static_ball(balls)
    space.step(1/50)
    pygame.display.update()
    clock.tick(120)