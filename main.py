import pygame
import agent, area, search, problem, pathfinder
import random



def demoA(pather):
    env_state = []
    env_state.append(area.RectArea((70, 0), (25, 25), "gray"))
    env_state.append(area.RectArea((70, 40), (25, 25), "gray"))
    env_state.append(area.RectArea((70, 80), (25, 25), "gray"))

    env_state.append(area.RectArea((100, 20), (25, 25), "gray"))
    env_state.append(area.RectArea((100, 60), (25, 25), "gray"))
    env_state.append(area.RectArea((100, 100), (25, 25), "gray"))

    env_state.append(area.RectArea((130, 0), (25, 25), "gray"))
    env_state.append(area.RectArea((130, 40), (25, 25), "gray"))
    env_state.append(area.RectArea((130, 80), (25, 25), "gray"))
    #env_state.append(area.CircleArea((25, 70), 20, "gray"))

    false_goal = area.RectArea((170, 40), (40, 40), "green")
    goal = area.RectArea((180, 50), (20, 20), "green")

    method = search.astar_search


    agents = []
    for i in range(100):
        agents.append(agent.Agent((random.random() * 50,random.random() * 120), env_state, goal, pather))
    
    render_scale = 5
    render_offset = (0,0)

    return env_state, false_goal, goal, method, agents, render_scale, render_offset

def gameloop(screen):
    run = True
    clock = pygame.time.Clock()

    # pather = pathfinder.Pathfinder(search.astar_search)
    pather = pathfinder.Pathfinder(search.astar_search, use_hierarchical=True)

    demo = "A"
    match demo:
        case "A":
            env_state, false_goal, goal, method, agents, render_scale, render_offset = demoA(pather)

    

    while run:
        for i in pygame.event.get():
            if i.type == pygame.QUIT:
                run = False

        # if len(pather.queue) != 0:
        #     for i in range(1):
        #         pather.pop_queue()
        if len(pather.queue) != 0:
            pather.process_group(5)  # Process 5 paths per frame

        for i in agents:
            if i.update():
                del i
        


        screen.fill("black")

        for i in env_state:
            i.render(screen, render_offset, render_scale)

        false_goal.render(screen, render_offset, render_scale)

        for i in agents:
            i.render(screen, render_offset, render_scale, render_path=True)
        
        
        clock.tick(60)
        pygame.display.update()
        #pygame.event.wait()


def main():
    pygame.init()

    screen = pygame.display.set_mode((1000,600))
    pygame.display.set_caption("Testing")
    gameloop(screen)

    

if __name__=="__main__":
    main()
    