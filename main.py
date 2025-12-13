"""
main.py

Interactive visualization and demo runner for multi-agent navigation
experiments using pygame.

This file supports:
- Density-based obstacle environments
- Multiple agent types (Flow, HA*, Group / WHCA*)
- Command-line configuration for agent count, scale, and density
- Real-time rendering and simulation updates

Intended for qualitative evaluation and debugging rather than benchmarking.
"""
import pygame
import agent, area, search, problem, pathfinder, flowField
import random
import getopt, sys


def density_env(dist=200, density = 30):
    env_state = []
    flip = True
    for x in range(70, dist - 60, density):
        for y in range(0 if flip else -density//2, dist//2, density):
            env_state.append(area.RectArea((x, y), (25, 25), "gray"))
        flip = not flip
    
    
    false_goal = area.RectArea((dist - 40, dist/4 - 20), (40, 40), "green")
    goal = area.RectArea((dist - 30, dist/4 - 10), (20, 20), "green")

    
    render_scale = 1000/dist
    render_offset = (0,0)

    return env_state, false_goal, goal, render_scale, render_offset

def demo_env(dist=200):
    #This seems to be cursed idk
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
    

    false_goal = area.RectArea((dist - 40, dist/4 - 20), (40, 40), "green")
    goal = area.RectArea((dist - 30, dist/4 - 10), (20, 20), "green")


    render_scale = 1000/dist
    render_offset = (0,0)

    return env_state, false_goal, goal, render_scale, render_offset

def gameloop(screen):
    run = True
    clock = pygame.time.Clock()

    pather = pathfinder.Pathfinder(search.astar_search)
    
    args = sys.argv[1:]
    options = "t:c:s:d:"
    long_options = ["Type=", "Count=", "Scale=", "Density="]

    agent_type = "Flow"
    agent_count = 100
    dist = 500
    density = 100

    try:
        arguments, values = getopt.getopt(args, options, long_options)
        for currentArg, currentVal in arguments:
            if currentArg in ("-t", "--Type"):
                if not currentVal in ("Deffered", "Flow", "HAstar", "Group"):
                    raise ValueError
                agent_type = currentVal
            elif currentArg in ("-c", "--Count"):
                if int(agent_count) < 1:
                    raise ValueError
                agent_count = int(currentVal)
            elif currentArg in ("-s", "--Scale"):
                if int(currentVal) < 100:
                    raise ValueError
                dist = int(currentVal)
            elif currentArg in ("-d", "--Density"):
                if int(currentVal) < 30:
                    raise ValueError
                density = int(currentVal)
    except getopt.error as err:
        print(str(err))

    env = "Density"
    
    if env == "Demo":
        dist = 200

    match env:
        case "Demo":
            env_state, false_goal, goal, render_scale, render_offset = demo_env(dist = dist)
        case "Density":
            env_state, false_goal, goal, render_scale, render_offset = density_env(dist = dist, density=density)


    
    
    agents = []
    match agent_type:
        case "Deffered":
            for i in range(agent_count):
                agents.append(agent.DefferedAgent((random.random() * 50,random.random() * dist/2), env_state, goal))
        case "Flow":
            flow = flowField.FlowField(goal, env_state, density = 2, region = (0,0, dist, dist/2))
            for i in range(agent_count):
                agents.append(agent.FlowAgent((random.random() * 50,random.random() * dist/2), env_state, goal, flow))
        case "HAstar":
            for i in range(agent_count):
                agents.append(agent.HAAgent((random.random() * 50,random.random() * dist/2), env_state, goal))
        case "Group":
            for i in range(agent_count):
                agents.append(agent.CoordinatedAgent((random.random() * 50,random.random() * dist/2), env_state, goal))



    render = False

    while run:
        for i in pygame.event.get():
            if i.type == pygame.QUIT:
                run = False

        if len(pather.queue) != 0:
            for i in range(1):
                pather.pop_queue()
            print("test")

        for i in agents:
            if i.update():
                del i
        


        screen.fill("black")

        for i in env_state:
            i.render(screen, render_offset, render_scale)

        false_goal.render(screen, render_offset, render_scale)

        for i in agents:
            i.render(screen, render_offset, render_scale, render_path=False)
        
        #flow.render(screen, scale = 5)
        
        clock.tick(60)
        pygame.display.update()
        #pygame.event.wait()


def main():
    pygame.init()

    screen = pygame.display.set_mode((1000,500))
    pygame.display.set_caption("Testing")
    gameloop(screen)

    




if __name__=="__main__":



    main()
    