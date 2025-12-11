import time, random
import area, agent


def generate_env_A(dist):
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
    # env_state.append(area.CircleArea((25, 70), 20, "gray"))

    goal = area.RectArea((200 - 30, 50), (20, 20), "green")

    return env_state, goal, (0, 0, 50, 120), (0, 0, 200, 125) #50_000, 1000)


def main():
    envs = ["A"]
    # Updated: Include "Astar" (standard Agent) and "HAAgent" for comparison
    agent_types = ["Group", "HAAgent"]
    # Reduced max agent count for initial tests to manage execution time
    agent_counts = [1, 10, 1000, 10000]
    #dists = [200, 400, 800, 1600, 3200, 10_000]
    dists = [200]
    for env in envs:
        for agent_type in agent_types:
            for agent_count in agent_counts:
                for dist in dists:
                    time_taken, total_finished, average_steps = run_bench(env, agent_type, agent_count, dist)

                    print("----- Benchmark Result -----")
                    print(f"Environment     : {env}")
                    print(f"Agent Type      : {agent_type}")
                    print(f"Agent Count     : {agent_count}")
                    print(f"Time Taken      : {(time_taken / 1_000_000_000):.3f} s")
                    print(f"Total Finished  : {total_finished}")
                    print(f"Average Steps   : {average_steps:.2f}")
                    print("----------------------------\n")


def run_bench(env_select, agent_select, agent_count, dist, max_steps=1000_000):
    match env_select:
        case "A":
            env_state, goal, start_region, region = generate_env_A(dist)

    agents = []
    flow = None

    match agent_select:
        case "Astar":
            for i in range(agent_count):
                # Uses the base Agent, which calls search.astar_search
                agents.append(agent.Agent((start_region[0] + random.random() * start_region[2],
                                           start_region[1] + random.random() * start_region[3]), env_state, goal))
        case "HAAgent":
            for i in range(agent_count):
                # Uses the new HAAgent, which calls hastar.hierarchical_astar_search
                agents.append(agent.HAAgent((start_region[0] + random.random() * start_region[2],
                                             start_region[1] + random.random() * start_region[3]), env_state, goal))
        case "Group":
            for i in range(agent_count):
                # Uses the new HAAgent, which calls hastar.hierarchical_astar_search
                agents.append(agent.CoordinatedAgent((start_region[0] + random.random() * start_region[2],
                                             start_region[1] + random.random() * start_region[3]), env_state, goal))
        case "Simple":
            for i in range(agent_count):
                agents.append(agent.simpleAgent((start_region[0] + random.random() * start_region[2],
                                                 start_region[1] + random.random() * start_region[3]), env_state, goal))


    return benchmark(agents, flow, max_steps=max_steps)


def benchmark(agents, flow=None, max_steps=10000):
    step = 0

    start = time.time_ns()

    if flow != None:
        flow.fit()

    step_counts = []

    while step < max_steps:
        step += 1
        finished = []
        for i in range(len(agents)):
            if agents[i].update():
                step_counts.append(agents[i].steps)
                finished.append(i)
        for i in reversed(finished):
            agents.pop(i)
        if len(agents) == 0:
            step = max_steps

    time_taken = time.time_ns() - start
    total_finished = len(step_counts)
    avg = 0
    if total_finished > 0:
        for i in step_counts:
            avg += i / total_finished

    return time_taken, total_finished, avg


# Check to ensure the main execution block is correct
if __name__ == "__main__":
    main()