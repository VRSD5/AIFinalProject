import time, random
import area, agent, flowField


SCALE = 2


def generate_env_A():
    env_state = []

    env_state.append(area.RectArea((70*SCALE, 0*SCALE), (25*SCALE, 25*SCALE), "gray"))
    env_state.append(area.RectArea((70*SCALE, 40*SCALE), (25*SCALE, 25*SCALE), "gray"))
    env_state.append(area.RectArea((70*SCALE, 80*SCALE), (25*SCALE, 25*SCALE), "gray"))

    env_state.append(area.RectArea((100*SCALE, 20*SCALE), (25*SCALE, 25*SCALE), "gray"))
    env_state.append(area.RectArea((100*SCALE, 60*SCALE), (25*SCALE, 25*SCALE), "gray"))
    env_state.append(area.RectArea((100*SCALE, 100*SCALE), (25*SCALE, 25*SCALE), "gray"))

    env_state.append(area.RectArea((130*SCALE, 0*SCALE), (25*SCALE, 25*SCALE), "gray"))
    env_state.append(area.RectArea((130*SCALE, 40*SCALE), (25*SCALE, 25*SCALE), "gray"))
    env_state.append(area.RectArea((130*SCALE, 80*SCALE), (25*SCALE, 25*SCALE), "gray"))

    goal = area.RectArea((180*SCALE, 50*SCALE), (20*SCALE, 20*SCALE), "green")

    return env_state, goal, (0, 0, 50*SCALE, 120*SCALE), (0, 0, 200*SCALE, 125*SCALE)


def main():
    envs = ["A"]

    # Agents to test
    agent_types = ["Astar", "HAAgent", "CoordinatedAgent", "Flow", "Simple"]

    # Reduced counts for sanity
    agent_counts = [1, 10, 100, 500, 1000]

    for env in envs:
        for agent_type in agent_types:
            for agent_count in agent_counts:
                time_taken, total_finished, average_steps = run_bench(env, agent_type, agent_count)

                print("----- Benchmark Result -----")
                print(f"Environment     : {env}")
                print(f"Agent Type      : {agent_type}")
                print(f"Agent Count     : {agent_count}")
                print(f"Time Taken      : {(time_taken / 1_000_000_000):.3f} s")
                print(f"Total Finished  : {total_finished}")
                print(f"Average Steps   : {average_steps:.2f}")
                print("----------------------------\n")


def run_bench(env_select, agent_select, agent_count, max_steps=6000):
    match env_select:
        case "A":
            env_state, goal, start_region, region = generate_env_A()

    agents = []
    flow = None

    match agent_select:
        case "Astar":
            for _ in range(agent_count):
                agents.append(agent.Agent(
                    (start_region[0] + random.random() * start_region[2],
                     start_region[1] + random.random() * start_region[3]),
                    env_state, goal))

        case "HAAgent":
            for _ in range(agent_count):
                agents.append(agent.HAAgent(
                    (start_region[0] + random.random() * start_region[2],
                     start_region[1] + random.random() * start_region[3]),
                    env_state, goal))

        case "CoordinatedAgent":
            for _ in range(agent_count):
                agents.append(agent.CoordinatedAgent(
                    (start_region[0] + random.random() * start_region[2],
                     start_region[1] + random.random() * start_region[3]),
                    env_state, goal))

        case "Simple":
            for _ in range(agent_count):
                agents.append(agent.simpleAgent(
                    (start_region[0] + random.random() * start_region[2],
                     start_region[1] + random.random() * start_region[3]),
                    env_state, goal))

        case "Flow":
            # --------------------------------------------
            # IMPORTANT: SCALE DENSITY SO GRID SIZE DOESN'T EXPLODE
            # --------------------------------------------
            flow = flowField.FlowField(
                goal,
                env_state,
                density=2 * SCALE,     # KEY FIX
                region=region
            )

            for _ in range(agent_count):
                agents.append(agent.FlowAgent(
                    (start_region[0] + random.random() * start_region[2],
                     start_region[1] + random.random() * start_region[3]),
                    env_state, goal, flow))

    return benchmark(agents, flow, max_steps=max_steps)


def benchmark(agents, flow=None, max_steps=6000):
    step = 0
    start = time.time_ns()

    if flow is not None:
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
            break

    time_taken = time.time_ns() - start
    total_finished = len(step_counts)

    avg = sum(step_counts) / total_finished if total_finished > 0 else 0

    return time_taken, total_finished, avg


if __name__ == "__main__":
    main()
