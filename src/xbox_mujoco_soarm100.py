# xbox_mujoco_soarm100.py
import multiprocessing
from utils.simulation_process import simulation_loop
from utils.viewer_process import viewer_loop


if __name__ == "__main__":
    # Create a queue to share state between processes
    state_queue = multiprocessing.Queue()

    # Create and start the simulation process
    sim_process = multiprocessing.Process(target=simulation_loop, args=(state_queue,))
    sim_process.start()

    # Create and start the viewer process
    vis_process = multiprocessing.Process(target=viewer_loop, args=(state_queue,))
    vis_process.start()

    try:
        # Wait for both processes to complete
        sim_process.join()
        vis_process.join()
    except KeyboardInterrupt:
        # Terminate both processes if interrupted
        sim_process.terminate()
        vis_process.terminate()
